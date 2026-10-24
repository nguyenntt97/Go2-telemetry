"""What to visualize, and how each topic is logged to Rerun.

Telemetry is grouped the way someone choosing what to look at thinks about it
(pose, imu, feet, battery, uwb, odom, clouds, video, events, jetson) rather
than one group per topic. Several groups read the same topic -- `/lf/lowstate`
feeds imu, feet and battery -- so the runner collects handlers by topic and
decodes each message once.
"""

from __future__ import annotations

import csv
import datetime
import json
import math
import os
import re
from dataclasses import dataclass, field

import rerun as rr

from go2export.clouds import cloud_to_arrays, numpy_available
from go2export.decode import DecodeError
from go2export.video import VideoError, is_keyframe, parse_frame

WORLD = "/world"
ROBOT = "/world/robot"
PLOTS = "/plots"
EVENTS = "/events"
VIDEO = "/video"

#: Go2 body, roughly, for something to hang the pose on.
_BODY_HALF_SIZE = (0.35, 0.155, 0.09)


@dataclass
class Options:
    """Everything the handlers can be told to do differently."""

    # The UWB angle fields are not documented, so the mapping is a parameter,
    # exactly as in uwb_rviz/. See "UWB calibration session" in the README.
    bearing_field: str = "orientation_est"
    bearing_sign: float = 1.0
    bearing_offset: float = 0.0
    elevation_field: str = "pitch_est"
    quaternion_order: str = "wxyz"
    cloud_stride: int = 1
    max_cloud_points: int = 0
    trail_every: int = 25
    trail_length: int = 600
    start_seconds: float = 0.0
    duration_seconds: float = 0.0


@dataclass
class Context:
    options: Options = field(default_factory=Options)
    start_ns: int = 0
    trail: list = field(default_factory=list)
    counters: dict = field(default_factory=dict)
    errors: dict = field(default_factory=dict)
    _seen: set = field(default_factory=set)
    _video_started: set = field(default_factory=set)

    def count(self, key: str, amount: int = 1) -> None:
        self.counters[key] = self.counters.get(key, 0) + amount

    def error(self, key: str, message: str) -> None:
        self.errors.setdefault(key, message)
        self.count(f"{key}:errors")

    def once(self, key: str) -> bool:
        """True the first time it is called with a given key."""
        if key in self._seen:
            return False
        self._seen.add(key)
        return True


def _as_float(value, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return default
    return number if math.isfinite(number) else default


def _scalars(path: str, values, names) -> None:
    for name, value in zip(names, values or []):
        rr.log(f"{path}/{name}", rr.Scalars(_as_float(value)))


def _triple(path: str, values) -> None:
    _scalars(path, values, ("x", "y", "z"))


# --- pose --------------------------------------------------------------------


def quaternion_xyzw(quaternion, order: str = "wxyz"):
    """Reorder a quaternion for Rerun, which wants x,y,z,w.

    Unitree sends w,x,y,z. Returns None for a missing or all-zero
    quaternion, which would otherwise be an invalid rotation.
    """
    if not quaternion or len(quaternion) < 4:
        return None
    values = [_as_float(v) for v in quaternion[:4]]
    if not any(values):
        return None
    if order == "wxyz":
        w, x, y, z = values
    else:
        x, y, z, w = values
    return [x, y, z, w]


def _rotation(quaternion, order: str):
    xyzw = quaternion_xyzw(quaternion, order)
    return None if xyzw is None else rr.Quaternion(xyzw=xyzw)


def handle_pose(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    options = ctx.options
    if ctx.once("pose:body"):
        rr.log(
            f"{ROBOT}/body",
            rr.Boxes3D(half_sizes=[_BODY_HALF_SIZE], colors=[(120, 140, 170)]),
            static=True,
        )

    position = [_as_float(v) for v in (message.get("position") or [0, 0, 0])[:3]]
    rotation = _rotation((message.get("imu_state") or {}).get("quaternion"), options.quaternion_order)
    rr.log(ROBOT, rr.Transform3D(translation=position, quaternion=rotation))

    ctx.trail.append(position)
    if len(ctx.trail) > options.trail_length:
        del ctx.trail[: len(ctx.trail) - options.trail_length]
    ctx.count("pose")
    if options.trail_every > 0 and ctx.counters["pose"] % options.trail_every == 0:
        if len(ctx.trail) > 1:
            rr.log(f"{WORLD}/trail", rr.LineStrips3D([list(ctx.trail)], colors=[(90, 170, 240)]))

    _triple(f"{PLOTS}/velocity", message.get("velocity"))
    rr.log(f"{PLOTS}/yaw_speed", rr.Scalars(_as_float(message.get("yaw_speed"))))
    rr.log(f"{PLOTS}/body_height", rr.Scalars(_as_float(message.get("body_height"))))
    rr.log(f"{PLOTS}/gait/type", rr.Scalars(_as_float(message.get("gait_type"))))
    rr.log(f"{PLOTS}/gait/mode", rr.Scalars(_as_float(message.get("mode"))))
    rr.log(
        f"{PLOTS}/gait/foot_raise_height",
        rr.Scalars(_as_float(message.get("foot_raise_height"))),
    )
    obstacle = message.get("range_obstacle")
    if obstacle:
        _scalars(f"{PLOTS}/range_obstacle", obstacle, [str(i) for i in range(len(obstacle))])


# --- imu ---------------------------------------------------------------------


def handle_imu(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    source = topic.strip("/").replace("/", "_")
    imu = message.get("imu_state")
    if isinstance(imu, dict):
        _triple(f"{PLOTS}/imu/{source}/gyro", imu.get("gyroscope"))
        _triple(f"{PLOTS}/imu/{source}/accel", imu.get("accelerometer"))
        _scalars(f"{PLOTS}/imu/{source}/rpy", imu.get("rpy"), ("roll", "pitch", "yaw"))
        if imu.get("temperature") is not None:
            rr.log(
                f"{PLOTS}/imu/{source}/temperature",
                rr.Scalars(_as_float(imu.get("temperature"))),
            )
        ctx.count("imu")
        return

    # sensor_msgs/Imu, as /utlidar/imu publishes.
    angular = message.get("angular_velocity")
    linear = message.get("linear_acceleration")
    if isinstance(angular, dict):
        _triple(f"{PLOTS}/imu/{source}/gyro", [angular.get("x"), angular.get("y"), angular.get("z")])
    if isinstance(linear, dict):
        _triple(f"{PLOTS}/imu/{source}/accel", [linear.get("x"), linear.get("y"), linear.get("z")])
    ctx.count("imu")


# --- feet --------------------------------------------------------------------


def handle_feet(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    force = message.get("foot_force")
    if force:
        _scalars(f"{PLOTS}/feet/force", force, [str(i) for i in range(len(force))])
        ctx.count("feet")

    flat = message.get("foot_position_body")
    if flat and len(flat) >= 12:
        points = [[_as_float(flat[i * 3 + a]) for a in range(3)] for i in range(4)]
        rr.log(
            f"{ROBOT}/feet",
            rr.Points3D(
                points,
                radii=0.025,
                colors=[(240, 180, 70)] * 4,
                labels=["FR", "FL", "RR", "RL"],
            ),
        )
        ctx.count("feet")


# --- battery -----------------------------------------------------------------


def handle_battery(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    bms = message.get("bms_state")
    if isinstance(bms, dict):
        for name in ("soc", "current", "cycle", "status"):
            if bms.get(name) is not None:
                rr.log(f"{PLOTS}/battery/{name}", rr.Scalars(_as_float(bms.get(name))))
        cells = bms.get("cell_vol") or []
        for index, value in enumerate(cells):
            rr.log(f"{PLOTS}/battery/cell/{index:02d}", rr.Scalars(_as_float(value)))
        ctx.count("battery")
    for name in ("power_v", "power_a"):
        if message.get(name) is not None:
            rr.log(f"{PLOTS}/battery/{name}", rr.Scalars(_as_float(message.get(name))))

    # /lf/battery_alarm carries per cell voltages as JSON in a string field.
    text = message.get("data")
    if isinstance(text, str) and text.strip().startswith("{"):
        try:
            parsed = json.loads(text)
        except ValueError:
            return
        for key, value in parsed.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                rr.log(f"{PLOTS}/battery/alarm/{key}", rr.Scalars(float(value)))
        ctx.count("battery")


# --- uwb ---------------------------------------------------------------------


def uwb_bearing(message: dict, options: Options) -> float:
    return (
        options.bearing_sign * _as_float(message.get(options.bearing_field))
        + options.bearing_offset
    )


def uwb_point(message: dict, options: Options) -> list:
    """Where the remote is, in the robot's frame.

    Which field is the bearing and which the elevation is not documented, so
    it comes from the options, the same way uwb_rviz/ parameterizes it.
    """
    distance = _as_float(message.get("distance_est"))
    bearing = uwb_bearing(message, options)
    elevation = (
        0.0
        if options.elevation_field in ("none", "", None)
        else _as_float(message.get(options.elevation_field))
    )
    horizontal = distance * math.cos(elevation)
    return [
        horizontal * math.cos(bearing),
        horizontal * math.sin(bearing),
        distance * math.sin(elevation),
    ]


def handle_uwb(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    options = ctx.options
    point = uwb_point(message, options)
    bearing = uwb_bearing(message, options)

    rr.log(
        f"{ROBOT}/uwb_tag",
        rr.Points3D([point], radii=0.08, colors=[(240, 90, 150)], labels=["remote"]),
    )
    rr.log(f"{ROBOT}/uwb_bearing", rr.LineStrips3D([[[0, 0, 0], point]], colors=[(240, 90, 150)]))

    # Every angle field, so the calibration session can be read straight off
    # the plots.
    for name in (
        "distance_est", "orientation_est", "pitch_est", "yaw_est",
        "tag_roll", "tag_pitch", "tag_yaw",
        "base_roll", "base_pitch", "base_yaw",
    ):
        if message.get(name) is not None:
            rr.log(f"{PLOTS}/uwb/{name}", rr.Scalars(_as_float(message.get(name))))
    rr.log(f"{PLOTS}/uwb/derived/bearing_used", rr.Scalars(bearing))
    joystick = message.get("joystick")
    if joystick:
        _scalars(f"{PLOTS}/uwb/joystick", joystick, [str(i) for i in range(len(joystick))])
    for name in ("error_state", "enabled_from_app", "buttons", "channel", "joy_mode"):
        if message.get(name) is not None:
            rr.log(f"{PLOTS}/uwb/{name}", rr.Scalars(_as_float(message.get(name))))
    ctx.count("uwb")


# --- odometry ----------------------------------------------------------------


def _find_pose(message: dict) -> dict | None:
    """Dig the pose out of Odometry, PoseStamped or a bare Pose."""
    node = message.get("pose")
    for _ in range(3):
        if not isinstance(node, dict):
            break
        if "position" in node:
            return node
        node = node.get("pose")
    if "position" in message:
        return message
    return None


def handle_odom(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    pose = _find_pose(message)
    if not pose:
        return
    position = pose.get("position") or {}
    orientation = pose.get("orientation") or {}
    name = topic.strip("/").replace("/", "_")
    translation = [_as_float(position.get(axis)) for axis in ("x", "y", "z")]
    quaternion = None
    if orientation:
        quaternion = _rotation(
            [orientation.get("w"), orientation.get("x"), orientation.get("y"), orientation.get("z")],
            "wxyz",
        )
    rr.log(f"{WORLD}/odom/{name}", rr.Transform3D(translation=translation, quaternion=quaternion))
    rr.log(
        f"{WORLD}/odom/{name}/point",
        rr.Points3D([translation], radii=0.03, colors=[(140, 220, 160)]),
    )
    _triple(f"{PLOTS}/odom/{name}/position", translation)

    twist = message.get("twist")
    for _ in range(2):
        if isinstance(twist, dict) and "linear" not in twist:
            twist = twist.get("twist")
        else:
            break
    if isinstance(twist, dict):
        linear = twist.get("linear") or {}
        angular = twist.get("angular") or {}
        if linear:
            _triple(f"{PLOTS}/odom/{name}/linear", [linear.get(a) for a in ("x", "y", "z")])
        if angular:
            _triple(f"{PLOTS}/odom/{name}/angular", [angular.get(a) for a in ("x", "y", "z")])
    ctx.count("odom")


# --- point clouds ------------------------------------------------------------

_RAMP = (
    (59, 76, 192), (112, 165, 239), (160, 214, 174), (242, 203, 91), (180, 54, 38),
)


def _intensity_colors(values):
    """Colour points by intensity with a small blue to red ramp."""
    import numpy as np

    data = np.asarray(values, dtype="float64").reshape(-1)
    finite = data[np.isfinite(data)]
    if finite.size == 0:
        return None
    low, high = np.percentile(finite, (2.0, 98.0))
    if not np.isfinite(low) or not np.isfinite(high) or high <= low:
        return None
    position = np.clip((data - low) / (high - low), 0.0, 1.0) * (len(_RAMP) - 1)
    lower = np.floor(position).astype("int32")
    upper = np.clip(lower + 1, 0, len(_RAMP) - 1)
    weight = (position - lower)[:, None]
    ramp = np.asarray(_RAMP, dtype="float64")
    blended = ramp[lower] * (1.0 - weight) + ramp[upper] * weight
    return blended.astype("uint8")


def handle_cloud(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    import numpy as np

    options = ctx.options
    key = f"cloud:{topic}"
    ctx.count(f"{key}:seen")
    if options.cloud_stride > 1 and (ctx.counters[f"{key}:seen"] - 1) % options.cloud_stride:
        return
    try:
        arrays = cloud_to_arrays(message)
    except Exception as exc:  # noqa: BLE001
        ctx.error(key, str(exc))
        return
    if not {"x", "y", "z"} <= set(arrays):
        ctx.error(key, "cloud has no x/y/z fields")
        return

    points = np.stack(
        [np.asarray(arrays[a], dtype="float32").reshape(-1) for a in ("x", "y", "z")], axis=1
    )
    keep = np.isfinite(points).all(axis=1)
    points = points[keep]
    intensity = arrays.get("intensity")
    if intensity is not None:
        intensity = np.asarray(intensity).reshape(-1)[keep]
    if options.max_cloud_points and len(points) > options.max_cloud_points:
        step = int(np.ceil(len(points) / options.max_cloud_points))
        points = points[::step]
        if intensity is not None:
            intensity = intensity[::step]

    colors = _intensity_colors(intensity) if intensity is not None and len(intensity) else None
    # cloud_base is already in the robot's frame, so it hangs off the robot
    # transform; the others are in the lidar/odom frame.
    parent = ROBOT if topic.endswith("cloud_base") else WORLD
    path = f"{parent}/{topic.strip('/').replace('/', '_')}"
    rr.log(path, rr.Points3D(points, radii=0.012, colors=colors))
    ctx.count(key)


# --- video -------------------------------------------------------------------


def handle_video(ctx: Context, topic: str, t_ns: int, blob: bytes) -> None:
    try:
        frame = parse_frame(blob)
    except VideoError as exc:
        ctx.error("video", str(exc))
        return

    resolution = frame["resolution"]
    path = f"{VIDEO}/{resolution}p"
    keyframe = is_keyframe(frame["payload"])
    if path not in ctx._video_started:
        # Nothing before the first keyframe can be decoded.
        if not keyframe:
            ctx.count(f"video:{resolution}p:dropped")
            return
        rr.log(path, rr.VideoStream(codec=rr.VideoCodec.H264), static=True)
        ctx._video_started.add(path)
    rr.log(path, rr.VideoStream.from_fields(sample=frame["payload"], is_keyframe=keyframe))
    ctx.count(f"video:{resolution}p")


# --- events ------------------------------------------------------------------


def handle_events(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    name = topic.strip("/").replace("/", "_")
    text = message.get("data")
    if isinstance(text, str) and text:
        rr.log(f"{EVENTS}/{name}", rr.TextLog(text, level=rr.TextLogLevel.INFO))
        ctx.count("events")
        return

    # /api/*/request and /response, /multiplestate and friends: show whatever
    # string fields they carry, which for the api topics is JSON.
    parts = [
        f"{key}={value}"
        for key, value in message.items()
        if isinstance(value, (str, int, float, bool)) and value not in ("", None)
    ]
    if parts:
        rr.log(f"{EVENTS}/{name}", rr.TextLog(" ".join(parts)[:2000], level=rr.TextLogLevel.DEBUG))
        ctx.count("events")


def handle_controller(ctx: Context, topic: str, t_ns: int, message: dict) -> None:
    name = topic.strip("/").replace("/", "_")
    for field_name in ("lx", "ly", "rx", "ry"):
        if message.get(field_name) is not None:
            rr.log(f"{PLOTS}/controller/{name}/{field_name}", rr.Scalars(_as_float(message[field_name])))
    if message.get("keys") is not None:
        rr.log(f"{PLOTS}/controller/{name}/keys", rr.Scalars(_as_float(message.get("keys"))))
    ctx.count("controller")


# --- the Jetson's own hardware log ------------------------------------------

_TIME_COLUMN = re.compile(r"^(t|time|timestamp|unix|epoch|t_ns|stamp)$", re.I)


def log_hardware_csv(ctx: Context, path: str) -> int:
    """`meta/hardware.csv`, so dropouts can be put next to Jetson load.

    The column names are not pinned down here: every numeric column is logged
    under its own name.
    """
    if not os.path.isfile(path):
        return 0
    rows = 0
    with open(path, newline="", encoding="utf-8", errors="replace") as handle:
        reader = csv.DictReader(handle)
        if not reader.fieldnames:
            return 0
        time_column = next((c for c in reader.fieldnames if _TIME_COLUMN.match(c.strip())), None)
        for index, row in enumerate(reader):
            seconds = None
            if time_column:
                raw = (row.get(time_column) or "").strip()
                seconds = _parse_time(raw)
            if seconds is None:
                # hardware.csv is written once a second.
                seconds = ctx.start_ns / 1e9 + index
            set_time(ctx, int(seconds * 1e9))
            for column, value in row.items():
                if column is None or column == time_column:
                    continue
                text = (value or "").strip()
                if not text:
                    continue
                try:
                    number = float(text)
                except ValueError:
                    continue
                if math.isfinite(number):
                    safe = re.sub(r"[^0-9A-Za-z_.-]", "_", column.strip())
                    rr.log(f"{PLOTS}/jetson/{safe}", rr.Scalars(number))
            rows += 1
    ctx.count("jetson", rows)
    return rows


def _parse_time(raw: str) -> float | None:
    if not raw:
        return None
    try:
        number = float(raw)
    except ValueError:
        try:
            return datetime.datetime.fromisoformat(raw).timestamp()
        except ValueError:
            return None
    # Guess the unit from the magnitude: seconds, millis or nanos since epoch.
    if number > 1e17:
        return number / 1e9
    if number > 1e14:
        return number / 1e6
    if number > 1e11:
        return number / 1e3
    return number


# --- groups ------------------------------------------------------------------


@dataclass(frozen=True)
class Group:
    key: str
    title: str
    match: str
    handler: object = None
    raw: bool = False
    needs_numpy: bool = False
    from_files: bool = False

    @property
    def pattern(self):
        return re.compile(self.match)


GROUPS = (
    Group("pose", "Robot pose, velocity and gait", r"^/sportmodestate$", handle_pose),
    Group("imu", "IMU of the body and the lidar", r"^(/lf/lowstate|/utlidar/imu)$", handle_imu),
    Group("feet", "Foot forces and foot positions", r"^(/lf/lowstate|/sportmodestate)$", handle_feet),
    Group("battery", "Battery, current and cell voltages", r"^(/lf/lowstate|/lf/battery_alarm)$", handle_battery),
    Group("uwb", "UWB remote position and angles", r"^/uwbstate$", handle_uwb),
    Group(
        "odom",
        "Lidar and SLAM odometry",
        r"^(/utlidar/robot_odom|/utlidar/robot_pose|/lio_sam_ros2/mapping/odometry|/uslam/.*)$",
        handle_odom,
    ),
    Group(
        "clouds",
        "Lidar point clouds",
        r"^/utlidar/(cloud|cloud_base|cloud_deskewed)$",
        handle_cloud,
        needs_numpy=True,
    ),
    Group("video", "Front camera, 720p and 360p", r"^/frontvideostream$", handle_video, raw=True),
    Group(
        "events",
        "Annotations, api calls and status",
        r"^(/annotations|/api/.*/(request|response)|/multiplestate|/slam_info)$",
        handle_events,
    ),
    Group("controller", "Wireless controller input", r"^/wirelesscontroller$", handle_controller),
    Group("jetson", "Jetson load from meta/hardware.csv", r"", None, from_files=True),
)

GROUPS_BY_KEY = {group.key: group for group in GROUPS}


def available_groups(topics, session_path: str) -> list:
    """Which groups this session actually has data for.

    Returns ``[(group, [topic, ...])]``, keeping GROUPS order.
    """
    out = []
    for group in GROUPS:
        if group.from_files:
            if os.path.isfile(os.path.join(session_path, "meta", "hardware.csv")):
                out.append((group, ["meta/hardware.csv"]))
            continue
        pattern = group.pattern
        matched = sorted(name for name in topics if pattern.search(name))
        if matched:
            out.append((group, matched))
    return out


def set_time(ctx: Context, t_ns: int) -> None:
    rr.set_time("receive", timestamp=t_ns / 1e9)
    rr.set_time("session", duration=(t_ns - ctx.start_ns) / 1e9)


def _in_window(ctx: Context, t_ns: int) -> bool:
    options = ctx.options
    if options.start_seconds <= 0 and options.duration_seconds <= 0:
        return True
    offset = (t_ns - ctx.start_ns) / 1e9
    if offset < options.start_seconds:
        return False
    if options.duration_seconds > 0 and offset > options.start_seconds + options.duration_seconds:
        return False
    return True


def stream(reader, decoder, groups, ctx: Context, session_path: str, progress=None) -> None:
    """Log every selected group to the active recording.

    Handlers are collected per topic so a topic feeding several groups, as
    `/lf/lowstate` does, is read and decoded once.
    """
    per_topic: dict = {}
    file_groups = []
    for group in groups:
        if group.from_files:
            file_groups.append(group)
            continue
        pattern = group.pattern
        for topic in reader.active_topics():
            if pattern.search(topic):
                per_topic.setdefault(topic, []).append(group)

    for topic, topic_groups in sorted(per_topic.items()):
        info = reader.topics[topic]
        raw_only = all(group.raw for group in topic_groups)
        decoded_groups = [group for group in topic_groups if not group.raw]
        raw_groups = [group for group in topic_groups if group.raw]
        if progress:
            progress(topic, info)

        for t_ns, blob in reader.messages(topic):
            if not _in_window(ctx, t_ns):
                continue
            set_time(ctx, t_ns)
            for group in raw_groups:
                group.handler(ctx, topic, t_ns, blob)
            if raw_only:
                continue
            try:
                message = decoder.decode(blob, info.type)
            except DecodeError as exc:
                ctx.error(topic, str(exc))
                continue
            for group in decoded_groups:
                try:
                    group.handler(ctx, topic, t_ns, message)
                except Exception as exc:  # noqa: BLE001 - one topic must not stop the rest
                    ctx.error(f"{topic}:{group.key}", str(exc))

    for group in file_groups:
        if progress:
            progress("meta/hardware.csv", None)
        log_hardware_csv(ctx, os.path.join(session_path, "meta", "hardware.csv"))
