#!/usr/bin/env python3
"""Serve a recorded Go2 session to Rerun, for a viewer on another machine.

    python3 viz/serve.py                          # pick session and telemetry
    python3 viz/serve.py logs/<session>           # pick telemetry
    python3 viz/serve.py logs/<session> -t body,uwb,cloud
    python3 viz/serve.py -t all --port 9876 --ip 100.64.113.51

The session is read straight from the rosbag2 SQLite files (see bag.py), decoded by
cdr.py, and pushed into a Rerun gRPC server. The viewer connects over the network:

    rerun --connect rerun+http://<this-host>:9876/proxy

Data is held in the server's memory, so a viewer that connects late still gets the
whole session.
"""
import argparse
import os
import re
import socket
import struct
import sys
import time

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

import bag

LOGS = "/media/nguyen/project_data/go2/logs"

# Conventional Unitree leg order for the 12 driven joints and the 4 feet.
LEGS = ["FR", "FL", "RR", "RL"]
JOINTS = [f"{leg}_{part}" for leg in LEGS for part in ("hip", "thigh", "calf")]


def say(text):
    print(text, flush=True)


def lan_address(override=None):
    """This machine's address on the network a remote viewer would come from.

    rr.serve_grpc listens on every interface but reports 127.0.0.1, which is no use
    to another machine, so the address is worked out here. Connecting a UDP socket
    towards a public address reveals which local interface routes outwards without
    sending anything.
    """
    if override:
        return override
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 53))
        return s.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())
    finally:
        s.close()


# --- helpers ------------------------------------------------------------------

def _tc(t):
    """Seconds since session start -> a Rerun time column."""
    return rr.TimeColumn("time", duration=t)


def series(path, t, values, label=None):
    """Log one scalar series columnwise."""
    v = np.asarray(values, dtype=np.float64)
    if label:
        rr.log(path, rr.SeriesLines(names=[label]), static=True)
    rr.send_columns(path, [_tc(t)], rr.Scalars.columns(scalars=v))


def vector(path, t, arr, names):
    """Log one series per column of `arr` under `path`."""
    arr = np.asarray(arr)
    for i, name in enumerate(names):
        series(f"{path}/{name}", t, arr[:, i], label=name)


def quat_xyzw(q_wxyz):
    """Unitree quaternions are (w, x, y, z); Rerun wants (x, y, z, w)."""
    q = np.asarray(q_wxyz, dtype=np.float32)
    return np.column_stack([q[:, 1], q[:, 2], q[:, 3], q[:, 0]])


# --- telemetry groups ---------------------------------------------------------
# Each loader takes (session, decimate) and logs one part of the session.

def load_pose(s, dec):
    """Robot path and attitude in 3D, from /sportmodestate."""
    t, m = s.decode("/sportmodestate", every=max(1, 3 * dec))
    if m is None:
        return False
    pos = m["position"].astype(np.float32)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/path", rr.LineStrips3D([pos], colors=[[120, 160, 255]], radii=[0.01]),
           static=True)
    rr.send_columns("world/robot", [_tc(t)],
                    rr.Transform3D.columns(translation=pos,
                                           quaternion=quat_xyzw(m["imu_state"]["quaternion"])))
    rr.log("world/robot", rr.Boxes3D(half_sizes=[[0.35, 0.15, 0.09]],
                                     colors=[[255, 170, 60]]), static=True)
    say(f"   pose: {len(t)} samples, path {_path_len(pos):.1f} m")
    return True


def _path_len(pos):
    return float(np.sum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)))


def load_body(s, dec):
    """Velocity, attitude, body height and foot contact, from /sportmodestate."""
    t, m = s.decode("/sportmodestate", every=max(1, 3 * dec))
    if m is None:
        return False
    vector("body/velocity", t, m["velocity"], ["x", "y", "z"])
    series("body/speed", t, np.linalg.norm(m["velocity"][:, :2], axis=1), "horizontal")
    series("body/yaw_speed", t, m["yaw_speed"], "yaw_speed")
    series("body/height", t, m["body_height"], "body_height")
    vector("body/rpy", t, np.degrees(m["imu_state"]["rpy"]), ["roll", "pitch", "yaw"])
    vector("body/foot_force", t, m["foot_force"], LEGS)
    series("body/error_code", t, m["error_code"], "error_code")
    vector("body/range_obstacle", t, m["range_obstacle"], ["0", "1", "2", "3"])
    say(f"   body: {len(t)} samples")
    return True


def load_imu(s, dec):
    """Body IMU at 20 Hz, from /lf/lowstate."""
    t, m = s.decode("/lf/lowstate", every=dec)
    if m is None:
        return False
    imu = m["imu_state"]
    vector("imu/accelerometer", t, imu["accelerometer"], ["x", "y", "z"])
    vector("imu/gyroscope", t, np.degrees(imu["gyroscope"]), ["x", "y", "z"])
    series("imu/accel_magnitude", t, np.linalg.norm(imu["accelerometer"], axis=1), "|a|")
    vector("imu/rpy", t, np.degrees(imu["rpy"]), ["roll", "pitch", "yaw"])
    say(f"   imu: {len(t)} samples")
    return True


def load_battery(s, dec):
    """Battery state of charge, voltage, current, cells and temperatures."""
    t, m = s.decode("/lf/lowstate", every=dec)
    if m is None:
        return False
    bms = m["bms_state"]
    series("battery/soc", t, bms["soc"], "soc %")
    series("battery/voltage", t, m["power_v"], "pack V")
    series("battery/current_a", t, m["power_a"], "A")
    series("battery/bms_current_ma", t, bms["current"], "mA")
    cells = bms["cell_vol"]
    live = np.flatnonzero(cells.any(axis=0))        # Go2 reports 8 of the 15 slots
    for i in live:
        series(f"battery/cells/cell{i}", t, cells[:, i], f"cell{i} mV")
    if len(live):
        spread = cells[:, live].max(axis=1) - cells[:, live].min(axis=1)
        series("battery/cell_spread_mv", t, spread, "spread mV")
    vector("battery/temp_bq", t, bms["bq_ntc"], ["0", "1"])
    vector("battery/temp_mcu", t, bms["mcu_ntc"], ["0", "1"])
    series("battery/board_ntc1", t, m["temperature_ntc1"], "ntc1 C")
    series("battery/board_ntc2", t, m["temperature_ntc2"], "ntc2 C")
    say(f"   battery: {len(t)} samples, {len(live)} live cells")
    return True


def load_motors(s, dec):
    """The 12 driven joints: angle, speed, torque and temperature."""
    t, m = s.decode("/lf/lowstate", every=dec)
    if m is None:
        return False
    ms = m["motor_state"]
    for i, name in enumerate(JOINTS):
        series(f"motors/{name}/q", t, ms["q"][:, i], "q rad")
        series(f"motors/{name}/dq", t, ms["dq"][:, i], "dq rad/s")
        series(f"motors/{name}/tau", t, ms["tau_est"][:, i], "tau Nm")
        series(f"motors/{name}/temperature", t, ms["temperature"][:, i], "C")
    say(f"   motors: {len(t)} samples x 12 joints")
    return True


def load_uwb(s, dec):
    """The UWB remote: range, bearing and attitude, plus a 3D marker.

    The angle fields are undocumented; `orientation_est` is used as the bearing and
    `pitch_est` as the elevation, matching the defaults in uwb_rviz/.
    """
    t, m = s.decode("/uwbstate", every=dec)
    if m is None:
        return False
    series("uwb/distance", t, m["distance_est"], "m")
    for f in ("orientation_est", "pitch_est", "yaw_est"):
        series(f"uwb/{f}", t, np.degrees(m[f]), "deg")
    vector("uwb/tag_attitude", t,
           np.degrees(np.column_stack([m["tag_roll"], m["tag_pitch"], m["tag_yaw"]])),
           ["roll", "pitch", "yaw"])
    vector("uwb/base_attitude", t,
           np.degrees(np.column_stack([m["base_roll"], m["base_pitch"], m["base_yaw"]])),
           ["roll", "pitch", "yaw"])
    vector("uwb/joystick", t, m["joystick"], ["x", "y"])
    series("uwb/buttons", t, m["buttons"], "buttons")

    d, bearing, elev = m["distance_est"], m["orientation_est"], m["pitch_est"]
    pos = np.column_stack([d * np.cos(elev) * np.cos(bearing),
                           d * np.cos(elev) * np.sin(bearing),
                           d * np.sin(elev)]).astype(np.float32)
    rr.send_columns("world/uwb_tag", [_tc(t)],
                    rr.Points3D.columns(positions=pos, radii=np.full(len(t), 0.08, np.float32)))
    rr.log("world/uwb_tag", rr.Points3D.from_fields(colors=[[80, 230, 140]]), static=True)
    say(f"   uwb: {len(t)} samples, range {d.min():.2f}..{d.max():.2f} m")
    return True


def load_odom(s, dec):
    """Lidar odometry: path and twist, from /utlidar/robot_odom."""
    t, m = s.decode("/utlidar/robot_odom", every=max(1, 5 * dec))
    if m is None:
        return False
    pos = m["position"].astype(np.float32)
    rr.log("world/odom_path", rr.LineStrips3D([pos], colors=[[255, 120, 190]], radii=[0.008]),
           static=True)
    vector("odom/linear", t, m["linear"], ["x", "y", "z"])
    vector("odom/angular", t, m["angular"], ["x", "y", "z"])
    say(f"   odom: {len(t)} samples, path {_path_len(pos):.1f} m")
    return True


def load_lidar_imu(s, dec):
    """The lidar's own IMU, from /utlidar/imu."""
    t, m = s.decode("/utlidar/imu", every=max(1, 10 * dec))
    if m is None:
        return False
    vector("lidar_imu/linear_acceleration", t, m["linear_acceleration"], ["x", "y", "z"])
    vector("lidar_imu/angular_velocity", t, np.degrees(m["angular_velocity"]), ["x", "y", "z"])
    say(f"   lidar_imu: {len(t)} samples")
    return True


def load_cloud(s, dec):
    """Lidar point cloud in the robot frame, from /utlidar/cloud_base."""
    every = max(1, 10 * dec)
    t, frames = s.clouds("/utlidar/cloud_base", every=every)
    if not frames:
        return False
    for ts, pts in zip(t, frames):
        rr.set_time("time", duration=float(ts))
        rr.log("world/cloud", rr.Points3D(pts, radii=0.015))
    say(f"   cloud: {len(frames)} frames (every {every}), "
        f"{min(len(f) for f in frames)}..{max(len(f) for f in frames)} points")
    return True


def load_video(s, dec, stream=720):
    """Front camera, from /frontvideostream.

    The recorded bytes do not match the local Go2FrontVideoData definition, so the
    message is unpacked by hand (see README): u64 time_frame, u32 resolution, u32
    length, then that many bytes of H.264 Annex-B. Samples are handed to Rerun as-is;
    nothing is decoded here. 720p and 360p are interleaved on the one topic.
    """
    t, blobs = s.blobs("/frontvideostream", every=dec)
    if not blobs:
        return False
    times, samples, keys = [], [], []
    for ts, b in zip(t, blobs):
        res, n = struct.unpack_from("<II", b, 12)
        if res != stream:
            continue
        frame = bytes(b[20:20 + n])
        # An SPS NAL (type 7) marks the start of a keyframe; a decoder cannot start
        # before the first one, so earlier frames are dropped.
        key = frame.find(b"\x00\x00\x00\x01\x67") >= 0
        if not keys and not key:
            continue
        times.append(ts)
        samples.append(frame)
        keys.append(key)
    if not samples:
        return False
    path = f"camera/front_{stream}p"
    rr.log(path, rr.VideoStream(codec=rr.VideoCodec.H264), static=True)
    rr.send_columns(path, [_tc(np.asarray(times))],
                    rr.VideoStream.columns(sample=samples, is_keyframe=keys))
    say(f"   video: {len(samples)} frames at {stream}p, {sum(keys)} keyframes")
    return True


def load_events(s, dec):
    """Annotations, battery alarms and robot status as a text log."""
    rows = []
    for topic, level in (("/annotations", "INFO"), ("/lf/battery_alarm", "WARN"),
                         ("/multiplestate", "DEBUG"), ("/audiohub/player/state", "DEBUG")):
        if not s.counts.get(topic):
            continue
        t, texts = s.strings(topic, every=1 if topic == "/annotations" else 20)
        rows += [(ts, f"{topic} {x}", level) for ts, x in zip(t, texts) if x]
    marks = os.path.join(s.path, "meta", "annotations.csv")
    if os.path.exists(marks):
        with open(marks) as f:
            for line in f.read().splitlines()[1:]:
                stamp, _, text = line.partition(",")
                try:
                    rows.append(((int(float(stamp) * 1e9) - s.start_ns) * 1e-9,
                                 f"mark: {text}", "INFO"))
                except ValueError:
                    pass
    if not rows:
        return False
    rows.sort(key=lambda r: r[0])
    t = np.array([r[0] for r in rows])
    rr.send_columns("events", [_tc(t)],
                    rr.TextLog.columns(text=[r[1] for r in rows], level=[r[2] for r in rows]))
    say(f"   events: {len(rows)} entries")
    return True


GROUPS = {
    "pose":      ("Robot path and attitude in 3D", ["/sportmodestate"], load_pose),
    "body":      ("Velocity, attitude, height, foot force", ["/sportmodestate"], load_body),
    "imu":       ("Body IMU (20 Hz)", ["/lf/lowstate"], load_imu),
    "battery":   ("Battery: SOC, voltage, cells, temps", ["/lf/lowstate"], load_battery),
    "motors":    ("12 joints: angle, speed, torque, temp", ["/lf/lowstate"], load_motors),
    "uwb":       ("UWB remote: range, bearing, attitude", ["/uwbstate"], load_uwb),
    "odom":      ("Lidar odometry path and twist", ["/utlidar/robot_odom"], load_odom),
    "lidar_imu": ("Lidar IMU", ["/utlidar/imu"], load_lidar_imu),
    "cloud":     ("Lidar point cloud (robot frame)", ["/utlidar/cloud_base"], load_cloud),
    "video":     ("Front camera, H.264 720p", ["/frontvideostream"], load_video),
    "events":    ("Annotations, alarms, status text", [], load_events),
}

DEFAULT = ["pose", "body", "imu", "battery", "uwb", "events"]


# --- blueprint ----------------------------------------------------------------

def blueprint(chosen):
    """A starting layout: 3D on the left, the chosen time series as tabs."""
    spatial = [g for g in ("pose", "odom", "cloud", "uwb") if g in chosen]
    tabs = []
    for g in chosen:
        if g == "events":
            tabs.append(rrb.TextLogView(origin="events", name="events"))
        elif g == "video":
            tabs.append(rrb.Spatial2DView(origin="camera", name="camera"))
        elif g in ("pose", "odom", "cloud"):
            continue
        else:
            tabs.append(rrb.TimeSeriesView(origin=f"/{g}", name=g))
    right = rrb.Tabs(*tabs) if tabs else None
    left = rrb.Spatial3DView(origin="world", name="world") if spatial else None
    if left and right:
        return rrb.Blueprint(rrb.Horizontal(left, right, column_shares=[1, 1]),
                             collapse_panels=False)
    return rrb.Blueprint(left or right or rrb.TimeSeriesView(origin="/"),
                         collapse_panels=False)


# --- interactive pickers ------------------------------------------------------

def pick_session(logs):
    ss = bag.sessions(logs)
    if not ss:
        sys.exit(f"no sessions with a bag under {logs}")
    say(f"\nSessions in {logs}:\n")
    for i, s in enumerate(ss, 1):
        say(f"  {i:2d}) {s.name}")
        say(f"      profile {s.profile}, {s.duration:.0f} s, {s.total} messages, "
            f"{len(s.active())} topics with data")
    while True:
        r = input(f"\nSession [1-{len(ss)}, default 1]: ").strip() or "1"
        if r.isdigit() and 1 <= int(r) <= len(ss):
            return ss[int(r) - 1]
        say("   pick one of the numbers listed")


def pick_groups(s):
    avail, missing = [], []
    for name, (desc, topics, _) in GROUPS.items():
        (avail if all(s.counts.get(t, 0) for t in topics) else missing).append((name, desc, topics))
    say("\nTelemetry available in this session:\n")
    for i, (name, desc, topics) in enumerate(avail, 1):
        n = min((s.counts.get(t, 0) for t in topics), default=0)
        star = " *" if name in DEFAULT else "  "
        say(f"  {i:2d}){star} {name:10s} {desc}" + (f"  ({n} msgs)" if n else ""))
    for name, desc, topics in missing:
        say(f"   --  {name:10s} {desc}  [no data on {', '.join(topics)}]")
    say("\n  * = default.  Enter numbers or names, comma separated, or 'all'.")
    names = [a[0] for a in avail]
    while True:
        r = input("Telemetry [default *]: ").strip().lower()
        if not r:
            return [g for g in DEFAULT if g in names]
        if r == "all":
            return names
        out, bad = [], []
        for part in (p.strip() for p in r.split(",") if p.strip()):
            if part.isdigit() and 1 <= int(part) <= len(names):
                out.append(names[int(part) - 1])
            elif part in names:
                out.append(part)
            else:
                bad.append(part)
        if out and not bad:
            return list(dict.fromkeys(out))
        say(f"   not available: {', '.join(bad) or 'nothing chosen'}")


# --- main ---------------------------------------------------------------------

def main():
    p = argparse.ArgumentParser(description="Serve a Go2 session to a remote Rerun viewer.")
    p.add_argument("session", nargs="?", help="session directory (prompts if left out)")
    p.add_argument("--logs", default=LOGS, help=f"where sessions live (default {LOGS})")
    p.add_argument("-t", "--telemetry", help="comma separated group names, or 'all'")
    p.add_argument("-l", "--list", action="store_true", help="list sessions and groups, then exit")
    p.add_argument("--port", type=int, default=9876, help="gRPC port (default 9876)")
    p.add_argument("--ip", "--host", default=None, metavar="IP",
                   help="host IP to display for remote clients (e.g. 100.64.113.51)")
    p.add_argument("--decimate", type=int, default=1, metavar="N",
                   help="keep 1 message in N; raised per topic for the fast ones")
    p.add_argument("--memory", default="4GiB", help="server memory limit (default 4GiB)")
    p.add_argument("--save", metavar="FILE.rrd",
                   help="write an .rrd file instead of serving (open it in any viewer)")
    args = p.parse_args()

    if args.list and not args.session:
        for s in bag.sessions(args.logs):
            say(f"{s.name}  profile {s.profile}  {s.duration:.0f}s  {s.total} msgs")
        say("\ngroups: " + ", ".join(GROUPS))
        return 0

    session = bag.Session(args.session) if args.session else pick_session(args.logs)
    if not session.files:
        sys.exit(f"no .db3 files under {session.path}/go2")

    if args.telemetry:
        want = list(GROUPS) if args.telemetry.strip().lower() == "all" else \
            [g.strip() for g in args.telemetry.split(",") if g.strip()]
        unknown = [g for g in want if g not in GROUPS]
        if unknown:
            sys.exit(f"unknown telemetry: {', '.join(unknown)}\navailable: {', '.join(GROUPS)}")
    else:
        want = pick_groups(session)
    if not want:
        sys.exit("nothing to show")

    say(f"\nSession {session.name}  ({session.duration:.0f} s, profile {session.profile})")
    say(f"Telemetry: {', '.join(want)}")

    # A plain app id, so the viewer does not have to migrate the name.
    rr.init(re.sub(r"[^A-Za-z0-9_-]+", "-", f"go2-{session.name}"), spawn=False)
    bp = blueprint(want)

    # Serving and saving are alternatives, not both: set_sinks replaces the sink a
    # running server holds, which severs it.
    if args.save:
        rr.set_sinks(rr.FileSink(args.save), default_blueprint=bp)
        rr.send_blueprint(bp, make_active=True)
        say(f"\nWriting {args.save} (not serving)\n")
    else:
        rr.serve_grpc(grpc_port=args.port, server_memory_limit=args.memory,
                      default_blueprint=bp)
        rr.send_blueprint(bp, make_active=True)
        ip = lan_address(args.ip)
        say(f"\nServing on 0.0.0.0:{args.port} (all interfaces)")
        say("Connect a viewer on another machine with:")
        say(f"\n    rerun --connect rerun+http://{ip}:{args.port}/proxy\n")
        say("(loading now; a viewer can connect at any time, including later)\n")

    t0 = time.time()
    loaded, empty = [], []
    for g in want:
        say(f"-- {g}")
        try:
            (loaded if GROUPS[g][2](session, args.decimate) else empty).append(g)
        except Exception as e:      # one bad topic should not sink the session
            say(f"   FAILED: {type(e).__name__}: {e}")
            empty.append(g)
    say(f"\nLoaded {', '.join(loaded) or 'nothing'} in {time.time() - t0:.1f} s")
    if empty:
        say(f"No data for: {', '.join(empty)}")

    if args.save:
        rec = rr.get_global_data_recording()
        if rec is not None:
            rec.flush(timeout_sec=120)
        say(f"\nWrote {args.save} ({os.path.getsize(args.save) / 1e6:.1f} MB)")
        say(f"View it with:  rerun {args.save}")
        return 0

    say("\nReady. Ctrl+C to stop the server.")
    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        say("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
