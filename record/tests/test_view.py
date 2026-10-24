"""Tests for the Rerun viewer.

The handlers are checked by standing in for the `rerun` module and recording
what they log, so the assertions are about entity paths and geometry rather
than about Rerun's own behaviour. The geometry that matters -- the UWB mapping
and the quaternion order -- is tested through pure functions.
"""

from __future__ import annotations

import contextlib
import io
import math
import os
import sys
import tempfile
import unittest
from unittest import mock

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
RECORD_DIR = os.path.dirname(TESTS_DIR)
MSGS_DIR = os.path.join(TESTS_DIR, "msgs")
for path in (RECORD_DIR, TESTS_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

import rerun as rr  # noqa: E402

import make_bag  # noqa: E402
import view as view_cli  # noqa: E402
from go2export.bagreader import BagReader, find_bag_dir  # noqa: E402
from go2export.clouds import numpy_available  # noqa: E402
from go2export.decode import Decoder  # noqa: E402
from go2export.msgdef import MsgRegistry  # noqa: E402
from go2view import layout, sessions, streams  # noqa: E402


class SpyRerun:
    """Stands in for `rerun`, recording log and time calls.

    Anything not intercepted (archetypes, VideoCodec) comes from the real
    module, so the handlers build genuine archetypes.
    """

    def __init__(self) -> None:
        self.logs: list = []
        self.times: list = []

    def __getattr__(self, name):
        return getattr(rr, name)

    def log(self, path, archetype=None, **kwargs):
        self.logs.append((path, archetype, kwargs))

    def set_time(self, timeline, **kwargs):
        self.times.append((timeline, kwargs))

    def send_blueprint(self, *args, **kwargs):
        pass

    @property
    def paths(self) -> list:
        return [path for path, _, _ in self.logs]

    def logged(self, prefix: str) -> list:
        return [path for path in self.paths if path.startswith(prefix)]


@contextlib.contextmanager
def spying():
    spy = SpyRerun()
    with mock.patch.object(streams, "rr", spy):
        yield spy


def registry() -> MsgRegistry:
    return MsgRegistry([MSGS_DIR])


def context(**kwargs) -> streams.Context:
    return streams.Context(options=streams.Options(**kwargs), start_ns=0)


class GeometryTests(unittest.TestCase):
    def test_quaternion_is_reordered_for_rerun(self):
        # Unitree sends w,x,y,z.
        self.assertEqual(
            streams.quaternion_xyzw([1.0, 0.0, 0.0, 0.0], "wxyz"), [0.0, 0.0, 0.0, 1.0]
        )
        self.assertEqual(
            streams.quaternion_xyzw([0.0, 0.0, 0.0, 1.0], "xyzw"), [0.0, 0.0, 0.0, 1.0]
        )

    def test_degenerate_quaternion_is_dropped(self):
        self.assertIsNone(streams.quaternion_xyzw([0.0, 0.0, 0.0, 0.0], "wxyz"))
        self.assertIsNone(streams.quaternion_xyzw(None))
        self.assertIsNone(streams.quaternion_xyzw([1.0, 0.0]))

    def test_uwb_point_straight_ahead(self):
        options = streams.Options(elevation_field="none")
        point = streams.uwb_point({"distance_est": 2.0, "orientation_est": 0.0}, options)
        self.assertAlmostEqual(point[0], 2.0, places=6)
        self.assertAlmostEqual(point[1], 0.0, places=6)
        self.assertAlmostEqual(point[2], 0.0, places=6)

    def test_uwb_point_to_the_left(self):
        options = streams.Options(elevation_field="none")
        point = streams.uwb_point(
            {"distance_est": 3.0, "orientation_est": math.pi / 2}, options
        )
        self.assertAlmostEqual(point[0], 0.0, places=6)
        self.assertAlmostEqual(point[1], 3.0, places=6)

    def test_uwb_bearing_sign_and_offset(self):
        message = {"distance_est": 1.0, "yaw_est": math.pi / 2}
        options = streams.Options(
            bearing_field="yaw_est", bearing_sign=-1.0, elevation_field="none"
        )
        self.assertAlmostEqual(streams.uwb_bearing(message, options), -math.pi / 2, places=6)
        point = streams.uwb_point(message, options)
        self.assertAlmostEqual(point[1], -1.0, places=6)

        options.bearing_offset = math.pi / 2
        self.assertAlmostEqual(streams.uwb_bearing(message, options), 0.0, places=6)

    def test_uwb_elevation_lifts_the_point(self):
        options = streams.Options(elevation_field="pitch_est")
        point = streams.uwb_point(
            {"distance_est": 2.0, "orientation_est": 0.0, "pitch_est": math.pi / 2}, options
        )
        self.assertAlmostEqual(point[2], 2.0, places=6)
        self.assertAlmostEqual(point[0], 0.0, places=6)

    def test_non_finite_values_do_not_propagate(self):
        options = streams.Options(elevation_field="none")
        point = streams.uwb_point({"distance_est": float("nan"), "orientation_est": 0.0}, options)
        self.assertTrue(all(math.isfinite(v) for v in point))


class HandlerTests(unittest.TestCase):
    def test_pose_logs_transform_plots_and_trail(self):
        with spying() as spy:
            ctx = context(trail_every=2)
            for index in range(4):
                streams.handle_pose(ctx, "/sportmodestate", index, make_bag.sport_mode_state(index))
        self.assertIn("/world/robot", spy.paths)
        self.assertIn("/world/robot/body", spy.paths)
        self.assertIn("/plots/velocity/x", spy.paths)
        self.assertIn("/plots/yaw_speed", spy.paths)
        self.assertIn("/world/trail", spy.paths)
        # The body box is static, logged once.
        self.assertEqual(spy.paths.count("/world/robot/body"), 1)
        # Trail every 2 messages out of 4.
        self.assertEqual(spy.paths.count("/world/trail"), 2)

    def test_trail_is_capped(self):
        with spying():
            ctx = context(trail_length=5, trail_every=0)
            for index in range(20):
                streams.handle_pose(ctx, "/sportmodestate", index, make_bag.sport_mode_state(index))
        self.assertEqual(len(ctx.trail), 5)

    def test_imu_handles_both_shapes(self):
        with spying() as spy:
            ctx = context()
            streams.handle_imu(ctx, "/lf/lowstate", 0, make_bag.low_state(0))
            streams.handle_imu(ctx, "/utlidar/imu", 0, make_bag.imu_message(0))
        self.assertIn("/plots/imu/lf_lowstate/gyro/x", spy.paths)
        self.assertIn("/plots/imu/lf_lowstate/rpy/yaw", spy.paths)
        # sensor_msgs/Imu has no imu_state, so the nested shape is used.
        self.assertIn("/plots/imu/utlidar_imu/gyro/z", spy.paths)
        self.assertIn("/plots/imu/utlidar_imu/accel/z", spy.paths)

    def test_feet_logs_forces_and_points(self):
        with spying() as spy:
            ctx = context()
            streams.handle_feet(ctx, "/lf/lowstate", 0, make_bag.low_state(0))
            streams.handle_feet(ctx, "/sportmodestate", 0, make_bag.sport_mode_state(0))
        self.assertIn("/plots/feet/force/0", spy.paths)
        self.assertIn("/plots/feet/force/3", spy.paths)
        self.assertIn("/world/robot/feet", spy.paths)

    def test_battery_logs_cells_and_json_alarm(self):
        with spying() as spy:
            ctx = context()
            streams.handle_battery(ctx, "/lf/lowstate", 0, make_bag.low_state(0))
            streams.handle_battery(
                ctx, "/lf/battery_alarm", 0, {"data": '{"soc": 88, "cell_vol_1": 4100}'}
            )
        self.assertIn("/plots/battery/soc", spy.paths)
        self.assertIn("/plots/battery/cell/00", spy.paths)
        self.assertIn("/plots/battery/cell/14", spy.paths)
        self.assertIn("/plots/battery/power_v", spy.paths)
        self.assertIn("/plots/battery/alarm/soc", spy.paths)

    def test_uwb_logs_point_ray_and_every_angle(self):
        with spying() as spy:
            ctx = context()
            streams.handle_uwb(ctx, "/uwbstate", 0, make_bag.uwb_state(2))
        self.assertIn("/world/robot/uwb_tag", spy.paths)
        self.assertIn("/world/robot/uwb_bearing", spy.paths)
        for name in ("distance_est", "orientation_est", "yaw_est", "tag_yaw", "base_yaw"):
            self.assertIn(f"/plots/uwb/{name}", spy.paths)
        self.assertIn("/plots/uwb/derived/bearing_used", spy.paths)

    def test_odometry_pose_and_twist(self):
        with spying() as spy:
            ctx = context()
            streams.handle_odom(ctx, "/utlidar/robot_odom", 0, make_bag.odometry(3))
        self.assertIn("/world/odom/utlidar_robot_odom", spy.paths)
        self.assertIn("/plots/odom/utlidar_robot_odom/position/x", spy.paths)
        self.assertIn("/plots/odom/utlidar_robot_odom/linear/x", spy.paths)
        self.assertIn("/plots/odom/utlidar_robot_odom/angular/z", spy.paths)

    def test_events_and_controller(self):
        with spying() as spy:
            ctx = context()
            streams.handle_events(ctx, "/annotations", 0, {"data": "start circle"})
            streams.handle_controller(ctx, "/wirelesscontroller", 0, make_bag.wireless_controller(1))
        self.assertIn("/events/annotations", spy.paths)
        self.assertIn("/plots/controller/wirelesscontroller/lx", spy.paths)
        self.assertIn("/plots/controller/wirelesscontroller/keys", spy.paths)

    def test_video_splits_resolutions_and_waits_for_a_keyframe(self):
        with spying() as spy:
            ctx = context()
            # First frame of each stream is not a keyframe, so it is dropped.
            streams.handle_video(ctx, "/frontvideostream", 0, make_bag.video_message(0, 720, make_bag.nal(1)))
            streams.handle_video(ctx, "/frontvideostream", 1, make_bag.video_message(1, 360, make_bag.nal(1)))
            streams.handle_video(
                ctx, "/frontvideostream", 2,
                make_bag.video_message(2, 720, make_bag.nal(7) + make_bag.nal(5)),
            )
            streams.handle_video(ctx, "/frontvideostream", 3, make_bag.video_message(3, 720, make_bag.nal(1)))
        self.assertEqual(ctx.counters.get("video:720p:dropped"), 1)
        self.assertEqual(ctx.counters.get("video:360p:dropped"), 1)
        self.assertEqual(ctx.counters.get("video:720p"), 2)
        self.assertIsNone(ctx.counters.get("video:360p"))
        # The codec is declared once, statically, then samples follow.
        static = [kw.get("static") for path, _, kw in spy.logs if path == "/video/720p"]
        self.assertEqual(static, [True, None, None])
        self.assertEqual(sorted(ctx._video_started), ["/video/720p"])

    def test_malformed_video_is_recorded_not_raised(self):
        with spying():
            ctx = context()
            streams.handle_video(ctx, "/frontvideostream", 0, b"\x00\x01")
        self.assertIn("video", ctx.errors)

    @unittest.skipUnless(numpy_available(), "numpy is not installed")
    def test_cloud_points_and_stride(self):
        from go2export.decode import decode_message, encode_message

        reg = registry()
        message = decode_message(
            encode_message(make_bag.point_cloud(10), "sensor_msgs/msg/PointCloud2", reg),
            "sensor_msgs/msg/PointCloud2",
            reg,
        )
        with spying() as spy:
            ctx = context(cloud_stride=2)
            for _ in range(4):
                streams.handle_cloud(ctx, "/utlidar/cloud_base", 0, message)
        # Every second frame, and cloud_base hangs off the robot transform.
        self.assertEqual(ctx.counters.get("cloud:/utlidar/cloud_base"), 2)
        self.assertEqual(spy.paths.count("/world/robot/utlidar_cloud_base"), 2)

        with spying() as spy:
            ctx = context()
            streams.handle_cloud(ctx, "/utlidar/cloud", 0, message)
        self.assertIn("/world/utlidar_cloud", spy.paths)

    @unittest.skipUnless(numpy_available(), "numpy is not installed")
    def test_intensity_colors(self):
        colors = streams._intensity_colors([0.0, 5.0, 10.0])
        self.assertEqual(colors.shape, (3, 3))
        self.assertEqual(colors.dtype.name, "uint8")
        # A flat field has no range to colour by.
        self.assertIsNone(streams._intensity_colors([1.0, 1.0, 1.0]))
        self.assertIsNone(streams._intensity_colors([]))


class HardwareCsvTests(unittest.TestCase):
    def test_numeric_columns_become_plots(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "hardware.csv")
            with open(path, "w", encoding="utf-8") as handle:
                handle.write("time,cpu_total,gpu_load,temp_cpu,note\n")
                handle.write("1761000000,35.5,12,48.2,hello\n")
                handle.write("1761000001,41.0,15,49.0,hello\n")
            with spying() as spy:
                ctx = context()
                rows = streams.log_hardware_csv(ctx, path)
        self.assertEqual(rows, 2)
        self.assertIn("/plots/jetson/cpu_total", spy.paths)
        self.assertIn("/plots/jetson/temp_cpu", spy.paths)
        # Non-numeric columns are left out, as is the time column.
        self.assertNotIn("/plots/jetson/note", spy.paths)
        self.assertNotIn("/plots/jetson/time", spy.paths)
        self.assertEqual(spy.times[0][0], "receive")

    def test_missing_file_is_not_an_error(self):
        with spying():
            self.assertEqual(streams.log_hardware_csv(context(), "/nope/hardware.csv"), 0)

    def test_time_units_are_guessed(self):
        self.assertAlmostEqual(streams._parse_time("1761000000"), 1761000000.0)
        self.assertAlmostEqual(streams._parse_time("1761000000000"), 1761000000.0)
        self.assertAlmostEqual(streams._parse_time("1761000000000000000"), 1761000000.0)
        self.assertIsNone(streams._parse_time("not a time"))
        self.assertIsNone(streams._parse_time(""))


class SelectionTests(unittest.TestCase):
    def test_available_groups_follow_the_topics(self):
        topics = {"/sportmodestate": None, "/uwbstate": None, "/frontvideostream": None}
        with tempfile.TemporaryDirectory() as tmp:
            available = streams.available_groups(topics, tmp)
        keys = [group.key for group, _ in available]
        self.assertEqual(keys, ["pose", "feet", "uwb", "video"])
        self.assertNotIn("battery", keys)
        self.assertNotIn("jetson", keys)

    def test_hardware_csv_adds_the_jetson_group(self):
        with tempfile.TemporaryDirectory() as tmp:
            os.makedirs(os.path.join(tmp, "meta"))
            open(os.path.join(tmp, "meta", "hardware.csv"), "w").close()
            available = streams.available_groups({"/sportmodestate": None}, tmp)
        self.assertIn("jetson", [group.key for group, _ in available])

    def test_choose_telemetry_parses_keys_numbers_and_all(self):
        with tempfile.TemporaryDirectory() as tmp:
            available = streams.available_groups(
                {"/sportmodestate": None, "/uwbstate": None}, tmp
            )
        keys = [group.key for group, _ in available]
        self.assertEqual(view_cli.choose_telemetry(available, "all", False), keys)
        self.assertEqual(view_cli.choose_telemetry(available, "uwb,pose", False), ["uwb", "pose"])
        self.assertEqual(view_cli.choose_telemetry(available, "1", False), ["pose"])
        self.assertEqual(view_cli.choose_telemetry(available, "pose pose", False), ["pose"])
        self.assertEqual(view_cli.choose_telemetry(available, None, False), keys)

    def test_unknown_telemetry_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            available = streams.available_groups({"/sportmodestate": None}, tmp)
        with self.assertRaises(SystemExit) as caught:
            view_cli.choose_telemetry(available, "lidar", False)
        self.assertIn("lidar", str(caught.exception))

    def test_blueprint_follows_the_selection(self):
        self.assertIsNotNone(layout.build(["pose", "uwb"]))
        self.assertIsNotNone(layout.build([]))
        self.assertIsNotNone(layout.build(["pose"], video_paths=["/video/720p"], has_events=True))


class AdvertiseTests(unittest.TestCase):
    def test_loopback_is_replaced_for_remote_clients(self):
        self.assertEqual(
            view_cli.advertised("rerun+http://127.0.0.1:9876/proxy", "192.168.1.50"),
            "rerun+http://192.168.1.50:9876/proxy",
        )
        self.assertEqual(
            view_cli.advertised("rerun+http://0.0.0.0:9876/proxy", "jetson.local"),
            "rerun+http://jetson.local:9876/proxy",
        )

    def test_local_ip_is_a_string(self):
        self.assertIsInstance(view_cli.local_ip(), str)
        self.assertTrue(view_cli.local_ip())


class SessionDiscoveryTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = os.path.join(self._tmp.name, "logs")
        reg = registry()
        for name in ("walk_test_20261023_143002", "uwb_calibration_20261022_101500"):
            builder = make_bag.BagBuilder(os.path.join(self.root, name, "go2", "a.db3"))
            builder.add("/uwbstate", "unitree_go/msg/UwbState", 1, make_bag.uwb_state(0), reg)
            builder.close()
        os.makedirs(os.path.join(self.root, "walk_test_20261023_143002", "meta"))
        with open(
            os.path.join(self.root, "walk_test_20261023_143002", "meta", "session.yaml"),
            "w", encoding="utf-8",
        ) as handle:
            handle.write('name: walk_test\nprofile: standard\nnote: "hallway, 2 people"\n')
        # Not a session: no bag inside.
        os.makedirs(os.path.join(self.root, "not_a_session"))

    def test_sessions_are_found_newest_first(self):
        found = sessions.find_sessions(self.root)
        self.assertEqual(
            [s.name for s in found],
            ["walk_test_20261023_143002", "uwb_calibration_20261022_101500"],
        )
        self.assertNotIn("not_a_session", [s.name for s in found])

    def test_session_metadata_is_read(self):
        newest = sessions.find_sessions(self.root)[0]
        self.assertEqual(newest.profile, "standard")
        self.assertEqual(newest.note, "hallway, 2 people")
        self.assertGreater(newest.size_bytes, 0)
        self.assertEqual(newest.started.year, 2026)

    def test_resolve_by_name_prefix_and_path(self):
        self.assertEqual(
            sessions.resolve_session("walk_test", self.root, interactive=False).name,
            "walk_test_20261023_143002",
        )
        path = os.path.join(self.root, "uwb_calibration_20261022_101500")
        self.assertEqual(sessions.resolve_session(path, self.root, False).name, os.path.basename(path))

    def test_resolve_without_argument_takes_the_newest(self):
        self.assertEqual(
            sessions.resolve_session(None, self.root, interactive=False).name,
            "walk_test_20261023_143002",
        )

    def test_unknown_session_is_an_error(self):
        with self.assertRaises(sessions.SessionError):
            sessions.resolve_session("nope", self.root, interactive=False)
        with self.assertRaises(sessions.SessionError):
            sessions.find_sessions(os.path.join(self._tmp.name, "missing"))

    def test_human_size(self):
        self.assertEqual(sessions.human_size(512), "512 B")
        self.assertEqual(sessions.human_size(5 * 1024 * 1024 * 1024), "5.0 GB")


class StreamDispatchTests(unittest.TestCase):
    """One topic feeding several groups must be decoded once."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.session = os.path.join(self._tmp.name, "logs", "walk_test_20261023_143002")
        reg = registry()
        base = 1_761_000_000_000_000_000
        builder = make_bag.BagBuilder(os.path.join(self.session, "go2", "a.db3"))
        for index in range(4):
            builder.add(
                "/lf/lowstate", "unitree_go/msg/LowState",
                base + index * 50_000_000, make_bag.low_state(index), reg,
            )
            builder.add(
                "/sportmodestate", "unitree_go/msg/SportModeState",
                base + index * 10_000_000, make_bag.sport_mode_state(index), reg,
            )
        builder.add("/uwbstate", "unitree_go/msg/UwbState", base, make_bag.uwb_state(0), reg)
        builder.add("/annotations", "std_msgs/msg/String", base, {"data": "mark"}, reg)
        builder.add(
            "/utlidar/robot_odom", "nav_msgs/msg/Odometry", base, make_bag.odometry(1), reg
        )
        builder.add_raw(
            "/frontvideostream", "unitree_go/msg/Go2FrontVideoData", base,
            make_bag.video_message(0, 720, make_bag.nal(7) + make_bag.nal(5)),
        )
        builder.close()
        self.reader = BagReader(find_bag_dir(self.session))
        self.registry = registry()

    def test_shared_topic_is_decoded_once_per_message(self):
        decoder = Decoder(self.registry, "msgdef")
        calls = []
        real_decode = decoder.decode

        def counting(blob, type_name):
            calls.append(type_name)
            return real_decode(blob, type_name)

        decoder.decode = counting
        groups = [streams.GROUPS_BY_KEY[k] for k in ("imu", "feet", "battery")]
        ctx = context()
        with spying() as spy:
            streams.stream(self.reader, decoder, groups, ctx, self.session)

        # 4 lowstate + 4 sportmodestate (feet reads both), decoded once each.
        self.assertEqual(calls.count("unitree_go/msg/LowState"), 4)
        self.assertEqual(calls.count("unitree_go/msg/SportModeState"), 4)
        # All three groups still got their data.
        self.assertTrue(spy.logged("/plots/imu"))
        self.assertTrue(spy.logged("/plots/feet"))
        self.assertTrue(spy.logged("/plots/battery"))

    def test_time_window_limits_what_is_logged(self):
        decoder = Decoder(self.registry, "msgdef")
        ctx = streams.Context(
            options=streams.Options(start_seconds=0.0, duration_seconds=0.015),
            start_ns=self.reader.start_time(),
        )
        with spying():
            streams.stream(self.reader, decoder, [streams.GROUPS_BY_KEY["pose"]], ctx, self.session)
        # sportmodestate is every 10 ms, so only the first two are inside 15 ms.
        self.assertEqual(ctx.counters.get("pose"), 2)

    def test_both_timelines_are_set(self):
        decoder = Decoder(self.registry, "msgdef")
        ctx = streams.Context(options=streams.Options(), start_ns=self.reader.start_time())
        with spying() as spy:
            streams.stream(self.reader, decoder, [streams.GROUPS_BY_KEY["pose"]], ctx, self.session)
        timelines = {name for name, _ in spy.times}
        self.assertEqual(timelines, {"receive", "session"})

    def test_raw_group_does_not_need_a_message_definition(self):
        """Go2FrontVideoData has no usable definition, and must not be decoded."""
        decoder = Decoder(self.registry, "msgdef")
        calls = []
        decoder.decode = lambda blob, type_name: calls.append(type_name)
        ctx = context()
        with spying() as spy:
            streams.stream(self.reader, decoder, [streams.GROUPS_BY_KEY["video"]], ctx, self.session)
        self.assertEqual(calls, [])
        self.assertIn("/video/720p", spy.paths)
        self.assertEqual(ctx.errors, {})

    def test_a_failing_handler_does_not_stop_the_rest(self):
        decoder = Decoder(self.registry, "msgdef")
        broken = streams.Group(
            "broken", "always raises", r"^/sportmodestate$",
            lambda ctx, topic, t_ns, msg: (_ for _ in ()).throw(RuntimeError("boom")),
        )
        ctx = context()
        with spying() as spy:
            streams.stream(
                self.reader, decoder, [broken, streams.GROUPS_BY_KEY["pose"]], ctx, self.session
            )
        self.assertTrue(any("boom" in m for m in ctx.errors.values()))
        self.assertTrue(spy.logged("/world/robot"))


class CliTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.root = os.path.join(self._tmp.name, "logs")
        self.name = "walk_test_20261023_143002"
        reg = registry()
        base = 1_761_000_000_000_000_000
        builder = make_bag.BagBuilder(os.path.join(self.root, self.name, "go2", "a.db3"))
        for index in range(3):
            builder.add(
                "/sportmodestate", "unitree_go/msg/SportModeState",
                base + index * 10_000_000, make_bag.sport_mode_state(index), reg,
            )
        builder.add("/uwbstate", "unitree_go/msg/UwbState", base, make_bag.uwb_state(0), reg)
        builder.close()

    def run_cli(self, extra):
        argv = ["--logs-root", self.root, "--msg-path", MSGS_DIR, "--yes"] + extra
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = view_cli.main(argv)
        return code, out.getvalue(), err.getvalue()

    def test_list_telemetry(self):
        code, out, _ = self.run_cli([self.name, "--list-telemetry"])
        self.assertEqual(code, 0)
        self.assertIn("pose", out)
        self.assertIn("uwb", out)
        self.assertIn("/sportmodestate", out)
        self.assertNotIn("battery", out)

    def test_save_writes_an_rrd(self):
        target = os.path.join(self._tmp.name, "out.rrd")
        code, out, _ = self.run_cli(
            [self.name, "--serve", "save", "--rrd", target, "--telemetry", "pose,uwb"]
        )
        self.assertEqual(code, 0, out)
        self.assertTrue(os.path.exists(target))
        self.assertGreater(os.path.getsize(target), 0)
        self.assertIn("telemetry: pose, uwb", out)

    def test_newest_session_is_used_without_an_argument(self):
        target = os.path.join(self._tmp.name, "auto.rrd")
        code, out, _ = self.run_cli(["--serve", "save", "--rrd", target])
        self.assertEqual(code, 0, out)
        self.assertIn(self.name, out)

    def test_unknown_session_is_an_error(self):
        code, _, err = self.run_cli(["nope", "--serve", "save"])
        self.assertEqual(code, 2)
        self.assertIn("no session", err)

    def test_missing_logs_root_is_an_error(self):
        out, err = io.StringIO(), io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = view_cli.main(
                ["--logs-root", os.path.join(self._tmp.name, "missing"), "--yes", "--serve", "save"]
            )
        self.assertEqual(code, 2)
        self.assertIn("no logs directory", err.getvalue())

    def test_default_logs_root_points_at_the_project_drive(self):
        self.assertEqual(sessions.DEFAULT_LOGS_ROOT, "/media/nguyen/project_data/go2/logs")


if __name__ == "__main__":
    unittest.main(verbosity=2)
