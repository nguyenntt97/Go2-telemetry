"""Tests for the session exporter.

Everything runs against bags built in `make_bag`, so no robot, no ROS and no
recorded session are needed.
"""

from __future__ import annotations

import contextlib
import csv
import io
import json
import os
import struct
import sys
import tempfile
import unittest

TESTS_DIR = os.path.dirname(os.path.abspath(__file__))
RECORD_DIR = os.path.dirname(TESTS_DIR)
MSGS_DIR = os.path.join(TESTS_DIR, "msgs")
for path in (RECORD_DIR, TESTS_DIR):
    if path not in sys.path:
        sys.path.insert(0, path)

import export as export_cli  # noqa: E402
import make_bag  # noqa: E402
from go2export import clouds as clouds_mod  # noqa: E402
from go2export import video as video_mod  # noqa: E402
from go2export.cdr import CdrReader, CdrWriter  # noqa: E402
from go2export.decode import decode_message, encode_message  # noqa: E402
from go2export.flatten import Flattener  # noqa: E402
from go2export.msgdef import MsgRegistry, parse_definition  # noqa: E402
from go2export.writers import (  # noqa: E402
    ColumnTable,
    pyarrow_available,
    topic_filename,
)


def read_json(path: str):
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def registry() -> MsgRegistry:
    return MsgRegistry([MSGS_DIR])


def read_csv(path: str) -> list:
    with open(path, newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


class CdrTests(unittest.TestCase):
    def test_header_and_alignment(self):
        writer = CdrWriter()
        writer.primitive("uint8", 7)
        writer.primitive("uint32", 1)  # must be padded to a 4 byte boundary
        blob = writer.getvalue()
        self.assertEqual(blob[:4], b"\x00\x01\x00\x00")
        self.assertEqual(blob, b"\x00\x01\x00\x00" + b"\x07\x00\x00\x00" + b"\x01\x00\x00\x00")

        reader = CdrReader(blob)
        self.assertEqual(reader.primitive("uint8"), 7)
        self.assertEqual(reader.primitive("uint32"), 1)
        self.assertEqual(reader.remaining, 0)

    def test_uint64_alignment(self):
        writer = CdrWriter()
        writer.primitive("uint16", 1)
        writer.primitive("uint64", 2)
        blob = writer.getvalue()
        # 2 bytes of payload, then 6 of padding, then the 8 byte value.
        self.assertEqual(len(blob), 4 + 2 + 6 + 8)
        reader = CdrReader(blob)
        self.assertEqual(reader.primitive("uint16"), 1)
        self.assertEqual(reader.primitive("uint64"), 2)

    def test_string_carries_its_terminator(self):
        writer = CdrWriter()
        writer.string("base_link")
        blob = writer.getvalue()
        self.assertEqual(struct.unpack_from("<I", blob, 4)[0], len("base_link") + 1)
        self.assertEqual(CdrReader(blob).string(), "base_link")

    def test_short_blob_is_rejected(self):
        with self.assertRaises(Exception):
            CdrReader(b"\x00\x01")


class MsgDefTests(unittest.TestCase):
    def test_parses_arrays_constants_and_comments(self):
        fields = parse_definition(
            """
            # a comment
            uint8 MODE_IDLE=0
            float32[3] position
            float32[] trail
            float32[<=4] bounded
            string<=32 label
            IMUState imu_state
            """,
            package="unitree_go",
        )
        by_name = {f.name: f for f in fields}
        self.assertNotIn("MODE_IDLE", by_name)
        self.assertEqual(by_name["position"].array, ("fixed", 3))
        self.assertEqual(by_name["trail"].array, ("seq", None))
        self.assertEqual(by_name["bounded"].array, ("seq", 4))
        self.assertEqual(by_name["label"].base, "string")
        self.assertEqual(by_name["imu_state"].base, "unitree_go/IMUState")

    def test_resolves_nested_and_builtin_types(self):
        reg = registry()
        names = [f.name for f in reg.fields("unitree_go/msg/SportModeState")]
        self.assertIn("imu_state", names)
        # std_msgs/Header is embedded, not on the search path.
        header = [f.name for f in reg.fields("std_msgs/Header")]
        self.assertEqual(header, ["stamp", "frame_id"])

    def test_missing_type_is_reported(self):
        with self.assertRaises(Exception):
            registry().fields("unitree_go/msg/NotAThing")


class RoundTripTests(unittest.TestCase):
    def round_trip(self, data: dict, type_name: str) -> dict:
        reg = registry()
        return decode_message(encode_message(data, type_name, reg), type_name, reg)

    def test_sport_mode_state(self):
        original = make_bag.sport_mode_state(3)
        decoded = self.round_trip(original, "unitree_go/msg/SportModeState")
        self.assertEqual(decoded["stamp"]["sec"], original["stamp"]["sec"])
        self.assertEqual(decoded["gait_type"], 1)
        self.assertEqual(decoded["foot_force"], [10, 11, 12, 13])
        self.assertAlmostEqual(decoded["position"][0], 0.3, places=5)
        self.assertAlmostEqual(decoded["yaw_speed"], 0.03, places=5)
        self.assertEqual(len(decoded["foot_position_body"]), 12)

    def test_low_state_including_nested_arrays(self):
        decoded = self.round_trip(make_bag.low_state(1), "unitree_go/msg/LowState")
        self.assertEqual(len(decoded["motor_state"]), 20)
        self.assertAlmostEqual(decoded["motor_state"][4]["q"], 0.4, places=5)
        self.assertEqual(decoded["motor_state"][4]["temperature"], 34)
        self.assertEqual(decoded["bms_state"]["soc"], 88)
        self.assertEqual(len(decoded["bms_state"]["cell_vol"]), 15)
        self.assertEqual(decoded["bms_state"]["cell_vol"][0], 4100)
        # One byte wide arrays come back as bytes.
        self.assertEqual(decoded["head"], b"\xfe\xef")
        self.assertEqual(len(decoded["wireless_remote"]), 40)
        self.assertAlmostEqual(decoded["power_v"], 29.5, places=4)

    def test_uwb_state(self):
        decoded = self.round_trip(make_bag.uwb_state(2), "unitree_go/msg/UwbState")
        self.assertAlmostEqual(decoded["distance_est"], 2.0, places=5)
        self.assertAlmostEqual(decoded["yaw_est"], -0.2, places=5)
        self.assertEqual(decoded["enabled_from_app"], 1)

    def test_point_cloud_with_unbounded_sequences(self):
        decoded = self.round_trip(make_bag.point_cloud(4), "sensor_msgs/msg/PointCloud2")
        self.assertEqual(decoded["header"]["frame_id"], "utlidar")
        self.assertEqual([f["name"] for f in decoded["fields"]], ["x", "y", "z", "intensity"])
        self.assertEqual(len(decoded["data"]), 64)

    def test_big_endian_payload(self):
        reg = registry()
        writer = CdrWriter(little_endian=False)
        from go2export.decode import encode_fields

        encode_fields(writer, reg.fields("unitree_go/TimeSpec"), {"sec": 7, "nanosec": 9}, reg)
        decoded = decode_message(writer.getvalue(), "unitree_go/TimeSpec", reg)
        self.assertEqual(decoded, {"sec": 7, "nanosec": 9})


class FlattenTests(unittest.TestCase):
    def test_motor_fields_are_dropped_by_default(self):
        reg = registry()
        decoded = decode_message(
            encode_message(make_bag.low_state(0), "unitree_go/msg/LowState", reg),
            "unitree_go/msg/LowState",
            reg,
        )
        row = Flattener().flatten(decoded)
        self.assertFalse([k for k in row if k.startswith("motor_state")])
        self.assertIn("imu_state.gyroscope.0", row)
        self.assertIn("bms_state.soc", row)
        # 40 bytes of remote state is a blob, so only its length is kept.
        self.assertEqual(row["wireless_remote.len"], 40)

    def test_motor_fields_can_be_kept(self):
        reg = registry()
        decoded = decode_message(
            encode_message(make_bag.low_state(0), "unitree_go/msg/LowState", reg),
            "unitree_go/msg/LowState",
            reg,
        )
        row = Flattener(skip=[]).flatten(decoded)
        self.assertAlmostEqual(row["motor_state.3.q"], 0.3, places=5)

    def test_long_arrays_become_a_length(self):
        row = Flattener(max_array=4).flatten({"cell_vol": list(range(15))})
        self.assertEqual(row, {"cell_vol.len": 15})

    def test_json_expansion_is_opt_in(self):
        message = {"data": '{"soc": 88, "cell": {"v": 4100}}'}
        self.assertEqual(Flattener().flatten(message), message)
        expanded = Flattener(expand_json=True).flatten(message)
        self.assertEqual(expanded["data.soc"], 88)
        self.assertEqual(expanded["data.cell.v"], 4100)


class ColumnTableTests(unittest.TestCase):
    def test_late_columns_are_backfilled(self):
        table = ColumnTable()
        table.add({"a": 1})
        table.add({"a": 2, "b": 3})
        table.add({"b": 4})
        self.assertEqual(len(table), 3)
        self.assertEqual(table.column("a"), [1, 2, None])
        self.assertEqual(table.column("b"), [None, 3, 4])

    def test_topic_filename(self):
        self.assertEqual(topic_filename("/lf/lowstate"), "lf__lowstate")
        self.assertEqual(topic_filename("/api/sport/request"), "api__sport__request")


class VideoTests(unittest.TestCase):
    def test_frame_layout(self):
        payload = make_bag.nal(7) + make_bag.nal(5)
        frame = video_mod.parse_frame(make_bag.video_message(123, 720, payload))
        self.assertEqual(frame["time_frame"], 123)
        self.assertEqual(frame["resolution"], 720)
        self.assertEqual(frame["length"], len(payload))
        self.assertEqual(frame["payload"], payload)
        self.assertEqual(frame["trailing"], 0)

    def test_nal_types_and_keyframes(self):
        self.assertTrue(video_mod.is_keyframe(make_bag.nal(7) + make_bag.nal(5)))
        self.assertTrue(video_mod.is_keyframe(make_bag.nal(5)))
        self.assertFalse(video_mod.is_keyframe(make_bag.nal(1)))
        types = [t for t, _, _ in video_mod.nal_units(make_bag.nal(7) + make_bag.nal(1))]
        self.assertEqual(types, [7, 1])

    def test_truncated_frame_is_rejected(self):
        blob = make_bag.video_message(1, 720, make_bag.nal(5))
        with self.assertRaises(video_mod.VideoError):
            video_mod.parse_frame(blob[:-4])

    def test_frames_before_the_first_keyframe_are_dropped(self):
        with tempfile.TemporaryDirectory() as tmp:
            writer = video_mod.StreamWriter(os.path.join(tmp, "s.h264"))
            self.assertIsNone(writer.add(1_000_000_000, make_bag.nal(1)))
            self.assertIsNone(writer.add(1_100_000_000, make_bag.nal(1)))
            self.assertIsNotNone(writer.add(1_200_000_000, make_bag.nal(7) + make_bag.nal(5)))
            self.assertIsNotNone(writer.add(1_300_000_000, make_bag.nal(1)))
            writer.close()
            self.assertEqual(writer.skipped_before_keyframe, 2)
            self.assertEqual(writer.frames, 2)
            self.assertEqual(writer.keyframes, 1)
            self.assertAlmostEqual(writer.fps, 10.0, places=3)
            self.assertEqual(os.path.getsize(writer.path), writer.bytes)


@unittest.skipUnless(clouds_mod.numpy_available(), "numpy is not installed")
class CloudTests(unittest.TestCase):
    def test_fields_become_arrays(self):
        reg = registry()
        decoded = decode_message(
            encode_message(make_bag.point_cloud(4), "sensor_msgs/msg/PointCloud2", reg),
            "sensor_msgs/msg/PointCloud2",
            reg,
        )
        arrays = clouds_mod.cloud_to_arrays(decoded)
        self.assertEqual(sorted(arrays), ["intensity", "x", "y", "z"])
        self.assertEqual(len(arrays["x"]), 4)
        self.assertAlmostEqual(float(arrays["x"][2]), 2.0, places=5)
        self.assertAlmostEqual(float(arrays["y"][2]), 2.5, places=5)
        self.assertAlmostEqual(float(arrays["intensity"][3]), 30.0, places=5)


class EndToEndTests(unittest.TestCase):
    """A session split across two files, as `--max-bag-duration 300` makes."""

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.session = os.path.join(self._tmp.name, "logs", "walk_test_20261023_101500")
        reg = registry()
        base = 1_700_000_000_000_000_000

        for part in range(2):
            builder = make_bag.BagBuilder(
                os.path.join(self.session, "go2", f"walk_test_{part}.db3")
            )
            # Every topic is declared in both files, as the recorder does,
            # but only some carry data in each.
            for index in range(5):
                stamp = base + (part * 5 + index) * 100_000_000
                builder.add(
                    "/sportmodestate", "unitree_go/msg/SportModeState",
                    stamp, make_bag.sport_mode_state(part * 5 + index), reg,
                )
                builder.add(
                    "/lf/lowstate", "unitree_go/msg/LowState",
                    stamp, make_bag.low_state(index), reg,
                )
            for index in range(3):
                stamp = base + (part * 3 + index) * 200_000_000
                builder.add(
                    "/uwbstate", "unitree_go/msg/UwbState",
                    stamp, make_bag.uwb_state(index), reg,
                )
            builder.add(
                "/annotations", "std_msgs/msg/String",
                base + part * 1_000_000_000, {"data": f"mark {part}"}, reg,
            )
            builder.add(
                "/lf/battery_alarm", "std_msgs/msg/String",
                base + part * 1_000_000_000,
                {"data": json.dumps({"soc": 88, "cell_vol_1": 4100})}, reg,
            )
            builder.add(
                "/utlidar/cloud_base", "sensor_msgs/msg/PointCloud2",
                base + part * 500_000_000, make_bag.point_cloud(4), reg,
            )
            # 720p and 360p interleaved on the one topic, each starting with
            # an undecodable frame.
            for index in range(4):
                for resolution in (720, 360):
                    payload = (
                        make_bag.nal(1)
                        if index == 0
                        else make_bag.nal(7) + make_bag.nal(5)
                        if index == 1
                        else make_bag.nal(1)
                    )
                    builder.add_raw(
                        "/frontvideostream",
                        "unitree_go/msg/Go2FrontVideoData",
                        base + (part * 4 + index) * 70_000_000,
                        make_bag.video_message(index, resolution, payload),
                    )
            # A topic the recorder subscribed to but which never published.
            builder.topic("/lowstate", "unitree_go/msg/LowState")
            builder.close()

        self.out = os.path.join(self.session, "export")

    def run_cli(self, extra=None) -> str:
        argv = [self.session, "--msg-path", MSGS_DIR, "--format", "csv"]
        argv += extra or []
        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = export_cli.main(argv)
        self.assertEqual(code, 0, buffer.getvalue())
        return buffer.getvalue()

    def test_list_reports_only_topics_with_data(self):
        output = self.run_cli(["--list"])
        self.assertIn("/sportmodestate", output)
        self.assertIn("/uwbstate", output)
        # Subscribed but silent, so it is not listed. Spaced to avoid
        # matching inside "/lf/lowstate".
        self.assertNotIn("  /lowstate  ", output)
        self.assertIn("8 topics, 7 with data", output)

    def test_tables_cover_both_split_files(self):
        self.run_cli()
        rows = read_csv(os.path.join(self.out, "sportmodestate.csv"))
        self.assertEqual(len(rows), 10)
        self.assertEqual(rows[0]["gait_type"], "1")
        self.assertAlmostEqual(float(rows[0]["t_rel"]), 0.0, places=6)
        self.assertAlmostEqual(float(rows[9]["t_rel"]), 0.9, places=6)
        self.assertAlmostEqual(float(rows[4]["position.0"]), 0.4, places=5)
        self.assertEqual(rows[0]["t_iso"][:4], "2023")
        # Every row keeps the same columns, even across files.
        self.assertEqual(len({tuple(r) for r in rows}), 1)

    def test_low_state_table_drops_motors_but_keeps_imu_and_battery(self):
        self.run_cli()
        rows = read_csv(os.path.join(self.out, "lf__lowstate.csv"))
        self.assertEqual(len(rows), 10)
        self.assertFalse([k for k in rows[0] if k.startswith("motor_state")])
        self.assertIn("imu_state.accelerometer.2", rows[0])
        self.assertEqual(rows[0]["bms_state.soc"], "88")
        self.assertEqual(rows[0]["foot_force.0"], "20")

    def test_uwb_and_annotations(self):
        self.run_cli()
        uwb = read_csv(os.path.join(self.out, "uwbstate.csv"))
        self.assertEqual(len(uwb), 6)
        self.assertAlmostEqual(float(uwb[1]["distance_est"]), 2.0, places=5)
        marks = read_csv(os.path.join(self.out, "annotations.csv"))
        self.assertEqual([m["data"] for m in marks], ["mark 0", "mark 1"])

    def test_json_fields_stay_whole_unless_asked(self):
        self.run_cli()
        rows = read_csv(os.path.join(self.out, "lf__battery_alarm.csv"))
        self.assertIn("soc", rows[0]["data"])
        self.assertNotIn("data.soc", rows[0])

        self.run_cli(["--expand-json"])
        rows = read_csv(os.path.join(self.out, "lf__battery_alarm.csv"))
        self.assertEqual(rows[0]["data.soc"], "88")
        self.assertEqual(rows[0]["data.cell_vol_1"], "4100")

    def test_video_is_split_by_resolution(self):
        self.run_cli()
        for resolution in (720, 360):
            stream = os.path.join(self.out, "video", f"frontvideostream_{resolution}p.h264")
            self.assertTrue(os.path.exists(stream), stream)
            self.assertGreater(os.path.getsize(stream), 0)
            index = read_csv(
                os.path.join(self.out, "video", f"frontvideostream_{resolution}p_index.csv")
            )
            # 8 messages per resolution across the two files. A stream spans
            # the split, so only the very first frame is before a keyframe.
            self.assertEqual(len(index), 7)
            self.assertEqual(index[0]["keyframe"], "1")
            self.assertEqual(index[0]["byte_offset"], "0")

        report = read_json(os.path.join(self.out, "export_report.json"))
        streams = {s["resolution"]: s for s in report["video"]["/frontvideostream"]["streams"]}
        self.assertEqual(streams[720]["dropped_before_keyframe"], 1)
        self.assertEqual(streams[720]["frames"], 7)
        self.assertEqual(streams[720]["keyframes"], 2)
        self.assertEqual(report["video"]["/frontvideostream"]["malformed"], 0)
        # The video topic gets no table, since its message type cannot decode.
        self.assertNotIn("/frontvideostream", report["topics"])

    def test_cloud_topic_is_a_table_unless_clouds_asked(self):
        self.run_cli()
        rows = read_csv(os.path.join(self.out, "utlidar__cloud_base.csv"))
        self.assertEqual(rows[0]["width"], "4")
        self.assertEqual(rows[0]["data.len"], "64")
        self.assertFalse(os.path.exists(os.path.join(self.out, "clouds")))

    @unittest.skipUnless(clouds_mod.numpy_available(), "numpy is not installed")
    def test_clouds_are_written_per_frame(self):
        self.run_cli(["--clouds"])
        cloud_dir = os.path.join(self.out, "clouds", "utlidar__cloud_base")
        frames = sorted(os.listdir(cloud_dir))
        self.assertEqual(len(frames), 2)
        self.assertTrue(frames[0].endswith(".npz"))

    def test_topic_filters(self):
        self.run_cli(["--topics", "^/uwbstate$"])
        self.assertTrue(os.path.exists(os.path.join(self.out, "uwbstate.csv")))
        self.assertFalse(os.path.exists(os.path.join(self.out, "sportmodestate.csv")))

    def test_report_records_the_decoder_used(self):
        self.run_cli()
        report = read_json(os.path.join(self.out, "export_report.json"))
        entry = report["topics"]["/sportmodestate"]
        self.assertEqual(entry["rows"], 10)
        self.assertEqual(entry["decode_failures"], 0)
        self.assertEqual(entry["decoder"], "msgdef")
        self.assertEqual(entry["type"], "unitree_go/msg/SportModeState")

    @unittest.skipUnless(pyarrow_available(), "pyarrow is not installed")
    def test_parquet_output(self):
        import pyarrow.parquet

        buffer = io.StringIO()
        with contextlib.redirect_stdout(buffer):
            code = export_cli.main(
                [self.session, "--msg-path", MSGS_DIR, "--format", "parquet"]
            )
        self.assertEqual(code, 0, buffer.getvalue())

        table = pyarrow.parquet.read_table(
            os.path.join(self.out, "sportmodestate.parquet")
        )
        self.assertEqual(table.num_rows, 10)
        self.assertIn("t_ns", table.column_names)
        self.assertIn("position.0", table.column_names)
        self.assertEqual(table.column("gait_type").to_pylist()[0], 1)
        self.assertAlmostEqual(table.column("position.0").to_pylist()[4], 0.4, places=5)

    @unittest.skipUnless(pyarrow_available(), "pyarrow is not installed")
    def test_parquet_takes_columns_that_are_sometimes_absent(self):
        """A column that appears late is null-padded, not a type error."""
        import pyarrow.parquet

        table = ColumnTable()
        table.add({"a": 1})
        table.add({"a": 2, "b": "text"})
        with tempfile.TemporaryDirectory() as tmp:
            from go2export.writers import write_parquet

            path = write_parquet(table, os.path.join(tmp, "t.parquet"))
            written = pyarrow.parquet.read_table(path)
        self.assertEqual(written.column("a").to_pylist(), [1, 2])
        self.assertEqual(written.column("b").to_pylist(), [None, "text"])

    def test_missing_session_is_an_error(self):
        buffer = io.StringIO()
        with contextlib.redirect_stderr(buffer):
            code = export_cli.main([os.path.join(self._tmp.name, "nope")])
        self.assertEqual(code, 2)
        self.assertIn("not a directory", buffer.getvalue())


if __name__ == "__main__":
    unittest.main(verbosity=2)
