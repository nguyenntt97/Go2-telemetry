#!/usr/bin/env python3
"""Flatten a recorded session into one file per topic.

    python3 record/export.py logs/<session>
    python3 record/export.py logs/<session> --format csv --clouds --video mp4

Every row is timestamped by when the Jetson received the message, which is the
clock all topics share, so the tables line up without further work.

Needs nothing but the standard library for the default run. pyarrow enables
Parquet, numpy enables `--clouds`, and OpenCV enables `--video mp4`; each is
reported as unavailable rather than failing the export.
"""

from __future__ import annotations

import argparse
import datetime
import json
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from go2export import clouds as clouds_mod
from go2export import video as video_mod
from go2export.bagreader import BagError, BagReader, find_bag_dir
from go2export.decode import DecodeError, Decoder
from go2export.flatten import DEFAULT_SKIP, Flattener
from go2export.msgdef import DEFAULT_SEARCH_PATHS, MsgRegistry
from go2export.writers import (
    ColumnTable,
    pyarrow_available,
    topic_filename,
    write_csv,
    write_parquet,
)

CLOUD_TYPES = {"sensor_msgs/msg/PointCloud2", "sensor_msgs/PointCloud2"}


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Flatten a Go2 recording session into per-topic tables.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("session", help="logs/<session>, or the bag directory itself")
    parser.add_argument("--out", help="output directory (default <session>/export)")
    parser.add_argument(
        "--format",
        choices=("auto", "parquet", "csv", "both"),
        default="auto",
        help="auto uses Parquet when pyarrow is installed, otherwise CSV",
    )
    parser.add_argument(
        "--topics", help="only topics whose name matches this regular expression"
    )
    parser.add_argument(
        "--exclude", help="skip topics whose name matches this regular expression"
    )
    parser.add_argument(
        "--msg-path",
        action="append",
        default=[],
        help=f"where to look for .msg files (default: {', '.join(DEFAULT_SEARCH_PATHS)})",
    )
    parser.add_argument(
        "--decoder",
        choices=("auto", "ros", "msgdef"),
        default="auto",
        help="auto prefers the installed ROS typesupport and falls back to .msg files",
    )
    parser.add_argument(
        "--clouds", action="store_true", help="also write point clouds as .npz per frame"
    )
    parser.add_argument(
        "--video",
        choices=("raw", "mp4", "none"),
        default="raw",
        help="raw writes .h264 as recorded; mp4 additionally re-encodes with OpenCV",
    )
    parser.add_argument(
        "--expand-json",
        action="store_true",
        help="also split JSON string fields (/api/* calls, battery cells) into columns",
    )
    parser.add_argument(
        "--max-array",
        type=int,
        default=32,
        help="arrays longer than this become a length column instead of values",
    )
    parser.add_argument(
        "--skip",
        action="append",
        default=[],
        help="drop fields whose path matches this regular expression (repeatable)",
    )
    parser.add_argument(
        "--no-default-skip",
        action="store_true",
        help=f"keep fields normally dropped ({', '.join(DEFAULT_SKIP)})",
    )
    parser.add_argument(
        "--list", action="store_true", help="list the topics and their counts, export nothing"
    )
    return parser.parse_args(argv)


def resolve_formats(choice: str) -> list:
    have_parquet = pyarrow_available()
    if choice == "auto":
        return ["parquet"] if have_parquet else ["csv"]
    if choice == "both":
        return ["parquet", "csv"] if have_parquet else ["csv"]
    if choice == "parquet" and not have_parquet:
        print("pyarrow is not installed; writing CSV instead", file=sys.stderr)
        return ["csv"]
    return [choice]


def write_table(table: ColumnTable, out_dir: str, stem: str, formats) -> list:
    written = []
    for fmt in formats:
        path = os.path.join(out_dir, f"{stem}.{'parquet' if fmt == 'parquet' else 'csv'}")
        try:
            written.append(
                write_parquet(table, path) if fmt == "parquet" else write_csv(table, path)
            )
        except Exception as exc:  # noqa: BLE001
            print(f"  could not write {path}: {exc}", file=sys.stderr)
    return written


def time_columns(timestamp_ns: int, start_ns: int) -> dict:
    return {
        "t_ns": timestamp_ns,
        "t_rel": (timestamp_ns - start_ns) / 1e9,
        "t_iso": datetime.datetime.fromtimestamp(
            timestamp_ns / 1e9, datetime.timezone.utc
        ).isoformat(),
    }


def export_table_topic(reader, topic, info, decoder, flattener, start_ns) -> dict:
    table = ColumnTable()
    failures = 0
    first_error = None
    for timestamp_ns, blob in reader.messages(topic):
        row = time_columns(timestamp_ns, start_ns)
        try:
            row.update(flattener.flatten(decoder.decode(blob, info.type)))
        except DecodeError as exc:
            failures += 1
            if first_error is None:
                first_error = str(exc)
            row["decode_error"] = 1
        table.add(row)
    return {
        "table": table,
        "decode_failures": failures,
        "first_error": first_error,
        "decoder": decoder.path_for(info.type),
    }


def export_video_topic(reader, topic, out_dir, start_ns, mode, formats) -> dict:
    streams: dict = {}
    indexes: dict = {}
    malformed = 0
    first_error = None
    stem = topic_filename(topic)

    for timestamp_ns, blob in reader.messages(topic):
        try:
            frame = video_mod.parse_frame(blob)
        except video_mod.VideoError as exc:
            malformed += 1
            if first_error is None:
                first_error = str(exc)
            continue
        resolution = frame["resolution"]
        writer = streams.get(resolution)
        if writer is None:
            writer = streams[resolution] = video_mod.StreamWriter(
                os.path.join(out_dir, "video", f"{stem}_{resolution}p.h264")
            )
            indexes[resolution] = ColumnTable()
        entry = writer.add(timestamp_ns, frame["payload"])
        if entry is not None:
            row = time_columns(timestamp_ns, start_ns)
            row.update(
                {
                    "index": entry["index"],
                    "resolution": resolution,
                    "time_frame": frame["time_frame"],
                    "byte_offset": entry["byte_offset"],
                    "length": entry["length"],
                    "keyframe": entry["keyframe"],
                }
            )
            indexes[resolution].add(row)

    result = {"streams": [], "malformed": malformed, "first_error": first_error}
    for resolution, writer in sorted(streams.items()):
        writer.close()
        stream = {
            "resolution": resolution,
            "path": writer.path,
            "frames": writer.frames,
            "keyframes": writer.keyframes,
            "bytes": writer.bytes,
            "fps": round(writer.fps, 2),
            "dropped_before_keyframe": writer.skipped_before_keyframe,
        }
        write_table(
            indexes[resolution], os.path.join(out_dir, "video"), f"{stem}_{resolution}p_index", formats
        )
        if mode == "mp4":
            stream["mp4"] = video_mod.transcode_mp4(writer.path, writer.fps)
        result["streams"].append(stream)
    return result


def export_clouds(reader, topic, out_dir, decoder, info) -> dict:
    stem = topic_filename(topic)
    target = os.path.join(out_dir, "clouds", stem)
    written = 0
    failures = 0
    first_error = None
    for index, (timestamp_ns, blob) in enumerate(reader.messages(topic)):
        try:
            message = decoder.decode(blob, info.type)
            clouds_mod.write_frame(message, target, index, timestamp_ns)
            written += 1
        except (DecodeError, clouds_mod.CloudError, ValueError) as exc:
            failures += 1
            if first_error is None:
                first_error = str(exc)
    return {"frames": written, "failures": failures, "first_error": first_error, "dir": target}


def main(argv=None) -> int:
    args = parse_args(argv)

    try:
        bag_dir = find_bag_dir(args.session)
        reader = BagReader(bag_dir)
    except BagError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    session_dir = os.path.abspath(os.path.expanduser(args.session))
    out_dir = args.out or os.path.join(session_dir, "export")
    active = reader.active_topics()

    if args.list:
        print(f"{bag_dir}: {len(reader.topics)} topics, {len(active)} with data")
        for name, info in sorted(active.items(), key=lambda kv: -kv[1].count):
            print(f"  {info.count:>9}  {name}  ({info.type})")
        return 0

    include = re.compile(args.topics) if args.topics else None
    exclude = re.compile(args.exclude) if args.exclude else None
    selected = {
        name: info
        for name, info in active.items()
        if (include is None or include.search(name))
        and (exclude is None or not exclude.search(name))
    }
    if not selected:
        print("error: no topics with data matched", file=sys.stderr)
        return 2

    start_ns = reader.start_time() or 0
    formats = resolve_formats(args.format)
    skip = None if not (args.skip or args.no_default_skip) else args.skip
    if args.no_default_skip and not args.skip:
        skip = []
    registry = MsgRegistry(args.msg_path or None)
    try:
        decoder = Decoder(registry, args.decoder)
    except DecodeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    flattener = Flattener(
        max_array=args.max_array, skip=skip, expand_json=args.expand_json
    )

    os.makedirs(out_dir, exist_ok=True)
    report = {
        "session": session_dir,
        "bag_dir": bag_dir,
        "out_dir": out_dir,
        "formats": formats,
        "decoder": args.decoder,
        "msg_search_paths": registry.search_paths,
        "start_ns": start_ns,
        "topics": {},
        "video": {},
        "clouds": {},
    }
    print(f"{bag_dir} -> {out_dir} ({', '.join(formats)})")

    for name, info in sorted(selected.items()):
        stem = topic_filename(name)
        is_video = "Go2FrontVideoData" in info.type or name.endswith("frontvideostream")
        is_cloud = info.type in CLOUD_TYPES

        if is_video and args.video != "none":
            result = export_video_topic(reader, name, out_dir, start_ns, args.video, formats)
            report["video"][name] = result
            for stream in result["streams"]:
                note = ""
                if stream["dropped_before_keyframe"]:
                    note = f", {stream['dropped_before_keyframe']} before first keyframe dropped"
                print(
                    f"  {name} {stream['resolution']}p: {stream['frames']} frames, "
                    f"{stream['fps']} Hz{note}"
                )
            if result["malformed"]:
                print(
                    f"    {result['malformed']} messages did not match the layout: "
                    f"{result['first_error']}",
                    file=sys.stderr,
                )
            continue

        if is_cloud and args.clouds:
            if not clouds_mod.numpy_available():
                print("  numpy is not installed; skipping --clouds", file=sys.stderr)
            else:
                result = export_clouds(reader, name, out_dir, decoder, info)
                report["clouds"][name] = result
                print(f"  {name}: {result['frames']} cloud frames -> {result['dir']}")

        outcome = export_table_topic(reader, name, info, decoder, flattener, start_ns)
        table = outcome["table"]
        paths = write_table(table, out_dir, stem, formats)
        report["topics"][name] = {
            "type": info.type,
            "messages": info.count,
            "rows": len(table),
            "columns": len(table.columns),
            "decoder": outcome["decoder"],
            "decode_failures": outcome["decode_failures"],
            "first_error": outcome["first_error"],
            "files": paths,
        }
        suffix = ""
        if outcome["decode_failures"]:
            suffix = f", {outcome['decode_failures']} failed to decode"
        print(f"  {name}: {len(table)} rows, {len(table.columns)} columns{suffix}")
        if outcome["first_error"]:
            print(f"    first error: {outcome['first_error']}", file=sys.stderr)

    report_path = os.path.join(out_dir, "export_report.json")
    with open(report_path, "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
