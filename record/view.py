#!/usr/bin/env python3
"""Visualize a recorded Go2 session in Rerun, served to a remote client.

    python3 record/view.py                        # pick a session, pick telemetry
    python3 record/view.py walk_test_20261023_143002
    python3 record/view.py <session> --telemetry pose,uwb,video
    python3 record/view.py <session> --serve web  # remote client just opens a browser

The server keeps the recording in memory and waits; connect the Rerun viewer
to the URL it prints. Sessions are read with `go2export`, so no ROS is needed
and this runs wherever the logs are mounted.
"""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rerun as rr  # noqa: E402

from go2export.bagreader import BagError, BagReader, find_bag_dir  # noqa: E402
from go2export.clouds import numpy_available  # noqa: E402
from go2export.decode import DecodeError, Decoder  # noqa: E402
from go2export.msgdef import DEFAULT_SEARCH_PATHS, MsgRegistry  # noqa: E402
from go2view import layout, streams  # noqa: E402
from go2view.sessions import (  # noqa: E402
    SessionError,
    default_logs_root,
    human_size,
    resolve_session,
)

APP_ID = "go2-telemetry"


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize a recorded Go2 session in Rerun.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "session",
        nargs="?",
        help="session directory, or its name under --logs-root; omit to choose from a list",
    )
    parser.add_argument(
        "--logs-root",
        default=default_logs_root(),
        help="where the sessions live (also $GO2_LOGS)",
    )
    parser.add_argument(
        "--telemetry",
        help="comma separated group keys, or 'all'; omit to choose from a list",
    )
    parser.add_argument(
        "--list-telemetry", action="store_true", help="list what this session has, then stop"
    )
    parser.add_argument(
        "--serve",
        choices=("grpc", "web", "spawn", "save"),
        default="grpc",
        help="grpc waits for a viewer; web also hosts the viewer; spawn opens one locally; "
        "save writes an .rrd",
    )
    parser.add_argument("--port", type=int, default=9876, help="gRPC port clients connect to")
    parser.add_argument("--web-port", type=int, default=9090, help="port for --serve web")
    parser.add_argument(
        "--advertise-host",
        help="host a remote client should dial (default: this machine's LAN address)",
    )
    parser.add_argument(
        "--memory-limit",
        default="2GiB",
        help="how much recording the server keeps before dropping the oldest",
    )
    parser.add_argument("--rrd", help="output path for --serve save")
    parser.add_argument(
        "--yes",
        "-y",
        action="store_true",
        help="take the defaults instead of asking (newest session, all telemetry)",
    )

    group = parser.add_argument_group("data selection")
    group.add_argument("--start", type=float, default=0.0, help="skip this many seconds in")
    group.add_argument(
        "--duration", type=float, default=0.0, help="only this many seconds (0 means all)"
    )
    group.add_argument(
        "--cloud-stride", type=int, default=1, help="keep every Nth point cloud frame"
    )
    group.add_argument(
        "--max-cloud-points", type=int, default=0, help="subsample clouds above this size (0 means all)"
    )
    group.add_argument("--trail-length", type=int, default=600, help="points kept in the pose trail")
    group.add_argument(
        "--trail-every", type=int, default=25, help="redraw the trail every Nth pose message"
    )

    group = parser.add_argument_group("decoding")
    group.add_argument(
        "--msg-path",
        action="append",
        default=[],
        help=f"where to look for .msg files (default: {', '.join(DEFAULT_SEARCH_PATHS)})",
    )
    group.add_argument("--decoder", choices=("auto", "ros", "msgdef"), default="auto")

    group = parser.add_argument_group("UWB mapping (as in uwb_rviz/)")
    group.add_argument("--bearing-field", default="orientation_est")
    group.add_argument("--bearing-sign", type=float, default=1.0)
    group.add_argument("--bearing-offset", type=float, default=0.0)
    group.add_argument(
        "--elevation-field", default="pitch_est", help="'none' puts the remote on the ground"
    )
    group.add_argument("--quaternion-order", choices=("wxyz", "xyzw"), default="wxyz")

    return parser.parse_args(argv)


def local_ip() -> str:
    """The address a client on the same network would dial."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(("8.8.8.8", 80))
        return sock.getsockname()[0]
    except OSError:
        return socket.gethostname()
    finally:
        sock.close()


def advertised(url: str, host: str) -> str:
    """Rerun reports 127.0.0.1 even though it listens on every interface."""
    return url.replace("127.0.0.1", host).replace("0.0.0.0", host)


def choose_telemetry(available, argument: str | None, interactive: bool) -> list:
    """Pick groups from the command line, or ask."""
    keys = [group.key for group, _ in available]
    if argument:
        if argument.strip().lower() in ("all", "*"):
            return keys
        wanted, unknown = [], []
        for raw in argument.replace(" ", ",").split(","):
            token = raw.strip()
            if not token:
                continue
            if token in keys:
                wanted.append(token)
            elif token.isdigit() and 1 <= int(token) <= len(keys):
                wanted.append(keys[int(token) - 1])
            else:
                unknown.append(token)
        if unknown:
            raise SystemExit(
                f"error: no telemetry {', '.join(unknown)} in this session; "
                f"available: {', '.join(keys)}"
            )
        return list(dict.fromkeys(wanted))
    if not interactive:
        return keys

    print("\nTelemetry in this session:\n")
    width = max(len(group.key) for group, _ in available)
    for index, (group, topics) in enumerate(available, 1):
        shown = ", ".join(topics[:3]) + (" ..." if len(topics) > 3 else "")
        print(f"  {index:>2}  {group.key:<{width}}  {group.title}")
        print(f"      {' ' * width}  {shown}")
    print()
    while True:
        try:
            answer = input("Telemetry [numbers or keys, default all]: ").strip()
        except EOFError:
            answer = ""
        if not answer or answer.lower() == "all":
            return keys
        try:
            return choose_telemetry(available, answer, interactive=False)
        except SystemExit as exc:
            print(f"  {exc}".replace("error: ", ""))


def main(argv=None) -> int:
    args = parse_args(argv)
    interactive = sys.stdin.isatty() and not args.yes

    try:
        session = resolve_session(args.session, args.logs_root, interactive=interactive)
        bag_dir = find_bag_dir(session.path)
        reader = BagReader(bag_dir)
    except (SessionError, BagError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    active = reader.active_topics()
    available = streams.available_groups(active, session.path)
    if not available:
        print(f"error: {session.name} has no telemetry this viewer knows about", file=sys.stderr)
        return 2

    print(f"\n{session.name}  ({human_size(session.size_bytes)}, {len(active)} topics with data)")

    if args.list_telemetry:
        for group, topics in available:
            total = sum(active[t].count for t in topics if t in active)
            count = f"{total} msgs" if total else "from file"
            print(f"  {group.key:<11} {group.title:<38} {count}")
            for topic in topics:
                print(f"              {topic}")
        return 0

    selected_keys = choose_telemetry(available, args.telemetry, interactive)
    groups = [group for group, _ in available if group.key in selected_keys]
    if not groups:
        print("error: nothing selected", file=sys.stderr)
        return 2

    dropped = [g.key for g in groups if g.needs_numpy and not numpy_available()]
    if dropped:
        print(f"  numpy is not installed, skipping: {', '.join(dropped)}", file=sys.stderr)
        groups = [g for g in groups if g.key not in dropped]
        selected_keys = [k for k in selected_keys if k not in dropped]

    options = streams.Options(
        bearing_field=args.bearing_field,
        bearing_sign=args.bearing_sign,
        bearing_offset=args.bearing_offset,
        elevation_field=args.elevation_field,
        quaternion_order=args.quaternion_order,
        cloud_stride=max(1, args.cloud_stride),
        max_cloud_points=max(0, args.max_cloud_points),
        trail_every=max(0, args.trail_every),
        trail_length=max(2, args.trail_length),
        start_seconds=max(0.0, args.start),
        duration_seconds=max(0.0, args.duration),
    )
    ctx = streams.Context(options=options, start_ns=reader.start_time() or 0)

    registry = MsgRegistry(args.msg_path or None)
    try:
        decoder = Decoder(registry, args.decoder)
    except DecodeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    # The sink has to exist before anything is logged, so the server starts
    # first and holds the recording until a viewer connects.
    rr.init(APP_ID, recording_id=session.name)
    url = None
    if args.serve in ("grpc", "web"):
        url = rr.serve_grpc(grpc_port=args.port, server_memory_limit=args.memory_limit)
    elif args.serve == "spawn":
        rr.spawn(memory_limit=args.memory_limit)
    else:
        target = args.rrd or os.path.join(session.path, f"{session.name}.rrd")
        rr.save(target)
        print(f"  writing {target}")

    print(f"  telemetry: {', '.join(selected_keys)}")
    print("  reading:")

    def progress(topic, info):
        if info is None:
            print(f"    {topic}")
        else:
            print(f"    {topic}  ({info.count} msgs, {info.type})")

    try:
        streams.stream(reader, decoder, groups, ctx, session.path, progress=progress)
    except KeyboardInterrupt:
        print("\n  interrupted while reading; serving what was logged", file=sys.stderr)

    video_paths = sorted(ctx._video_started)
    rr.send_blueprint(
        layout.build(selected_keys, video_paths=video_paths, has_events=ctx.counters.get("events", 0) > 0)
    )
    rr.get_global_data_recording().flush(timeout_sec=30)

    logged = {k: v for k, v in sorted(ctx.counters.items()) if not k.endswith(":seen")}
    print("\n  logged: " + (", ".join(f"{k}={v}" for k, v in logged.items()) or "nothing"))
    for key, message in sorted(ctx.errors.items()):
        print(f"  problem on {key}: {message}", file=sys.stderr)

    if args.serve == "save":
        return 0
    if args.serve == "spawn":
        print("\n  sent to the local viewer")
        return 0

    host = args.advertise_host or local_ip()
    remote = advertised(url, host)
    print("\n  Rerun server is up. Connect a remote client with:\n")
    print(f"    rerun --connect {remote}")
    print(f"\n  or paste {remote} into the viewer's \"Add connection\".")
    if args.serve == "web":
        rr.serve_web_viewer(web_port=args.web_port, open_browser=False, connect_to=remote)
        print(f"\n  Or just open a browser at:  http://{host}:{args.web_port}")
    print("\n  Ctrl+C to stop serving.\n")

    try:
        while True:
            time.sleep(3600)
    except KeyboardInterrupt:
        print("stopped")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
