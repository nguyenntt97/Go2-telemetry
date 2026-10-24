#!/usr/bin/env python3
"""Per-topic summary of a recording session: count, rate, largest gap, size.

Usage: python3 summarize.py logs/<session>
Writes logs/<session>/meta/summary.csv and prints the table.
Flags topics that are slower than the rates measured on 2026-10-23 (see README).
"""
import csv
import glob
import os
import sqlite3
import sys

# Rates measured on this robot (Hz); used to flag dropouts
EXPECTED_HZ = {
    "/sportmodestate": 300, "/lf/sportmodestate": 20, "/lf/lowstate": 20,
    "/utlidar/imu": 250, "/utlidar/robot_odom": 150, "/utlidar/robot_pose": 19,
    "/utlidar/lidar_state": 5, "/utlidar/cloud": 15, "/utlidar/cloud_base": 15,
    "/utlidar/cloud_deskewed": 15, "/frontvideostream": 29, "/audiosender": 50,
    "/uwbstate": 5,
}

GAP_SQL = """
select topic_id, count(*), sum(length(data)), min(timestamp), max(timestamp), max(timestamp - prev)
from (select topic_id, timestamp, data,
             lag(timestamp) over (partition by topic_id order by timestamp) as prev
      from messages)
group by topic_id
"""


def main():
    session = sys.argv[1].rstrip("/")
    files = sorted(glob.glob(os.path.join(session, "go2", "*.db3")),
                   key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))
    if not files:
        sys.exit(f"no .db3 files in {session}/go2")

    stats = {}   # topic -> [count, bytes, first, last, max_gap, type]
    for path in files:
        # immutable: read the closed bag without creating -shm/-wal files next to it
        db = sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)
        topics = {tid: (name, mtype) for tid, name, mtype in db.execute("select id, name, type from topics")}
        for tid, n, size, first, last, gap in db.execute(GAP_SQL):
            name, mtype = topics[tid]
            s = stats.get(name)
            if s is None:
                stats[name] = [n, size, first, last, gap or 0, mtype]
                continue
            # Include the gap across the split-file boundary
            s[4] = max(s[4], gap or 0, first - s[3])
            s[0] += n
            s[1] += size
            s[3] = last
        db.close()

    rows = []
    for name, (n, size, first, last, gap, mtype) in sorted(stats.items()):
        dur = (last - first) * 1e-9
        hz = (n - 1) / dur if n > 1 and dur > 0 else 0.0
        exp = EXPECTED_HZ.get(name)
        flag = "LOW_RATE" if exp and hz < 0.8 * exp else ""
        rows.append([name, mtype, n, f"{hz:.2f}", f"{gap * 1e-9:.3f}", f"{size / 1024:.1f}", flag])
    missing = [t for t in EXPECTED_HZ if t not in stats]

    out = os.path.join(session, "meta", "summary.csv")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["topic", "type", "count", "rate_hz", "max_gap_s", "size_kb", "flag"])
        w.writerows(rows)
        for t in missing:
            w.writerow([t, "", 0, "0", "", "0", "NO_DATA"])

    print(f"{'topic':40s} {'count':>8s} {'Hz':>8s} {'max gap s':>10s} {'KB':>10s}  flag")
    for name, _, n, hz, gap, kb, flag in rows:
        print(f"{name:40s} {n:8d} {hz:>8s} {gap:>10s} {kb:>10s}  {flag}")
    for t in missing:
        print(f"{t:40s} {0:8d} {'':>8s} {'':>10s} {'':>10s}  NO_DATA (excluded by profile, or not publishing)")
    print(f"-> {out}")


if __name__ == "__main__":
    main()
