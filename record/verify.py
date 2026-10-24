#!/usr/bin/env python3
"""Check that the readings in a recording session are plausible, not just present.

Usage: python3 verify.py logs/<session>
Decodes the key topics, prints value ranges with OK / WARN per check, and writes
logs/<session>/meta/verify.txt. Also summarizes Jetson hardware (meta/hardware.csv,
or meta/tegrastats.log for sessions recorded before hardware.csv existed).
"""
import csv
import glob
import json
import math
import os
import re
import sqlite3
import struct
import sys
from collections import Counter

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

G = 9.81


class Report:
    def __init__(self):
        self.lines, self.warns = [], 0

    def section(self, title):
        self.lines.append(f"\n== {title}")

    def info(self, text):
        self.lines.append(f"   {text}")

    def check(self, ok, text):
        self.lines.append(f"   {'OK  ' if ok else 'WARN'} {text}")
        self.warns += not ok
        return ok


class Bag:
    def __init__(self, session):
        files = glob.glob(os.path.join(session, "go2", "*.db3"))
        self.files = sorted(files, key=lambda p: int(p.rsplit("_", 1)[1].split(".")[0]))
        self.types = {}
        for path in self.files:
            db = self.open(path)
            self.types.update(dict(db.execute("select name, type from topics")))
            db.close()

    @staticmethod
    def open(path):
        return sqlite3.connect(f"file:{path}?mode=ro&immutable=1", uri=True)

    def raw(self, topic, every=1):
        """(receive time s, serialized bytes) for every n-th message."""
        out = []
        for path in self.files:
            db = self.open(path)
            q = ("select m.timestamp, m.data from messages m join topics t on t.id = m.topic_id "
                 "where t.name = ? order by m.timestamp")
            out += [(ts * 1e-9, data) for i, (ts, data) in enumerate(db.execute(q, (topic,))) if i % every == 0]
            db.close()
        return out

    def msgs(self, topic, every=1):
        if topic not in self.types:
            return []
        mtype = get_message(self.types[topic])
        return [(t, deserialize_message(d, mtype)) for t, d in self.raw(topic, every)]


def rng(a, fmt="{:.2f}"):
    a = np.asarray(a, dtype=float)
    return f"{fmt.format(a.min())} .. {fmt.format(a.max())} (mean {fmt.format(a.mean())})"


def check_sport(r, bag):
    r.section("Body state  /sportmodestate")
    ms = bag.msgs("/sportmodestate", every=10)   # 30 Hz is plenty for ranges
    if not r.check(bool(ms), f"{len(ms)} messages decoded (every 10th)"):
        return
    m = [x for _, x in ms]
    err = Counter(x.error_code for x in m)
    r.check(set(err) == {0}, f"error_code values {dict(err)}")
    r.info(f"mode values {dict(Counter(x.mode for x in m))}, gait_type {dict(Counter(x.gait_type for x in m))}")
    pos = np.array([x.position for x in m])
    r.info(f"position x {rng(pos[:, 0])}  y {rng(pos[:, 1])}  z {rng(pos[:, 2])} m")
    r.info(f"displacement start->end {np.linalg.norm(pos[-1, :2] - pos[0, :2]):.2f} m, "
           f"path length {np.sum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1)):.2f} m")
    vel = np.array([x.velocity for x in m])
    speed = np.linalg.norm(vel[:, :2], axis=1)
    r.check(speed.max() < 4.0, f"horizontal speed {rng(speed)} m/s (Go2 max ~3.7)")
    r.info(f"yaw_speed {rng([x.yaw_speed for x in m])} rad/s")
    bh = np.array([x.body_height for x in m])
    r.check(bh.min() > 0.0 and bh.max() < 0.6, f"body_height {rng(bh, '{:.3f}')} m")
    rpy = np.degrees([x.imu_state.rpy for x in m])
    r.check(np.abs(rpy[:, :2]).max() < 45,
            f"roll {rng(rpy[:, 0], '{:.1f}')}  pitch {rng(rpy[:, 1], '{:.1f}')}  yaw {rng(rpy[:, 2], '{:.1f}')} deg")
    ff = np.array([x.foot_force for x in m])
    r.info(f"foot_force per leg max {ff.max(axis=0).tolist()}  mean {ff.mean(axis=0).round().tolist()}")


def check_lowstate(r, bag):
    r.section("Low-rate state  /lf/lowstate (IMU, battery, motors)")
    ms = bag.msgs("/lf/lowstate")
    if not r.check(bool(ms), f"{len(ms)} messages decoded"):
        return
    m = [x for _, x in ms]
    acc = np.array([x.imu_state.accelerometer for x in m])
    an = np.linalg.norm(acc, axis=1)
    r.check(abs(np.median(an) - G) < 0.6, f"IMU accel |a| {rng(an)} m/s^2 (expect ~{G} at rest)")
    gyro = np.degrees(np.linalg.norm([x.imu_state.gyroscope for x in m], axis=1))
    r.info(f"IMU gyro |w| {rng(gyro, '{:.1f}')} deg/s")
    q = np.array([x.imu_state.quaternion for x in m])
    qn = np.linalg.norm(q, axis=1)
    r.check(np.all(np.abs(qn - 1) < 0.01), f"IMU quaternion norm {rng(qn, '{:.4f}')}")
    r.check(max(x.imu_state.temperature for x in m) < 80, f"IMU temperature {rng([x.imu_state.temperature for x in m], '{:.0f}')} C")

    soc = [x.bms_state.soc for x in m]
    r.check(0 < min(soc) and max(soc) <= 100, f"battery SOC {min(soc)} .. {max(soc)} %  (drop {soc[0] - soc[-1]} %)")
    pv = [x.power_v for x in m]
    r.check(min(pv) > 24 and max(pv) < 34, f"power_v {rng(pv)} V (Go2 8S pack: ~25-33.6)")
    r.info(f"power_a {rng([x.power_a for x in m])} A,  bms current {rng([x.bms_state.current for x in m], '{:.0f}')} mA")
    cells = np.array([[c for c in x.bms_state.cell_vol if c] for x in m if any(x.bms_state.cell_vol)])
    if cells.size:
        spread = cells.max(axis=1) - cells.min(axis=1)
        r.check(spread.max() < 100, f"{cells.shape[1]} cells {rng(cells, '{:.0f}')} mV, max spread {spread.max()} mV")
    ntc = [x.bms_state.bq_ntc[0] for x in m] + [x.bms_state.mcu_ntc[0] for x in m]
    r.check(max(ntc) < 60, f"battery temperatures (bq/mcu ntc) {rng(ntc, '{:.0f}')} C")

    mt = np.array([[ms_.temperature for ms_ in x.motor_state[:12]] for x in m])
    r.check(mt.max() < 70, f"motor temperatures (12 joints) {rng(mt, '{:.0f}')} C, hottest joint #{int(mt.max(axis=0).argmax())}")
    r.info(f"board ntc1 {rng([x.temperature_ntc1 for x in m], '{:.0f}')} C, ntc2 {rng([x.temperature_ntc2 for x in m], '{:.0f}')} C")
    ticks = np.diff([x.tick for x in m])
    r.check(np.all(ticks > 0), f"tick strictly increasing (step {rng(ticks, '{:.0f}')} ms)")


def check_lidar(r, bag):
    r.section("Lidar  /utlidar/*")
    imu = bag.msgs("/utlidar/imu", every=25)
    if imu:
        an = np.linalg.norm([[x.linear_acceleration.x, x.linear_acceleration.y, x.linear_acceleration.z] for _, x in imu], axis=1)
        r.check(abs(np.median(an) - G) < 0.6, f"lidar IMU |a| {rng(an)} m/s^2")
    st = bag.msgs("/utlidar/lidar_state")
    if st:
        s = [x for _, x in st]
        r.info(f"firmware {s[0].firmware_version}, software {s[0].software_version}")
        r.check(all(x.error_state == 0 for x in s), f"error_state {dict(Counter(x.error_state for x in s))}")
        # Values look like percent (1-6 on a healthy standing robot)
        r.check(max(x.cloud_packet_loss_rate for x in s) < 10,
                f"cloud packet loss {rng([x.cloud_packet_loss_rate for x in s])} %, "
                f"imu packet loss {rng([x.imu_packet_loss_rate for x in s])} %")
        r.info(f"cloud_frequency {rng([x.cloud_frequency for x in s], '{:.1f}')} Hz, "
               f"rotation {rng([x.sys_rotation_speed for x in s], '{:.0f}')}")
    for topic in ("/utlidar/cloud", "/utlidar/cloud_base", "/utlidar/cloud_deskewed"):
        cl = bag.msgs(topic, every=15)
        if cl:
            n = [x.width * x.height for _, x in cl]
            r.check(min(n) > 0, f"{topic}: points per frame {rng(n, '{:.0f}')}, frame '{cl[0][1].header.frame_id}'")
    odom = bag.msgs("/utlidar/robot_odom", every=5)
    if odom:
        p = np.array([[x.pose.pose.position.x, x.pose.pose.position.y] for _, x in odom])
        r.info(f"robot_odom displacement {np.linalg.norm(p[-1] - p[0]):.2f} m, "
               f"path {np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)):.2f} m, frame '{odom[0][1].header.frame_id}'")


def check_uwb(r, bag):
    r.section("UWB  /uwbstate")
    ms = bag.msgs("/uwbstate")
    if not r.check(bool(ms), f"{len(ms)} messages decoded"):
        return
    m = [x for _, x in ms]
    r.info(f"version {[int(v) for v in m[0].version]}, channel {m[0].channel}, joy_mode {dict(Counter(x.joy_mode for x in m))}")
    r.check(all(x.error_state == 0 for x in m), f"error_state {dict(Counter(x.error_state for x in m))}")
    r.check(all(x.enabled_from_app for x in m), f"enabled_from_app {dict(Counter(x.enabled_from_app for x in m))}")
    d = [x.distance_est for x in m]
    r.check(0 < min(d) and max(d) < 50, f"distance_est {rng(d)} m")
    for f in ("orientation_est", "pitch_est", "yaw_est", "tag_roll", "tag_pitch", "tag_yaw"):
        r.info(f"{f:16s} {rng(np.degrees([getattr(x, f) for x in m]), '{:.1f}')} deg")
    jx = np.array([list(x.joystick) for x in m])
    r.info(f"joystick {rng(jx, '{:.2f}')}, buttons seen {sorted({x.buttons for x in m})}")
    t = np.array([t for t, _ in ms])
    gaps = np.diff(t)
    r.check(gaps.max() < 1.5, f"rate {len(t) / (t[-1] - t[0]):.1f} Hz, gaps > 0.5 s: {int((gaps > 0.5).sum())}, max {gaps.max():.2f} s")


def check_misc(r, bag):
    r.section("Battery cells, commands, media")
    alarms = [json.loads(x.data) for _, x in bag.msgs("/lf/battery_alarm")]
    if alarms:
        r.check(all(a.get("alarm_status") == 0 for a in alarms),
                f"battery_alarm status {dict(Counter(a.get('alarm_status') for a in alarms))}, last '{alarms[-1].get('description')}'")
    states = [x.data for _, x in bag.msgs("/multiplestate")]
    if states:
        r.info(f"multiplestate (last) {states[-1]}")
    for topic in sorted(t for t in bag.types if t.startswith("/api/") and t.endswith("/request")):
        ids = Counter(x.header.identity.api_id for _, x in bag.msgs(topic))
        if ids:
            r.info(f"{topic:30s} api_id counts {dict(ids)}")
    # The robot's video message is not the local Go2FrontVideoData: after the CDR header and
    # uint64 time_frame come uint32 resolution (720 / 360) and uint32 length, then H.264 Annex-B
    video = bag.raw("/frontvideostream")
    if video:
        res, keyframes = Counter(), Counter()
        for _, d in video:
            rs, n = struct.unpack_from("<II", d, 12)
            res[rs] += 1
            keyframes[rs] += d[20:20 + n].find(b"\x00\x00\x00\x01\x67") >= 0   # SPS = keyframe
        ok = all(d[20:24] == b"\x00\x00\x00\x01" for _, d in video[:50])
        r.check(ok and bool(res), f"/frontvideostream: H.264 frames per stream {dict(res)}, keyframes {dict(keyframes)}")
    audio = bag.raw("/audiosender")
    if audio:
        sz = [len(d) for _, d in audio]
        r.check(min(sz) > 64, f"/audiosender: {len(audio)} frames, {rng(sz, '{:.0f}')} bytes/frame (format unknown)")


def check_hardware(r, session):
    r.section("Jetson hardware")
    hw = os.path.join(session, "meta", "hardware.csv")
    if os.path.exists(hw):
        rows = list(csv.DictReader(open(hw)))
        col = lambda k: [float(x[k]) for x in rows if x.get(k) not in ("", None)]
        r.info(f"{len(rows)} samples from hardware.csv")
        r.check(max(col("cpu_pct")) < 90, f"CPU total {rng(col('cpu_pct'), '{:.0f}')} %")
        r.info(f"recorder process CPU {rng(col('rec_cpu_pct'), '{:.0f}')} % (100 = one core), RSS {rng(col('rec_rss_mb'), '{:.0f}')} MB")
        r.check(min(col("ram_avail_mb")) > 500, f"RAM used {rng(col('ram_used_mb'), '{:.0f}')} MB, min available {min(col('ram_avail_mb')):.0f} MB")
        r.info(f"GPU {rng(col('gpu_pct'), '{:.0f}')} %")
        temps = {k: max(col(k)) for k in rows[0] if k.startswith("temp_")}
        r.check(max(temps.values()) < 85, "max temps " + ", ".join(f"{k[5:-2]} {v:.0f}C" for k, v in temps.items()))
        for k in (k for k in rows[0] if k.startswith("power_")):
            r.info(f"{k[6:-3]:16s} {rng(col(k), '{:.0f}')} mW")
        r.info(f"disk write {rng(col('disk_write_mb_s'))} MB/s, free {min(col('disk_free_gb')):.0f} GB")
        r.info(f"enP8p1s0 rx {rng(col('enP8p1s0_rx_mb_s'))} MB/s")
        return
    tl = os.path.join(session, "meta", "tegrastats.log")
    if not os.path.exists(tl):
        r.check(False, "no hardware.csv or tegrastats.log")
        return
    cpu, ram, tj, pin = [], [], [], []
    for line in open(tl):
        cores = [int(c) for c in re.findall(r"(\d+)%@\d+", line.split("CPU [")[1].split("]")[0])]
        cpu.append(sum(cores) / len(cores))
        ram.append(int(re.search(r"RAM (\d+)/", line).group(1)))
        tj.append(float(re.search(r"tj@([\d.]+)C", line).group(1)))
        pin.append(int(re.search(r"VDD_IN (\d+)mW", line).group(1)))
    r.info(f"{len(cpu)} samples from tegrastats.log (no hardware.csv in this session)")
    r.check(max(cpu) < 90, f"CPU total {rng(cpu, '{:.0f}')} %")
    r.info(f"RAM used {rng(ram, '{:.0f}')} MB")
    r.check(max(tj) < 85, f"junction temp {rng(tj, '{:.1f}')} C")
    r.info(f"VDD_IN {rng(pin, '{:.0f}')} mW")


def main():
    session = sys.argv[1].rstrip("/")
    bag = Bag(session)
    if not bag.files:
        sys.exit(f"no .db3 files in {session}/go2")
    r = Report()
    r.lines.append(f"Session {os.path.basename(session)}")
    for fn in (check_sport, check_lowstate, check_lidar, check_uwb, check_misc):
        try:
            fn(r, bag)
        except Exception as e:   # one broken topic should not hide the other checks
            r.check(False, f"{fn.__name__} failed: {type(e).__name__}: {e}")
    check_hardware(r, session)
    r.lines.append(f"\n{r.warns} warning(s)")
    text = "\n".join(r.lines)
    print(text)
    with open(os.path.join(session, "meta", "verify.txt"), "w") as f:
        f.write(text + "\n")


if __name__ == "__main__":
    main()
