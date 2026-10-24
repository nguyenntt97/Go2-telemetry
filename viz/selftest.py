#!/usr/bin/env python3
"""Check the hand-written layouts in msgs.py against a real session.

cdr.compile already refuses a schema that does not account for every byte, but the
right size does not prove the right field order. These checks decode real messages
and assert the values are physically sensible: a quaternion norm of 1, gravity at
9.81 m/s^2, a battery between 0 and 100 %. Wrong field order fails them.

Usage: python3 selftest.py [logs/<session>]
"""
import sys

import numpy as np

import bag

G = 9.81
FAILED = []


def check(ok, text):
    print(f"   {'OK  ' if ok else 'FAIL'} {text}")
    if not ok:
        FAILED.append(text)


def main():
    logs = "/media/nguyen/project_data/go2/logs"
    if len(sys.argv) > 1:
        session = bag.Session(sys.argv[1])
    else:
        session = next(s for s in bag.sessions(logs) if "Uwb" in s.name)
    print(f"Session {session.name}")

    print("\n== unitree_go/msg/SportModeState  /sportmodestate")
    t, m = session.decode("/sportmodestate", every=50, limit=2000)
    check(m is not None and len(m) > 100, f"{0 if m is None else len(m)} messages decoded")
    if m is not None:
        q = np.linalg.norm(m["imu_state"]["quaternion"], axis=1)
        check(np.allclose(q, 1, atol=0.01), f"IMU quaternion norm {q.min():.4f}..{q.max():.4f}")
        bh = m["body_height"]
        check(0 < bh.min() and bh.max() < 0.6, f"body_height {bh.min():.3f}..{bh.max():.3f} m")
        rpy = np.degrees(m["imu_state"]["rpy"])
        check(np.abs(rpy[:, :2]).max() < 45,
              f"roll/pitch within +-45 deg (max {np.abs(rpy[:, :2]).max():.1f})")
        spd = np.linalg.norm(m["velocity"][:, :2], axis=1)
        check(spd.max() < 4.0, f"horizontal speed {spd.min():.2f}..{spd.max():.2f} m/s")
        # error_code is the robot's own: 100 in every session here, plus 2007 in the
        # UWB one. Not a decode problem -- `stamp` beside it tracks the bag duration.
        print(f"   --   error_code (robot's) {np.unique(m['error_code']).tolist()}, "
              f"mode {np.unique(m['mode']).tolist()} (see msgs.py note)")
        check(m["gait_type"].max() < 10, f"gait_type {np.unique(m['gait_type']).tolist()}")
        span = m["stamp"]["sec"].max() - m["stamp"]["sec"].min()
        check(abs(span - t[-1]) < 120, f"stamp spans {span} s vs {t[-1]:.0f} s of bag")
        check(np.abs(m["position"]).max() < 500, f"position |x| max {np.abs(m['position']).max():.2f} m")

    print("\n== unitree_go/msg/LowState  /lf/lowstate")
    t, m = session.decode("/lf/lowstate", every=1, limit=4000)
    check(m is not None and len(m) > 100, f"{0 if m is None else len(m)} messages decoded")
    if m is not None:
        a = np.linalg.norm(m["imu_state"]["accelerometer"], axis=1)
        check(abs(np.median(a) - G) < 0.6, f"|accel| median {np.median(a):.2f} m/s^2 (gravity {G})")
        q = np.linalg.norm(m["imu_state"]["quaternion"], axis=1)
        check(np.allclose(q, 1, atol=0.01), f"IMU quaternion norm {q.min():.4f}..{q.max():.4f}")
        soc = m["bms_state"]["soc"]
        check(0 < soc.min() and soc.max() <= 100, f"battery SOC {soc.min()}..{soc.max()} %")
        pv = m["power_v"]
        check(24 < pv.min() and pv.max() < 34, f"power_v {pv.min():.1f}..{pv.max():.1f} V (8S pack)")
        cells = m["bms_state"]["cell_vol"][:, :8]
        check(2500 < cells.min() and cells.max() < 4300,
              f"cell voltages {cells.min()}..{cells.max()} mV")
        mt = m["motor_state"]["temperature"][:, :12]
        check(0 < mt.min() and mt.max() < 80, f"motor temperatures {mt.min()}..{mt.max()} C")
        nt = np.concatenate([m["temperature_ntc1"], m["temperature_ntc2"]])
        check(0 < nt.min() and nt.max() < 80, f"board temperatures {nt.min()}..{nt.max()} C")
        bt = np.concatenate([m["bms_state"]["bq_ntc"].ravel(), m["bms_state"]["mcu_ntc"].ravel()])
        check(0 < bt.min() and bt.max() < 60, f"battery temperatures {bt.min()}..{bt.max()} C")
        dt = np.diff(m["tick"].astype(np.int64))
        check(np.all(dt > 0) and 30 < np.median(dt) < 70,
              f"tick increasing, median step {np.median(dt):.0f} ms (20 Hz topic)")
        mq = m["motor_state"]["q"][:, :12]
        check(np.abs(mq).max() < 3.2, f"joint angles |q| max {np.abs(mq).max():.2f} rad")
        check(not m["motor_state"]["q"][:, 12:].any(), "joints 12..19 unused and zero")
        thigh = mq[:, 1::3].mean()
        calf = mq[:, 2::3].mean()
        check(0.4 < thigh < 1.2 and -2.0 < calf < -1.0,
              f"standing pose: mean thigh {thigh:.2f}, mean calf {calf:.2f} rad")

    print("\n== unitree_go/msg/UwbState  /uwbstate")
    t, m = session.decode("/uwbstate")
    if m is None:
        print("   -- not in this session")
    else:
        check(len(m) > 10, f"{len(m)} messages decoded")
        d = m["distance_est"]
        check(0 < d.min() and d.max() < 50, f"distance_est {d.min():.2f}..{d.max():.2f} m")
        check(set(np.unique(m["error_state"])) == {0}, f"error_state {np.unique(m['error_state']).tolist()}")
        for f in ("orientation_est", "pitch_est", "yaw_est", "tag_yaw", "base_yaw"):
            v = np.degrees(m[f])
            check(np.abs(v).max() <= 360.1, f"{f:16s} {v.min():8.1f}..{v.max():8.1f} deg")
        j = m["joystick"]
        check(np.abs(j).max() <= 1.5, f"joystick within +-1.5 (max {np.abs(j).max():.2f})")

    print("\n== sensor_msgs/msg/Imu  /utlidar/imu")
    t, m = session.decode("/utlidar/imu", every=25)
    if m is not None:
        a = np.linalg.norm(m["linear_acceleration"], axis=1)
        check(abs(np.median(a) - G) < 0.6, f"|accel| median {np.median(a):.2f} m/s^2")
        q = np.linalg.norm(m["orientation"], axis=1)
        check(np.allclose(q, 1, atol=0.01), f"quaternion norm {q.min():.4f}..{q.max():.4f}")

    print("\n== nav_msgs/msg/Odometry  /utlidar/robot_odom")
    t, m = session.decode("/utlidar/robot_odom", every=20)
    if m is not None:
        q = np.linalg.norm(m["orientation"], axis=1)
        check(np.allclose(q, 1, atol=0.01), f"quaternion norm {q.min():.4f}..{q.max():.4f}")
        check(np.abs(m["position"]).max() < 500, f"position |x| max {np.abs(m['position']).max():.2f} m")
        check(np.abs(m["linear"]).max() < 10, f"linear velocity max {np.abs(m['linear']).max():.2f} m/s")

    print("\n== geometry_msgs/msg/PoseStamped  /utlidar/robot_pose")
    t, m = session.decode("/utlidar/robot_pose", every=10)
    if m is not None:
        q = np.linalg.norm(m["orientation"], axis=1)
        check(np.allclose(q, 1, atol=0.01), f"quaternion norm {q.min():.4f}..{q.max():.4f}")

    print("\n== sensor_msgs/msg/PointCloud2  /utlidar/cloud_base")
    t, cl = session.clouds("/utlidar/cloud_base", every=400)
    if cl:
        n = [len(c) for c in cl]
        rad = [np.linalg.norm(c, axis=1).max() for c in cl if len(c)]
        check(min(n) > 0, f"{len(cl)} frames, {min(n)}..{max(n)} points each")
        check(max(rad) < 100, f"max point range {max(rad):.1f} m")

    print(f"\n{len(FAILED)} failure(s)")
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
