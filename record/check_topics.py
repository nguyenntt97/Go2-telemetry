#!/usr/bin/env python3
"""Listen to key Go2 topics for a few seconds and report which are publishing.

Usage: python3 check_topics.py [seconds (default 3)]
Prints one line per topic (`<topic> <count>`) and `slam_active <true|false>`.
Exit code 1 if any key topic got no messages.
"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from unitree_go.msg import LowState, SportModeState, UwbState

KEY = {
    "/sportmodestate": SportModeState,
    "/lf/lowstate": LowState,
    "/uwbstate": UwbState,
    "/utlidar/robot_odom": Odometry,
}
SLAM = {
    "/uslam/frontend/odom": Odometry,
    "/uslam/localization/odom": Odometry,
    "/lio_sam_ros2/mapping/odometry": Odometry,
}


def main():
    sec = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = rclpy.create_node("go2_topic_check")
    counts = {t: 0 for t in {**KEY, **SLAM}}

    def counter(topic):
        def cb(_):
            counts[topic] += 1
        return cb

    for topic, mtype in {**KEY, **SLAM}.items():
        node.create_subscription(mtype, topic, counter(topic), 10)
        # Odometry publishers may be best-effort; a second subscription catches those
        if mtype is Odometry:
            node.create_subscription(mtype, topic, counter(topic), qos_profile_sensor_data)

    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()

    for topic in KEY:
        print(f"{topic} {counts[topic]}")
    for topic in SLAM:
        print(f"{topic} {counts[topic]}")
    print(f"slam_active {'true' if any(counts[t] for t in SLAM) else 'false'}")
    sys.exit(0 if all(counts[t] for t in KEY) else 1)


if __name__ == "__main__":
    main()
