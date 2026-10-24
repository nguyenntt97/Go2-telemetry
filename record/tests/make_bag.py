"""Build synthetic rosbag2 sqlite3 bags, so the exporter can be tested here.

The real sessions are made on the Jetson. These bags use the same schema and
the same CDR encoding, which is enough to exercise everything but the
agreement between the local `.msg` files and what the robot actually sends.
"""

from __future__ import annotations

import os
import sqlite3

from go2export.cdr import CdrWriter
from go2export.decode import encode_message

_SCHEMA = """
CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);
CREATE TABLE topics(
  id INTEGER PRIMARY KEY,
  name TEXT NOT NULL,
  type TEXT NOT NULL,
  serialization_format TEXT NOT NULL,
  offered_qos_profiles TEXT NOT NULL);
CREATE TABLE messages(
  id INTEGER PRIMARY KEY,
  topic_id INTEGER NOT NULL,
  timestamp INTEGER NOT NULL,
  data BLOB NOT NULL);
CREATE INDEX timestamp_idx ON messages(timestamp ASC);
"""


class BagBuilder:
    """One `.db3` file. A session that splits has several of these."""

    def __init__(self, path: str) -> None:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.path = path
        self._conn = sqlite3.connect(path)
        self._conn.executescript(_SCHEMA)
        self._conn.execute(
            "INSERT INTO schema VALUES (?, ?)", (3, "humble")
        )
        self._topic_ids: dict[str, int] = {}
        self._next_message_id = 1

    def topic(self, name: str, type_name: str) -> int:
        if name not in self._topic_ids:
            topic_id = len(self._topic_ids) + 1
            self._conn.execute(
                "INSERT INTO topics VALUES (?, ?, ?, ?, ?)",
                (topic_id, name, type_name, "cdr", ""),
            )
            self._topic_ids[name] = topic_id
        return self._topic_ids[name]

    def add_raw(self, name: str, type_name: str, timestamp_ns: int, blob: bytes) -> None:
        topic_id = self.topic(name, type_name)
        self._conn.execute(
            "INSERT INTO messages VALUES (?, ?, ?, ?)",
            (self._next_message_id, topic_id, timestamp_ns, sqlite3.Binary(blob)),
        )
        self._next_message_id += 1

    def add(self, name, type_name, timestamp_ns, data, registry) -> None:
        self.add_raw(
            name, type_name, timestamp_ns, encode_message(data, type_name, registry)
        )

    def close(self) -> None:
        self._conn.commit()
        self._conn.close()


# --- message makers ----------------------------------------------------------


def sport_mode_state(index: int) -> dict:
    return {
        "stamp": {"sec": 1700000000 + index, "nanosec": 0},
        "error_code": 0,
        "imu_state": imu_state(index),
        "mode": 1,
        "progress": 0.0,
        "gait_type": 1,
        "foot_raise_height": 0.09,
        "position": [0.1 * index, 0.0, 0.32],
        "body_height": 0.32,
        "velocity": [0.5, 0.0, 0.0],
        "yaw_speed": 0.01 * index,
        "range_obstacle": [1.0, 2.0, 3.0, 4.0],
        "foot_force": [10, 11, 12, 13],
        "foot_position_body": [float(i) for i in range(12)],
        "foot_speed_body": [0.0] * 12,
    }


def imu_state(index: int) -> dict:
    return {
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "gyroscope": [0.01, 0.02, 0.03],
        "accelerometer": [0.0, 0.0, 9.81],
        "rpy": [0.0, 0.0, 0.001 * index],
        "temperature": 42,
    }


def low_state(index: int) -> dict:
    return {
        "head": b"\xfe\xef",
        "level_flag": 0,
        "frame_reserve": 0,
        "sn": [1, 2],
        "version": [1, 0],
        "bandwidth": 100,
        "imu_state": imu_state(index),
        "motor_state": [
            {
                "mode": 1,
                "q": 0.1 * motor,
                "dq": 0.0,
                "ddq": 0.0,
                "tau_est": 1.5,
                "temperature": 30 + motor,
                "lost": 0,
            }
            for motor in range(20)
        ],
        "bms_state": {
            "version_high": 1,
            "version_low": 2,
            "status": 0,
            "soc": 88,
            "current": -1200,
            "cycle": 37,
            "bq_ntc": [25, 26],
            "mcu_ntc": [30, 31],
            "cell_vol": [4100 + i for i in range(15)],
        },
        "foot_force": [20, 21, 22, 23],
        "foot_force_est": [0, 0, 0, 0],
        "tick": 1000 + index,
        "wireless_remote": bytes(40),
        "bit_flag": 0,
        "adc_reel": 0.0,
        "temperature_ntc1": 40,
        "temperature_ntc2": 41,
        "power_v": 29.5,
        "power_a": 1.2,
        "crc": 0,
    }


def uwb_state(index: int) -> dict:
    return {
        "version": b"\x01\x00",
        "channel": 5,
        "joy_mode": 0,
        "orientation_est": 0.1 * index,
        "pitch_est": 0.02,
        "distance_est": 2.0,
        "yaw_est": -0.1 * index,
        "tag_roll": 0.0,
        "tag_pitch": 0.0,
        "tag_yaw": 0.5,
        "base_roll": 0.0,
        "base_pitch": 0.0,
        "base_yaw": 0.25,
        "joystick": [0.0, 0.0],
        "error_state": 0,
        "buttons": 0,
        "enabled_from_app": 1,
    }


def odometry(index: int) -> dict:
    return {
        "header": {"stamp": {"sec": 1700000000 + index, "nanosec": 0}, "frame_id": "odom"},
        "child_frame_id": "base_link",
        "pose": {
            "pose": {
                "position": {"x": 0.2 * index, "y": 0.1, "z": 0.32},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
            "covariance": [0.0] * 36,
        },
        "twist": {
            "twist": {
                "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.05},
            },
            "covariance": [0.0] * 36,
        },
    }


def imu_message(index: int) -> dict:
    return {
        "header": {"stamp": {"sec": 1700000000 + index, "nanosec": 0}, "frame_id": "utlidar_imu"},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "orientation_covariance": [0.0] * 9,
        "angular_velocity": {"x": 0.01, "y": 0.02, "z": 0.03},
        "angular_velocity_covariance": [0.0] * 9,
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81},
        "linear_acceleration_covariance": [0.0] * 9,
    }


def wireless_controller(index: int) -> dict:
    return {"lx": 0.1 * index, "ly": 0.0, "rx": -0.2, "ry": 0.0, "keys": 2}


def point_cloud(points: int = 4) -> dict:
    import struct

    payload = b"".join(
        struct.pack("<ffff", float(i), float(i) + 0.5, 0.3, float(i) * 10)
        for i in range(points)
    )
    return {
        "header": {"stamp": {"sec": 1700000000, "nanosec": 0}, "frame_id": "utlidar"},
        "height": 1,
        "width": points,
        "fields": [
            {"name": "x", "offset": 0, "datatype": 7, "count": 1},
            {"name": "y", "offset": 4, "datatype": 7, "count": 1},
            {"name": "z", "offset": 8, "datatype": 7, "count": 1},
            {"name": "intensity", "offset": 12, "datatype": 7, "count": 1},
        ],
        "is_bigendian": False,
        "point_step": 16,
        "row_step": 16 * points,
        "data": payload,
        "is_dense": True,
    }


# --- the video topic, which has no usable message definition ----------------


def nal(nal_type: int, body: bytes = b"\x00" * 8) -> bytes:
    """One Annex-B NAL unit. 7 is SPS, 5 an IDR slice, 1 a normal slice."""
    return b"\x00\x00\x00\x01" + bytes([0x60 | nal_type]) + body


def video_message(time_frame: int, resolution: int, payload: bytes) -> bytes:
    """Encode the layout measured on the robot, which the local
    `Go2FrontVideoData` definition does not match."""
    writer = CdrWriter()
    writer.primitive("uint64", time_frame)
    writer.primitive("uint32", resolution)
    writer.uint32(len(payload))
    writer.block(payload)
    return writer.getvalue()
