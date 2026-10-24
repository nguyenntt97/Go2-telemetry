#!/usr/bin/env python3
"""Message layouts for the topics this robot records.

The `.msg` files live in `unitree_ros2` on the Jetson and are not installed here, so
the layouts are written out by hand. Each one is checked against the real message
size by `cdr.compile`, which refuses a schema that does not account for every byte;
`selftest.py` additionally checks the decoded values are physically sensible.

Field order and types must match the `.msg` exactly -- CDR carries no field names.
"""
import cdr

# --- nested types -------------------------------------------------------------

STRUCTS = {
    "TimeSpec": [("sec", "i32"), ("nanosec", "u32")],

    # 52 B. The `int8 temperature` this message is usually documented with is not
    # here: with it, LowState's motor block would start 4 B later and the battery
    # would not land where it measurably does (see the note on LowState).
    #
    # Unresolved: the byte just past this block reads 79-80 and drifts slowly in both
    # LowState and SportModeState, which looks like an IMU temperature in degrees C
    # rather than the `mode` the .msg puts there. The battery anchor rules out moving
    # the motor array to suit it, so the byte is left where the arithmetic puts it and
    # neither message's `mode` is trusted (nothing below plots it).
    "IMUState": [
        ("quaternion", ("arr", "f32", 4)),
        ("gyroscope", ("arr", "f32", 3)),
        ("accelerometer", ("arr", "f32", 3)),
        ("rpy", ("arr", "f32", 3)),
    ],

    "MotorState": [
        ("mode", "u8"),
        ("q", "f32"), ("dq", "f32"), ("ddq", "f32"), ("tau_est", "f32"),
        ("q_raw", "f32"), ("dq_raw", "f32"), ("ddq_raw", "f32"),
        ("temperature", "i8"),
        ("lost", "u32"),
        ("reserve", ("arr", "u32", 2)),
    ],

    "BmsState": [
        ("version_high", "u8"), ("version_low", "u8"),
        ("status", "u8"), ("soc", "u8"),
        ("current", "i32"),
        ("cycle", "u16"),
        ("bq_ntc", ("arr", "i8", 2)),
        ("mcu_ntc", ("arr", "i8", 2)),
        ("cell_vol", ("arr", "u16", 15)),
    ],

    "PathPoint": [
        ("t_from_start", "f32"), ("x", "f32"), ("y", "f32"), ("yaw", "f32"),
        ("vx", "f32"), ("vy", "f32"), ("vyaw", "f32"),
    ],
}

# --- whole messages -----------------------------------------------------------
# Sizes in the comments are the message body (total length minus the 4-byte
# encapsulation header), as measured in the 2026-10-23 sessions.

MESSAGES = {
    # 512 B
    "unitree_go/msg/SportModeState": [
        ("stamp", "TimeSpec"),
        ("error_code", "u32"),
        ("imu_state", "IMUState"),
        ("mode", "u8"),
        ("progress", "f32"),
        ("gait_type", "u8"),
        ("foot_raise_height", "f32"),
        ("position", ("arr", "f32", 3)),
        ("body_height", "f32"),
        ("velocity", ("arr", "f32", 3)),
        ("yaw_speed", "f32"),
        ("range_obstacle", ("arr", "f32", 4)),
        ("foot_force", ("arr", "i16", 4)),
        ("foot_position_body", ("arr", "f32", 12)),
        ("foot_speed_body", ("arr", "f32", 12)),
        # 10 points of the planned path, each (t_from_start, x, y, yaw, vx, vy, vyaw)
        ("path_point", ("arr", "PathPoint", 10)),
    ],

    # 1176 B exactly. The layout after `imu_state` was pinned against the recorded
    # bytes rather than taken on trust: `bms_state.cell_vol` is measurably at 1050
    # (the only offset holding 8 plausible cell voltages) and `power_v` at 1152, which
    # puts `bms_state` at 1036 and so `motor_state` at 76..1036 (20 x 48 B). That is
    # confirmed twice over -- the 8 joints a quadruped does not use, 12..19, are
    # exactly zero from 652, and reading joints 0..11 there gives a standing pose
    # (hip ~0, thigh ~0.8, calf ~-1.5 rad) with sane motor temperatures.
    "unitree_go/msg/LowState": [
        ("head", ("arr", "u8", 2)),
        ("level_flag", "u8"),
        ("frame_reserve", "u8"),
        ("sn", ("arr", "u32", 2)),
        ("version", ("arr", "u32", 2)),
        ("bandwidth", "u16"),
        ("imu_state", "IMUState"),
        ("motor_state", ("arr", "MotorState", 20)),
        ("bms_state", "BmsState"),
        ("foot_force", ("arr", "i16", 4)),
        ("foot_force_est", ("arr", "i16", 4)),
        ("tick", "u32"),
        ("wireless_remote", ("arr", "u8", 40)),
        ("bit_flag", "u8"),
        ("adc_reel", "f32"),
        ("temperature_ntc1", "i8"),
        ("temperature_ntc2", "i8"),
        ("power_v", "f32"),
        ("power_a", "f32"),
        ("fan_frequency", ("arr", "u16", 4)),
        ("reserve", "u32"),
        ("crc", "u32"),
    ],

    # 56 B
    "unitree_go/msg/UwbState": [
        ("version", ("arr", "u8", 2)),
        ("channel", "u8"),
        ("joy_mode", "u8"),
        ("orientation_est", "f32"),
        ("pitch_est", "f32"),
        ("distance_est", "f32"),
        ("yaw_est", "f32"),
        ("tag_roll", "f32"), ("tag_pitch", "f32"), ("tag_yaw", "f32"),
        ("base_roll", "f32"), ("base_pitch", "f32"), ("base_yaw", "f32"),
        ("joystick", ("arr", "f32", 2)),
        ("error_state", "u8"),
        ("buttons", "u8"),
        ("enabled_from_app", "u8"),
    ],

    # 320 B
    "sensor_msgs/msg/Imu": [
        ("header", "hdr"),
        ("orientation", ("arr", "f64", 4)),
        ("orientation_covariance", ("arr", "f64", 9)),
        ("angular_velocity", ("arr", "f64", 3)),
        ("angular_velocity_covariance", ("arr", "f64", 9)),
        ("linear_acceleration", ("arr", "f64", 3)),
        ("linear_acceleration_covariance", ("arr", "f64", 9)),
    ],

    # 720 B
    "nav_msgs/msg/Odometry": [
        ("header", "hdr"),
        ("child_frame_id", "str"),
        ("position", ("arr", "f64", 3)),
        ("orientation", ("arr", "f64", 4)),
        ("pose_covariance", ("arr", "f64", 36)),
        ("linear", ("arr", "f64", 3)),
        ("angular", ("arr", "f64", 3)),
        ("twist_covariance", ("arr", "f64", 36)),
    ],

    # 80 B
    "geometry_msgs/msg/PoseStamped": [
        ("header", "hdr"),
        ("position", ("arr", "f64", 3)),
        ("orientation", ("arr", "f64", 4)),
    ],

    # 36 B
    "geometry_msgs/msg/PointStamped": [
        ("header", "hdr"),
        ("point", ("arr", "f64", 3)),
    ],
}

cdr.SCHEMA_STRUCTS = STRUCTS
