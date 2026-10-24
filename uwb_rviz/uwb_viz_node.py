#!/usr/bin/env python3
"""Go2 UWB remote -> RViz: turns /uwbstate into markers, a TF frame and a pose.

Usage (after `source ~/unitree_ros2/setup.sh`):
    python3 uwb_viz_node.py [--ros-args -p bearing_field:=yaw_est ...]
Publishes (frame: base_link by default):
    /uwb/markers   visualization_msgs/MarkerArray  robot, tag, range ring, trail, readout
    /uwb/tag_pose  geometry_msgs/PoseStamped        tag position + raw tag attitude
    TF base_link -> uwb_tag

Unitree does not document the angle conventions of UwbState, so which field is the
bearing / elevation (and its sign) is a parameter. Check it live: walk around the
robot with the remote and watch whether the tag sphere follows you.
"""
import math
from collections import deque

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray
from unitree_go.msg import UwbState

ANGLE_FIELDS = ("orientation_est", "pitch_est", "yaw_est",
                "tag_roll", "tag_pitch", "tag_yaw", "none")


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,   # x
            cr * sp * cy + sr * cp * sy,   # y
            cr * cp * sy - sr * sp * cy,   # z
            cr * cp * cy + sr * sp * sy)   # w


def rgba(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


class UwbViz(Node):
    def __init__(self):
        super().__init__("uwb_viz")
        self.frame = self.declare_parameter("frame_id", "base_link").value
        self.bearing_field = self.declare_parameter("bearing_field", "orientation_est").value
        self.bearing_sign = self.declare_parameter("bearing_sign", 1.0).value
        self.bearing_offset = self.declare_parameter("bearing_offset", 0.0).value
        self.elev_field = self.declare_parameter("elevation_field", "pitch_est").value
        self.trail_len = self.declare_parameter("trail_length", 150).value
        self.stale_sec = self.declare_parameter("stale_timeout", 1.0).value
        for f in (self.bearing_field, self.elev_field):
            if f not in ANGLE_FIELDS:
                raise ValueError(f"unknown field '{f}', choose from {ANGLE_FIELDS}")

        self.last = None          # last UwbState
        self.last_t = None        # receive time (node clock)
        self.count = 0
        self.trail = deque(maxlen=self.trail_len)

        self.create_subscription(UwbState, "/uwbstate", self.on_uwb, 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/uwb/markers", 10)
        self.pub_pose = self.create_publisher(PoseStamped, "/uwb/tag_pose", 10)
        self.tf = TransformBroadcaster(self)
        # Redraw at 10 Hz so stale data is visibly greyed out even if /uwbstate stops
        self.create_timer(0.1, self.draw)
        self.get_logger().info(
            f"bearing={self.bearing_sign:+g}*{self.bearing_field}{self.bearing_offset:+g}, "
            f"elevation={self.elev_field}, frame={self.frame}")

    def angle(self, m, field):
        return 0.0 if field == "none" else float(getattr(m, field))

    def tag_xyz(self, m):
        d = float(m.distance_est)
        az = self.bearing_sign * self.angle(m, self.bearing_field) + self.bearing_offset
        el = self.angle(m, self.elev_field)
        return (d * math.cos(el) * math.cos(az),
                d * math.cos(el) * math.sin(az),
                d * math.sin(el))

    def on_uwb(self, m):
        self.last, self.last_t = m, self.get_clock().now()
        self.count += 1
        self.trail.append(self.tag_xyz(m))

    def marker(self, ns, mid, mtype, stamp, color, scale=(1.0, 1.0, 1.0)):
        mk = Marker()
        mk.header.frame_id, mk.header.stamp = self.frame, stamp
        mk.ns, mk.id, mk.type, mk.action = ns, mid, mtype, Marker.ADD
        mk.pose.orientation.w = 1.0
        mk.scale.x, mk.scale.y, mk.scale.z = scale
        mk.color = color
        return mk

    def draw(self):
        now = self.get_clock().now()
        stamp = now.to_msg()
        out = MarkerArray()

        # Robot body (Go2 is roughly 0.7 x 0.31 x 0.4 m) + heading arrow, always shown
        body = self.marker("robot", 0, Marker.CUBE, stamp, rgba(0.6, 0.6, 0.65, 0.8), (0.6, 0.28, 0.15))
        head = self.marker("robot", 1, Marker.ARROW, stamp, rgba(1.0, 0.6, 0.0), (0.05, 0.08, 0.1))
        head.points = [Point(x=0.0), Point(x=0.6)]
        out.markers += [body, head]

        m = self.last
        if m is None:
            txt = self.marker("readout", 0, Marker.TEXT_VIEW_FACING, stamp, rgba(1.0, 1.0, 1.0), (0.0, 0.0, 0.12))
            txt.pose.position.z = 0.6
            txt.text = "waiting for /uwbstate ..."
            out.markers.append(txt)
            self.pub_markers.publish(out)
            return

        age = (now - self.last_t).nanoseconds * 1e-9
        stale = age > self.stale_sec
        if stale:
            state, col = f"STALE {age:.1f}s", rgba(0.5, 0.5, 0.5)
        elif m.error_state:
            state, col = f"ERROR {m.error_state}", rgba(1.0, 0.15, 0.15)
        elif not m.enabled_from_app:
            state, col = "DISABLED in app", rgba(1.0, 0.8, 0.0)
        else:
            state, col = "OK", rgba(0.1, 0.9, 0.3)

        x, y, z = self.tag_xyz(m)
        q = quat_from_rpy(m.tag_roll, m.tag_pitch, m.tag_yaw)

        # Tag pose + TF (orientation is the raw tag_roll/pitch/yaw, frame unverified)
        ps = PoseStamped()
        ps.header.frame_id, ps.header.stamp = self.frame, stamp
        ps.pose.position = Point(x=x, y=y, z=z)
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = q
        self.pub_pose.publish(ps)
        tf = TransformStamped()
        tf.header = ps.header
        tf.child_frame_id = "uwb_tag"
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x, y, z
        tf.transform.rotation = ps.pose.orientation
        self.tf.sendTransform(tf)

        tag = self.marker("tag", 0, Marker.SPHERE, stamp, col, (0.12, 0.12, 0.12))
        tag.pose.position = Point(x=x, y=y, z=z)

        link = self.marker("link", 0, Marker.LINE_LIST, stamp, rgba(col.r, col.g, col.b, 0.6), (0.015, 0.0, 0.0))
        link.points = [Point(), Point(x=x, y=y, z=z)]

        # Ground-plane range ring at the measured horizontal distance
        ring = self.marker("range_ring", 0, Marker.LINE_STRIP, stamp, rgba(0.3, 0.6, 1.0, 0.5), (0.01, 0.0, 0.0))
        r = math.hypot(x, y)
        ring.points = [Point(x=r * math.cos(a * math.pi / 36), y=r * math.sin(a * math.pi / 36))
                       for a in range(73)]

        trail = self.marker("trail", 0, Marker.LINE_STRIP, stamp, rgba(1.0, 0.3, 1.0, 0.7), (0.02, 0.0, 0.0))
        trail.points = [Point(x=px, y=py, z=pz) for px, py, pz in self.trail]

        # Joystick deflection drawn as an arrow from the tag (lx -> robot x, ly -> robot y)
        jx, jy = float(m.joystick[0]), float(m.joystick[1])   # numpy float32 -> float
        joy = self.marker("joystick", 0, Marker.ARROW, stamp, rgba(0.2, 0.9, 1.0), (0.02, 0.05, 0.05))
        joy.points = [Point(x=x, y=y, z=z + 0.15),
                      Point(x=x + 0.4 * jy, y=y - 0.4 * jx, z=z + 0.15)]

        deg = math.degrees
        txt = self.marker("readout", 0, Marker.TEXT_VIEW_FACING, stamp, col, (0.0, 0.0, 0.09))
        txt.pose.position.z = 0.9
        txt.text = "\n".join([
            f"UWB {state}   v{m.version[0]}.{m.version[1]} ch{m.channel} joy_mode {m.joy_mode}  msgs {self.count}",
            f"distance {m.distance_est:.2f} m",
            f"orientation {deg(m.orientation_est):7.1f}  pitch {deg(m.pitch_est):7.1f}  yaw {deg(m.yaw_est):7.1f} deg",
            f"tag  r/p/y {deg(m.tag_roll):6.1f} {deg(m.tag_pitch):6.1f} {deg(m.tag_yaw):6.1f} deg",
            f"base r/p/y {deg(m.base_roll):6.1f} {deg(m.base_pitch):6.1f} {deg(m.base_yaw):6.1f} deg",
            f"joystick {jx:+.2f} {jy:+.2f}   buttons 0b{m.buttons:08b}",
        ])

        out.markers += [tag, link, ring, trail, joy, txt]
        self.pub_markers.publish(out)


def main():
    rclpy.init()
    node = UwbViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
