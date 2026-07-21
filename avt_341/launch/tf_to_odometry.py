#!/usr/bin/env python3
"""Publishes odometry derived from a TF lookup (parent_frame -> child_frame)."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros


class TfToOdometry(Node):
    def __init__(self):
        super().__init__('tf_to_odometry')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'mrzr/base_link')
        self.declare_parameter('output_topic', '/mrzr/avt_341/odometry')
        self.declare_parameter('rate_hz', 10.0)

        self._parent = self.get_parameter('parent_frame').get_parameter_value().string_value
        self._child = self.get_parameter('child_frame').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value

        self._buf = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._buf, self)
        self._pub = self.create_publisher(Odometry, output_topic, 10)
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        try:
            # Time(0) means "latest available", works with both wall and sim time.
            t = self._buf.lookup_transform(self._parent, self._child, rclpy.time.Time())
        except Exception:
            return

        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = self._parent
        odom.child_frame_id = self._child
        odom.pose.pose.position.x = t.transform.translation.x
        odom.pose.pose.position.y = t.transform.translation.y
        odom.pose.pose.position.z = t.transform.translation.z
        odom.pose.pose.orientation = t.transform.rotation
        self._pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(TfToOdometry())


if __name__ == '__main__':
    main()
