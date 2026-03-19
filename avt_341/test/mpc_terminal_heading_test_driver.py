#!/usr/bin/env python3
"""
Test driver for the MPC terminal heading feature.

Simulates vehicle motion using a simple kinematic bicycle model driven by
MPC drive commands. The user sets a goal pose (position + heading) using the
"2D Goal Pose" tool in RViz. The MPC planner steers the vehicle toward the
goal and aligns with the desired heading on arrival.

Published topics:
  avt_341/odometry          -- simulated vehicle odometry
  avt_341/steering_angle    -- current steering angle (for veh_converter)
  avt_341/occupancy_grid    -- empty occupancy grid (no obstacles)
  avt_341/nav_command_state -- nav stack active command

Subscribed topics:
  avt_341/drive             -- AckermannDriveStamped commands from MPC
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64, Int32


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


class MPCTerminalHeadingTestDriver(Node):
    def __init__(self):
        super().__init__('mpc_terminal_heading_test_driver')

        # Parameters
        self.declare_parameter('map_width_m', 100.0)
        self.declare_parameter('map_height_m', 100.0)
        self.declare_parameter('map_resolution_m', 1.0)
        self.declare_parameter('start_x', 10.0)
        self.declare_parameter('start_y', 10.0)
        self.declare_parameter('start_yaw_deg', 0.0)
        self.declare_parameter('wheelbase', 2.77)   # MRZR wheelbase [m]
        self.declare_parameter('sim_rate_hz', 50.0)

        # Initial vehicle state
        self._x = self.get_parameter('start_x').value
        self._y = self.get_parameter('start_y').value
        self._yaw = math.radians(self.get_parameter('start_yaw_deg').value)
        self._speed = 0.0
        self._steering = 0.0
        self._wheelbase = self.get_parameter('wheelbase').value

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, 'avt_341/odometry', 1)
        self._steer_pub = self.create_publisher(Float64, 'avt_341/steering_angle', 1)
        # Obstacle processor subscribes to avt_341/occupancy_grid.
        # Global planner (config_mrzr) subscribes to avt_341/occupancy_grid_low_res.
        # Publish the same empty map on both topics.
        self._grid_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid', 1)
        self._grid_low_res_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid_low_res', 1)
        self._nav_pub = self.create_publisher(Int32, 'avt_341/nav_command_state', 1)

        # Subscriber
        self._drive_sub = self.create_subscription(
            AckermannDriveStamped, 'avt_341/drive', self._drive_callback, 1)

        # Timers
        sim_rate = self.get_parameter('sim_rate_hz').value
        self._dt = 1.0 / sim_rate
        self._sim_timer = self.create_timer(self._dt, self._sim_step)
        # Publish occupancy grid and nav state at 1 Hz
        self._grid_timer = self.create_timer(1.0, self._publish_grid)
        self._nav_timer = self.create_timer(1.0, self._publish_nav_state)

        self.get_logger().info(
            f'MPC terminal heading test driver started. '
            f'Vehicle at ({self._x:.1f}, {self._y:.1f}), yaw={math.degrees(self._yaw):.1f} deg. '
            f'Set a goal in RViz using the "2D Goal Pose" tool.')

    # ------------------------------------------------------------------
    def _drive_callback(self, msg: AckermannDriveStamped):
        self._speed = msg.drive.speed
        self._steering = msg.drive.steering_angle

    # ------------------------------------------------------------------
    def _sim_step(self):
        """Integrate kinematic bicycle model."""
        dt = self._dt
        v = self._speed
        sa = self._steering
        L = self._wheelbase

        self._x += v * math.cos(self._yaw) * dt
        self._y += v * math.sin(self._yaw) * dt
        if abs(L) > 1e-6:
            self._yaw += v * math.tan(sa) / L * dt

        self._publish_odometry()
        self._publish_steering()

    # ------------------------------------------------------------------
    def _publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self._yaw)
        msg.twist.twist.linear.x = self._speed
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.angular.z = (
            self._speed * math.tan(self._steering) / self._wheelbase
            if abs(self._wheelbase) > 1e-6 else 0.0
        )
        self._odom_pub.publish(msg)

    def _publish_steering(self):
        msg = Float64()
        msg.data = self._steering
        self._steer_pub.publish(msg)

    def _publish_grid(self):
        """Publish an empty occupancy grid covering the test area."""
        width_m = self.get_parameter('map_width_m').value
        height_m = self.get_parameter('map_height_m').value
        res = self.get_parameter('map_resolution_m').value
        cols = int(width_m / res)
        rows = int(height_m / res)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = res
        msg.info.width = cols
        msg.info.height = rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.data = [0] * (cols * rows)
        self._grid_pub.publish(msg)
        self._grid_low_res_pub.publish(msg)

    def _publish_nav_state(self):
        """Publish nav_command_state = 0 (Active) to keep the stack running."""
        msg = Int32()
        msg.data = 0
        self._nav_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MPCTerminalHeadingTestDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
