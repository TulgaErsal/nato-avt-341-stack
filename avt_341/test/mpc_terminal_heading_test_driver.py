#!/usr/bin/env python3
"""
Test driver for the MPC terminal heading feature.

Simulates vehicle motion using the same 3DOF dynamic bicycle model (Pacejka
rigid tire) that the MPC planner uses internally (ThreeDOF_rigid in
mpc_models.jl).  Vehicle parameters are taken directly from mpc_parameters.jl.

The user sets a goal pose (position + heading) using the "2D Goal Pose" tool
in RViz.  The MPC planner steers the vehicle toward the goal and aligns with
the desired heading on arrival.

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
from visualization_msgs.msg import Marker


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


# ---------------------------------------------------------------------------
# Vehicle parameters matching mpc_parameters.jl (MRZR, Pacejka tire model)
# ---------------------------------------------------------------------------
_G    = 9.81
_LA   = 1.54134           # CG to front axle [m]
_L    = 2.71534           # wheelbase [m]
_LB   = _L - _LA          # CG to rear axle [m]
_M    = 1.269e3           # vehicle mass [kg]
_IZZ  = 1.620e3           # yaw moment of inertia [kg·m²]
_H_CG = 0.634             # CG height [m]
_KZX  = 0.5 * _M * _H_CG / _L
_FZ   = _M * _G
_FZF0 = _FZ * _LB / _L / 2.0   # nominal front half-axle normal load [N]
_FZR0 = _FZ * _LA / _L / 2.0   # nominal rear half-axle normal load [N]
_MU   = 0.977706
_P1   = -7.33706
_P2   = 1.11368
_P3   = -1.04179
_EP   = 0.01              # small epsilon to avoid division by zero


def _pacejka_lateral_forces(v, r, ux, sa, ax):
    """Pacejka lateral forces matching ThreeDOF_rigid in mpc_models.jl."""
    fzf = _FZF0 - _KZX * (-v * r + ax)
    fzr = _FZR0 + _KZX * (-v * r + ax)

    x1f = _P1 / _MU * (math.atan2(v + _LA * r, _EP + ux) - sa)
    x1r = _P1 / _MU * math.atan2(v - _LB * r, _EP + ux)

    fyf = 2.0 * _MU * fzf * math.sin(_P2 * math.atan(x1f - _P3 * (x1f - math.atan(x1f))))
    fyr = 2.0 * _MU * fzr * math.sin(_P2 * math.atan(x1r - _P3 * (x1r - math.atan(x1r))))
    return fyf, fyr


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
        self.declare_parameter('sim_rate_hz', 50.0)
        self.declare_parameter('num_obstacles', 0)
        self.declare_parameter('obstacle_min_size_m', 1.0)
        self.declare_parameter('obstacle_max_size_m', 3.0)
        # obstacle_seed < 0 means use a non-deterministic seed
        self.declare_parameter('obstacle_seed', -1)

        # 3DOF dynamic bicycle model state (matches MPC internal state vector)
        #   x, y  : CG position [m]
        #   psi   : yaw angle [rad]
        #   v     : lateral velocity at CG [m/s]
        #   r     : yaw rate [rad/s]
        #   ux    : longitudinal velocity [m/s]
        #   ax    : longitudinal acceleration [m/s²]
        #   sa    : front-wheel steering angle [rad]
        self._x   = self.get_parameter('start_x').value
        self._y   = self.get_parameter('start_y').value
        self._psi = math.radians(self.get_parameter('start_yaw_deg').value)
        self._v   = 0.0
        self._r   = 0.0
        self._ux  = 0.0
        self._ax  = 0.0
        self._sa  = 0.0

        # Generate random obstacles once at startup.
        self._obstacle_cells = self._generate_obstacle_cells()

        # Publishers
        self._odom_pub = self.create_publisher(Odometry, 'avt_341/odometry', 1)
        self._steer_pub = self.create_publisher(Float64, 'avt_341/steering_angle', 1)
        # Obstacle processor subscribes to avt_341/occupancy_grid.
        # Global planner (config_mrzr) subscribes to avt_341/occupancy_grid_low_res.
        # Publish the same empty map on both topics.
        self._grid_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid', 1)
        self._grid_low_res_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid_low_res', 1)
        self._nav_pub = self.create_publisher(Int32, 'avt_341/nav_command_state', 1)
        self._speed_text_pub = self.create_publisher(Marker, 'avt_341/vehicle_speed_text', 1)

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
            f'Vehicle at ({self._x:.1f}, {self._y:.1f}), yaw={math.degrees(self._psi):.1f} deg. '
            f'Set a goal in RViz using the "2D Goal Pose" tool.')

    # ------------------------------------------------------------------
    def _drive_callback(self, msg: AckermannDriveStamped):
        # Treat the MPC speed and steering commands as direct state targets.
        # Longitudinal acceleration is derived from the speed change so that
        # the Pacejka normal-load transfer term (KZX * ax) stays consistent.
        dt = self._dt if self._dt > 0.0 else 1e-3
        self._ax = (msg.drive.speed - self._ux) / dt
        self._ux = msg.drive.speed
        self._sa = msg.drive.steering_angle

    # ------------------------------------------------------------------
    def _sim_step(self):
        """Integrate 3DOF dynamic bicycle model (ThreeDOF_rigid / Pacejka)."""
        dt  = self._dt
        v   = self._v
        r   = self._r
        psi = self._psi
        ux  = self._ux
        ax  = self._ax
        sa  = self._sa

        fyf, fyr = _pacejka_lateral_forces(v, r, ux, sa, ax)

        # State derivatives (from mpc_models.jl ThreeDOF_rigid, CG frame)
        x_dot   = ux * math.cos(psi) - v * math.sin(psi)
        y_dot   = ux * math.sin(psi) + v * math.cos(psi)
        v_dot   = (fyf + fyr) / _M - r * ux
        r_dot   = (_LA * fyf - _LB * fyr) / _IZZ
        psi_dot = r

        # Forward Euler integration
        self._x   += x_dot   * dt
        self._y   += y_dot   * dt
        self._v   += v_dot   * dt
        self._r   += r_dot   * dt
        self._psi += psi_dot * dt

        self._publish_odometry()
        self._publish_steering()
        self._publish_speed_text()

    # ------------------------------------------------------------------
    def _publish_odometry(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(self._psi)
        msg.twist.twist.linear.x  = self._ux
        msg.twist.twist.linear.y  = self._v
        msg.twist.twist.angular.z = self._r
        self._odom_pub.publish(msg)

    def _publish_steering(self):
        msg = Float64()
        msg.data = self._sa
        self._steer_pub.publish(msg)

    def _publish_speed_text(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.ns = 'vehicle_speed'
        msg.id = 0
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.ADD
        msg.pose.position.x = self._x
        msg.pose.position.y = self._y
        msg.pose.position.z = 2.0
        msg.scale.z = 1.5          # text height [m]
        msg.color.r = 1.0
        msg.color.g = 1.0
        msg.color.b = 1.0
        msg.color.a = 1.0
        msg.text = f'{self._ux:.1f} m/s'
        self._speed_text_pub.publish(msg)

    def _generate_obstacle_cells(self):
        """Return a set of (col, row) grid cells occupied by random obstacles.

        Obstacles are square blobs placed randomly within the map.  A keep-out
        radius of 5 m around the vehicle start position is enforced so the
        vehicle is never spawned inside an obstacle.
        """
        import random

        n         = self.get_parameter('num_obstacles').value
        min_size  = self.get_parameter('obstacle_min_size_m').value
        max_size  = self.get_parameter('obstacle_max_size_m').value
        seed      = self.get_parameter('obstacle_seed').value
        width_m   = self.get_parameter('map_width_m').value
        height_m  = self.get_parameter('map_height_m').value
        res       = self.get_parameter('map_resolution_m').value
        cols      = int(width_m  / res)
        rows      = int(height_m / res)

        if n <= 0:
            return set()

        rng = random.Random(seed if seed >= 0 else None)

        start_x = self.get_parameter('start_x').value
        start_y = self.get_parameter('start_y').value
        keepout_sq = 5.0 ** 2   # 5 m keep-out radius around vehicle start

        cells = set()
        placed = 0
        max_attempts = n * 20   # give up after this many misses
        attempts = 0
        while placed < n and attempts < max_attempts:
            attempts += 1
            cx = rng.uniform(0.0, width_m)
            cy = rng.uniform(0.0, height_m)
            # Reject if too close to the vehicle start position.
            dx, dy = cx - start_x, cy - start_y
            if dx * dx + dy * dy < keepout_sq:
                continue
            size = rng.uniform(min_size, max_size)
            half = size / 2.0
            c0 = max(0, int((cx - half) / res))
            c1 = min(cols - 1, int((cx + half) / res))
            r0 = max(0, int((cy - half) / res))
            r1 = min(rows - 1, int((cy + half) / res))
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    cells.add((c, r))
            placed += 1

        self.get_logger().info(
            f'Generated {placed} random obstacle(s) '
            f'(seed={seed if seed >= 0 else "random"}, '
            f'size=[{min_size:.1f}, {max_size:.1f}] m).')
        return cells

    def _publish_grid(self):
        """Publish an occupancy grid covering the test area with random obstacles."""
        width_m  = self.get_parameter('map_width_m').value
        height_m = self.get_parameter('map_height_m').value
        res      = self.get_parameter('map_resolution_m').value
        cols = int(width_m / res)
        rows = int(height_m / res)

        data = [0] * (cols * rows)
        for (c, r) in self._obstacle_cells:
            if 0 <= c < cols and 0 <= r < rows:
                data[r * cols + c] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = res
        msg.info.width  = cols
        msg.info.height = rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.data = data
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
