#!/usr/bin/env python3
"""
Test driver for the MPC adaptive prediction-horizon ("final time as a design
variable") feature.

Simulates a single vehicle using the same 3DOF dynamic bicycle model (Pacejka
rigid tire) that the MPC planner uses internally (ThreeDOF_rigid in
mpc_models.jl, parameters from mpc_parameters.jl) -- identical to the follower
model in formation_distance_test_driver.py.  The vehicle drives autonomously
from a fixed start pose to a fixed goal pose through a randomly generated
obstacle field, using the full local-planning stack (global path, goal point
processor, obstacle processor, MPC planner).

This driver replaces the perception stack: it publishes a synthetic occupancy
grid containing the random obstacles, publishes the vehicle's simulated
odometry, and sends the one-time goal_pose / nav_command_state messages
needed to make the global planner and goal point processor go active.  It
does not require any interactive input, so it is suitable for a
launch-and-record scenario over SSH.

Obstacle field tuning knobs (see launch file for the corresponding arguments):
  random_seed          RNG seed for obstacle placement (-1 = non-deterministic)
  num_obstacles        Number of randomly placed square obstacles
  obstacle_min_size_m  Minimum obstacle side length [m]
  obstacle_max_size_m  Maximum obstacle side length [m]
  mpc_max_speed        Maximum speed handed to the MPC planner [m/s]

Scenario end / reporting
-------------------------
The scenario ends when the global planner reports the goal has been reached
(avt_341/goal_reached), or after scenario_timeout_s elapses, whichever comes
first.  A short settle period is added after arrival so the recording captures
the vehicle coming to rest.  On completion the driver logs MPC solve-time
statistics (mean/std) and the percentage of solves that returned an optimal
status, then shuts itself down -- which the launch file uses as the signal to
stop screen recording and tear down the rest of the scenario.

Published topics:
  avt_341/odometry              -- simulated vehicle odometry
  avt_341/steering_angle        -- current steering angle (for veh_converter)
  avt_341/occupancy_grid        -- occupancy grid with random obstacles
  avt_341/occupancy_grid_low_res -- same grid (for the global planner)
  avt_341/nav_command_state     -- NavStateCmd::GoActive (1), sent once
  avt_341/goal_pose             -- one-time goal for the global planner
  avt_341/speed_setpoint        -- mpc_max_speed [m/s]
  avt_341/vehicle_speed_text    -- Marker: current speed label

Subscribed topics:
  avt_341/drive                 -- AckermannDriveStamped commands from MPC
  avt_341/goal_reached           -- NavState published once the goal is reached
  avt_341/mpc_solve_diagnostics -- [status_code, solve_time_ms, effective_tf]
"""

import math
import random
import statistics

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Quaternion, PoseStamped, TransformStamped
from std_msgs.msg import Float64, Int32, Float64MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from avt_341_msgs.msg import NavState


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


# ---------------------------------------------------------------------------
# MRZR vehicle parameters matching mpc_parameters.jl (Pacejka rigid tires).
# Identical to formation_distance_test_driver.py's follower model.
# ---------------------------------------------------------------------------
_LA   = 1.54134           # CG to front axle [m]
_L    = 2.71534           # wheelbase [m]
_LB   = _L - _LA          # CG to rear axle [m]
_M    = 1.269e3           # vehicle mass [kg]
_IZZ  = 1.620e3           # yaw moment of inertia [kg*m^2]
_FZ   = _M * 9.81
_FZF0 = _FZ * _LB / _L / 2.0   # nominal front half-axle normal load [N]
_FZR0 = _FZ * _LA / _L / 2.0   # nominal rear half-axle normal load [N]
_MU   = 0.977706
_P1   = -7.33706
_P2   = 1.11368
_P3   = -1.04179
_EP   = 0.01              # small epsilon to avoid division by zero


def _pacejka_lateral_forces(v: float, r: float, ux: float, sa: float) -> tuple:
    """Pacejka lateral forces matching ThreeDOF_rigid in mpc_models.jl.

    ax is assumed zero because speed is commanded directly from the MPC
    output, bypassing the powertrain.  The static normal loads FZF0 / FZR0
    apply.
    """
    x1f = _P1 / _MU * (math.atan2(v + _LA * r, _EP + ux) - sa)
    x1r = _P1 / _MU * math.atan2(v - _LB * r, _EP + ux)

    fyf = 2.0 * _MU * _FZF0 * math.sin(
        _P2 * math.atan(x1f - _P3 * (x1f - math.atan(x1f))))
    fyr = 2.0 * _MU * _FZR0 * math.sin(
        _P2 * math.atan(x1r - _P3 * (x1r - math.atan(x1r))))
    return fyf, fyr


class FinalTimeMPCTestDriver(Node):
    def __init__(self):
        super().__init__('final_time_mpc_test_driver')

        # -- Parameters ---------------------------------------------------
        self.declare_parameter('map_width_m', 120.0)
        self.declare_parameter('map_height_m', 120.0)
        self.declare_parameter('map_resolution_m', 1.0)
        self.declare_parameter('start_x', 10.0)
        self.declare_parameter('start_y', 10.0)
        self.declare_parameter('start_yaw_deg', 45.0)
        self.declare_parameter('goal_x', 110.0)
        self.declare_parameter('goal_y', 110.0)
        self.declare_parameter('obstacle_keepout_m', 6.0)
        self.declare_parameter('num_obstacles', 10)
        self.declare_parameter('obstacle_min_size_m', 3.0)
        self.declare_parameter('obstacle_max_size_m', 7.0)
        self.declare_parameter('random_seed', 42)          # < 0 => non-deterministic
        self.declare_parameter('mpc_max_speed', 4.0)
        self.declare_parameter('sim_rate_hz', 50.0)
        self.declare_parameter('physics_dt', 0.001)
        self.declare_parameter('scenario_timeout_s', 200.0)
        self.declare_parameter('post_arrival_settle_s', 3.0)

        self._start_x   = float(self.get_parameter('start_x').value)
        self._start_y   = float(self.get_parameter('start_y').value)
        self._goal_x    = float(self.get_parameter('goal_x').value)
        self._goal_y    = float(self.get_parameter('goal_y').value)
        self._mpc_max_speed = float(self.get_parameter('mpc_max_speed').value)
        self._physics_dt     = float(self.get_parameter('physics_dt').value)
        self._timeout_s       = float(self.get_parameter('scenario_timeout_s').value)
        self._settle_s         = float(self.get_parameter('post_arrival_settle_s').value)
        sim_rate               = float(self.get_parameter('sim_rate_hz').value)
        self._pub_dt     = 1.0 / sim_rate
        self._n_substeps = max(1, int(round(self._pub_dt / self._physics_dt)))

        # -- Vehicle 3DOF state (ThreeDOF_rigid, CG frame) -----------------
        self._x   = self._start_x
        self._y   = self._start_y
        self._psi = math.radians(self.get_parameter('start_yaw_deg').value)
        self._v   = 0.0   # lateral velocity [m/s]
        self._r   = 0.0   # yaw rate [rad/s]
        self._ux  = 0.0   # longitudinal speed [m/s] -- set by MPC drive command
        self._sa  = 0.0   # steering angle [rad]     -- set by MPC drive command
        self._t   = 0.0   # simulation time [s]

        # -- Random obstacle field -----------------------------------------
        self._obstacle_cells = self._generate_obstacle_cells()

        # -- Scenario end-state --------------------------------------------
        self._goal_reached  = False
        self._arrival_t      = None
        self._finished        = False

        # -- Solve diagnostics accumulators ---------------------------------
        self._solve_time_ms   = []
        self._solve_optimal    = []
        self._effective_tf     = []

        # -- TF broadcaster --------------------------------------------
        self._tf_br = TransformBroadcaster(self)

        # -- Publishers ------------------------------------------------
        self._odom_pub   = self.create_publisher(Odometry, 'avt_341/odometry', 1)
        self._steer_pub  = self.create_publisher(Float64, 'avt_341/steering_angle', 1)
        self._grid_pub   = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid', 1)
        self._grid_lr_pub = self.create_publisher(OccupancyGrid, 'avt_341/occupancy_grid_low_res', 1)
        self._nav_pub     = self.create_publisher(Int32, 'avt_341/nav_command_state', 1)
        self._goal_pub    = self.create_publisher(PoseStamped, 'avt_341/goal_pose', 1)
        self._speed_pub   = self.create_publisher(Float64, 'avt_341/speed_setpoint', 1)
        self._speed_txt_pub = self.create_publisher(Marker, 'avt_341/vehicle_speed_text', 1)

        # -- Subscribers -------------------------------------------------
        self._drive_sub = self.create_subscription(
            AckermannDriveStamped, 'avt_341/drive', self._drive_cb, 1)
        self._goal_reached_sub = self.create_subscription(
            NavState, 'avt_341/goal_reached', self._goal_reached_cb, 1)
        self._diag_sub = self.create_subscription(
            Float64MultiArray, 'avt_341/mpc_solve_diagnostics', self._diag_cb, 10)

        # -- Timers ----------------------------------------------------
        self._sim_timer     = self.create_timer(self._pub_dt, self._sim_step)
        self._grid_timer    = self.create_timer(1.0, self._publish_grid)
        self._slow_timer    = self.create_timer(0.2, self._publish_slow)
        self._monitor_timer = self.create_timer(0.5, self._monitor_step)

        # One-shot activation: give the other nodes a moment to come up
        # before sending the goal, so the global planner is subscribed.
        self._activated = False
        self._activate_timer = self.create_timer(2.0, self._activate_once)

        self.get_logger().info(
            f'Final-time MPC test driver started. '
            f'Start=({self._start_x:.1f}, {self._start_y:.1f}) '
            f'Goal=({self._goal_x:.1f}, {self._goal_y:.1f}) '
            f'mpc_max_speed={self._mpc_max_speed:.1f} m/s, '
            f'timeout={self._timeout_s:.0f} s.')

    # ------------------------------------------------------------------
    # One-time activation of the global planner / goal point processor
    # ------------------------------------------------------------------
    def _activate_once(self):
        if self._activated:
            return
        self._activated = True
        self._activate_timer.cancel()

        nav = Int32()
        nav.data = 1   # NavStateCmd::GoActive
        self._nav_pub.publish(nav)

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = self._goal_x
        goal.pose.position.y = self._goal_y
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)

        self.get_logger().info('Scenario activated: goal published, nav stack going active.')

    # ------------------------------------------------------------------
    # MPC drive command callback
    # ------------------------------------------------------------------
    def _drive_cb(self, msg: AckermannDriveStamped):
        self._ux = msg.drive.speed
        self._sa = msg.drive.steering_angle

    def _goal_reached_cb(self, msg: NavState):
        if not self._goal_reached:
            self._goal_reached = True
            self._arrival_t = self._t
            self.get_logger().info(f'Goal reached at t={self._t:.1f} s.')

    def _diag_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 3:
            return
        status_code, solve_time_ms, effective_tf = msg.data[0], msg.data[1], msg.data[2]
        self._solve_optimal.append(1 if status_code >= 0.5 else 0)
        self._solve_time_ms.append(solve_time_ms)
        self._effective_tf.append(effective_tf)

    # ------------------------------------------------------------------
    # Physics integration (1 ms sub-steps, matches formation_distance_test_driver.py)
    # ------------------------------------------------------------------
    def _step_vehicle(self, dt: float):
        v, r, psi = self._v, self._r, self._psi
        ux, sa    = self._ux, self._sa

        fyf, fyr = _pacejka_lateral_forces(v, r, ux, sa)

        self._x   += (ux * math.cos(psi) - v * math.sin(psi)) * dt
        self._y   += (ux * math.sin(psi) + v * math.cos(psi)) * dt
        self._v   += ((fyf + fyr) / _M - r * ux) * dt
        self._r   += ((_LA * fyf - _LB * fyr) / _IZZ) * dt
        self._psi += r * dt

    def _sim_step(self):
        dt = self._physics_dt
        for _ in range(self._n_substeps):
            self._step_vehicle(dt)
            self._t += dt

        now = self.get_clock().now().to_msg()
        self._publish_odometry(now)
        self._publish_tf(now)
        self._publish_steering()
        self._publish_speed_text(now)

    # ------------------------------------------------------------------
    # Per-tick publishers
    # ------------------------------------------------------------------
    def _publish_odometry(self, stamp):
        msg = Odometry()
        msg.header.stamp    = stamp
        msg.header.frame_id = 'odom'
        msg.child_frame_id  = 'base_link'
        msg.pose.pose.position.x  = self._x
        msg.pose.pose.position.y  = self._y
        msg.pose.pose.orientation = yaw_to_quaternion(self._psi)
        msg.twist.twist.linear.x  = self._ux
        msg.twist.twist.linear.y  = self._v
        msg.twist.twist.angular.z = self._r
        self._odom_pub.publish(msg)

    def _publish_tf(self, stamp):
        t = TransformStamped()
        t.header.stamp    = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation      = yaw_to_quaternion(self._psi)
        self._tf_br.sendTransform(t)

    def _publish_steering(self):
        msg = Float64()
        msg.data = self._sa
        self._steer_pub.publish(msg)

    def _publish_speed_text(self, stamp):
        m = Marker()
        m.header.stamp    = stamp
        m.header.frame_id = 'map'
        m.ns   = 'vehicle_speed'
        m.id   = 0
        m.type   = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = self._x
        m.pose.position.y = self._y
        m.pose.position.z = 2.0
        m.scale.z = 1.5
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = f'{self._ux:.1f} m/s'
        self._speed_txt_pub.publish(m)

    # ------------------------------------------------------------------
    # Slow / periodic publishers
    # ------------------------------------------------------------------
    def _publish_slow(self):
        sp = Float64()
        sp.data = self._mpc_max_speed
        self._speed_pub.publish(sp)

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
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = res
        msg.info.width  = cols
        msg.info.height = rows
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.data = data
        self._grid_pub.publish(msg)
        self._grid_lr_pub.publish(msg)

    # ------------------------------------------------------------------
    # Random obstacle field generation
    # ------------------------------------------------------------------
    def _generate_obstacle_cells(self):
        """Return a set of (col, row) grid cells occupied by random obstacles.

        Obstacles are square blobs placed randomly within the map.  A
        keep-out radius around both the start and goal positions is enforced
        so neither is ever spawned inside an obstacle or fully boxed in.
        """
        n         = self.get_parameter('num_obstacles').value
        min_size  = self.get_parameter('obstacle_min_size_m').value
        max_size  = self.get_parameter('obstacle_max_size_m').value
        seed      = self.get_parameter('random_seed').value
        width_m   = self.get_parameter('map_width_m').value
        height_m  = self.get_parameter('map_height_m').value
        res       = self.get_parameter('map_resolution_m').value
        keepout   = self.get_parameter('obstacle_keepout_m').value
        cols      = int(width_m  / res)
        rows      = int(height_m / res)

        if n <= 0:
            return set()

        rng = random.Random(seed if seed >= 0 else None)
        keepout_sq = keepout ** 2

        cells = set()
        placed = 0
        max_attempts = n * 40   # give up after this many misses
        attempts = 0
        while placed < n and attempts < max_attempts:
            attempts += 1
            cx = rng.uniform(0.0, width_m)
            cy = rng.uniform(0.0, height_m)
            for (px, py) in ((self._start_x, self._start_y), (self._goal_x, self._goal_y)):
                dx, dy = cx - px, cy - py
                if dx * dx + dy * dy < keepout_sq:
                    break
            else:
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

    # ------------------------------------------------------------------
    # Scenario completion monitor
    # ------------------------------------------------------------------
    def _monitor_step(self):
        if self._finished:
            return
        if self._goal_reached and (self._t - self._arrival_t) >= self._settle_s:
            self._finish(reached=True, reason='goal reached')
        elif self._t >= self._timeout_s:
            self._finish(reached=False, reason='scenario timeout')

    def _finish(self, reached: bool, reason: str):
        self._finished = True
        for timer in (self._sim_timer, self._grid_timer, self._slow_timer,
                      self._monitor_timer, self._activate_timer):
            timer.cancel()

        n = len(self._solve_time_ms)
        mean_ms = statistics.fmean(self._solve_time_ms) if n else 0.0
        std_ms  = statistics.pstdev(self._solve_time_ms) if n > 1 else 0.0
        n_optimal = sum(self._solve_optimal)
        pct_optimal = 100.0 * n_optimal / n if n else 0.0
        tf_min = min(self._effective_tf) if self._effective_tf else 0.0
        tf_max = max(self._effective_tf) if self._effective_tf else 0.0

        report = (
            '\n' + '=' * 72 + '\n'
            'Final-Time MPC Test Scenario Complete\n'
            f'  Result:            {"GOAL REACHED" if reached else "TIMED OUT"} ({reason})\n'
            f'  Scenario duration: {self._t:.1f} s (sim time)\n'
            f'  MPC solve cycles:  {n}\n'
            f'  Solve time [ms]:   mean={mean_ms:.2f}  std={std_ms:.2f}\n'
            f'  Optimal solves:    {pct_optimal:.1f}% ({n_optimal}/{n})\n'
            f'  Effective tf [s]:  min={tf_min:.2f}  max={tf_max:.2f}\n'
            + '=' * 72
        )
        self.get_logger().info(report)
        rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = FinalTimeMPCTestDriver()
    # rclpy.spin(node) relies on the global executor's blocking spin_once(),
    # which does not reliably wake up on a shutdown requested from within a
    # callback (rclpy.try_shutdown() in _finish()).  Spinning explicitly with
    # a timeout and checking rclpy.ok() each iteration ensures the process
    # actually exits once the scenario finishes, which the launch file
    # depends on to tear down the rest of the stack and finalize the
    # recording.
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
