#!/usr/bin/env python3
"""
Formation distance-keeping study test driver.

Two vehicles: a scripted lead and a follower governed by the MPC local planner.
The lead follows a prescribed trajectory; the follower receives the lead's true
odometry (no object tracker or comms delay) and must maintain the formation
offset.  This driver replaces the mission manager for path generation: it
computes the formation target path directly from the leader odometry and the
selected formation shape, so only the goal_point_processor and MPC nodes are
needed in addition to this driver.

Leader motion profiles (leader_motion parameter):
  straight           -- constant forward speed, zero yaw rate
  sine               -- constant forward speed with sinusoidal yaw rate
  straight_then_sine -- straight for straight_duration seconds, then sine

Formation shapes (formation parameter):
  column -- follower offset: x = -x_scale m (behind), y = 0
  wedge  -- follower offset: x = -x_scale m (behind), y = +y_scale m (left)

Offset conventions match formation_definition.cpp:
  x < 0 => behind leader along leader heading
  y > 0 => to the left of leader (right-hand frame)

The follower obeys the same 3-DOF Pacejka dynamic bicycle model as the MPC
(ThreeDOF_rigid in mpc_models.jl, parameters from mpc_parameters.jl for the
MRZR).  The MPC speed command is applied directly (no powertrain or speed
controller).  Physics are integrated with a 1 ms fixed time-step; the publish
rate is independently set by sim_rate_hz.

Published topics:
  avt_341/odometry              -- follower vehicle odometry (for veh_converter)
  avt_341/leader_odometry       -- leader vehicle odometry (true, zero noise)
  avt_341/global_path           -- formation target path (nav_msgs/Path)
  avt_341/occupancy_grid        -- empty occupancy grid (for obstacle_processor)
  avt_341/occupancy_grid_low_res -- empty occupancy grid (for any planner nodes)
  avt_341/nav_command_state     -- NavStackState Active (0)
  avt_341/speed_setpoint        -- leader speed [m/s]
  avt_341/leader_status         -- Bool True (leader is present)
  avt_341/follower_status       -- formation offsets for goal_point_processor
  avt_341/steering_angle        -- follower steering angle (for veh_converter)
  avt_341/lead_speed_text       -- Marker: leader speed label
  avt_341/follow_speed_text     -- Marker: follower speed label

Subscribed topics:
  avt_341/drive                 -- AckermannDriveStamped commands from MPC

TF frames broadcast:
  odom -> base_link      (follower)
  odom -> lead_base_link (leader)

Starting positions
------------------
For COLUMN with default x_scale=5.0:
  leader at (0, 0), follower at (-5, 0).

For WEDGE with default x_scale=5.0, y_scale=5.0:
  leader at (0, 0), follower at (-5, 5)
  -- override start_x_follow=-5 start_y_follow=5 in the launch arguments.
"""

import math
import collections

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from std_msgs.msg import Float64, Int32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from avt_341_msgs.msg import FollowerStatus


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


# ---------------------------------------------------------------------------
# MRZR vehicle parameters matching mpc_parameters.jl (Pacejka rigid tires)
# ---------------------------------------------------------------------------
_G    = 9.81
_LA   = 1.54134           # CG to front axle [m]
_L    = 2.71534           # wheelbase [m]
_LB   = _L - _LA          # CG to rear axle [m]
_M    = 1.269e3           # vehicle mass [kg]
_IZZ  = 1.620e3           # yaw moment of inertia [kg*m^2]
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


def _pacejka_lateral_forces(v: float, r: float, ux: float, sa: float) -> tuple:
    """Pacejka lateral forces matching ThreeDOF_rigid in mpc_models.jl.

    ax is assumed zero because speed is commanded directly from MPC output,
    bypassing the powertrain.  The static normal loads FZF0 / FZR0 apply.
    """
    fyf_fzf = _FZF0
    fyr_fzr = _FZR0

    x1f = _P1 / _MU * (math.atan2(v + _LA * r, _EP + ux) - sa)
    x1r = _P1 / _MU * math.atan2(v - _LB * r, _EP + ux)

    fyf = 2.0 * _MU * fyf_fzf * math.sin(
        _P2 * math.atan(x1f - _P3 * (x1f - math.atan(x1f))))
    fyr = 2.0 * _MU * fyr_fzr * math.sin(
        _P2 * math.atan(x1r - _P3 * (x1r - math.atan(x1r))))
    return fyf, fyr


# ---------------------------------------------------------------------------
# Formation offset unit vectors (before scaling).
# Conventions: x<0 → behind leader; y>0 → left of leader.
# Matches formation_definition.cpp (follower1 offsets for each shape).
# ---------------------------------------------------------------------------
_FORMATION_UNIT_OFFSETS = {
    'column': (-1.0,  0.0),
    'wedge':  (-1.0,  1.0),
}

_MAX_PATH_POINTS = 2000   # maximum accumulated global-path waypoints kept


class FormationDistanceTestDriver(Node):
    def __init__(self):
        super().__init__('formation_distance_test_driver')

        # -- Parameters -------------------------------------------------
        self.declare_parameter('formation', 'column')
        self.declare_parameter('leader_motion', 'straight')
        self.declare_parameter('leader_speed', 3.0)
        self.declare_parameter('sine_yaw_rate_amp', 0.15)   # [rad/s]
        self.declare_parameter('sine_period', 10.0)          # [s]
        self.declare_parameter('straight_duration', 10.0)    # [s]
        self.declare_parameter('x_scale', 5.0)               # [m]
        self.declare_parameter('y_scale', 5.0)               # [m]
        self.declare_parameter('start_x_lead', 0.0)
        self.declare_parameter('start_y_lead', 0.0)
        self.declare_parameter('start_yaw_lead_deg', 0.0)
        self.declare_parameter('start_x_follow', -5.0)
        self.declare_parameter('start_y_follow', 0.0)
        self.declare_parameter('start_yaw_follow_deg', 0.0)
        self.declare_parameter('map_size_m', 200.0)
        self.declare_parameter('map_resolution_m', 1.0)
        self.declare_parameter('path_point_spacing', 0.5)    # [m]
        self.declare_parameter('sim_rate_hz', 50.0)
        self.declare_parameter('physics_dt', 0.001)          # [s]
        self.declare_parameter('mpc_max_speed', 4.0)         # must match max_speed in mpc_local_planner.yaml

        formation     = self.get_parameter('formation').value.lower()
        leader_motion = self.get_parameter('leader_motion').value.lower()

        if formation not in _FORMATION_UNIT_OFFSETS:
            self.get_logger().warn(
                f"Unknown formation '{formation}', defaulting to 'column'.")
            formation = 'column'
        self._formation     = formation
        self._leader_motion = leader_motion

        self._leader_speed = float(self.get_parameter('leader_speed').value)
        self._r_amp        = float(self.get_parameter('sine_yaw_rate_amp').value)
        self._sine_period  = float(self.get_parameter('sine_period').value)
        self._straight_dur = float(self.get_parameter('straight_duration').value)
        self._x_scale      = float(self.get_parameter('x_scale').value)
        self._y_scale      = float(self.get_parameter('y_scale').value)
        self._path_spacing = float(self.get_parameter('path_point_spacing').value)
        self._physics_dt   = float(self.get_parameter('physics_dt').value)
        self._mpc_max_speed = float(self.get_parameter('mpc_max_speed').value)
        sim_rate           = float(self.get_parameter('sim_rate_hz').value)
        self._pub_dt       = 1.0 / sim_rate

        self._n_substeps = max(1, int(round(self._pub_dt / self._physics_dt)))

        unit_x, unit_y = _FORMATION_UNIT_OFFSETS[formation]
        self._x_offset = unit_x * self._x_scale
        self._y_offset = unit_y * self._y_scale

        # -- Leader kinematic state ------------------------------------
        self._xl    = float(self.get_parameter('start_x_lead').value)
        self._yl    = float(self.get_parameter('start_y_lead').value)
        self._psi_l = math.radians(self.get_parameter('start_yaw_lead_deg').value)
        self._t     = 0.0   # simulation time [s]

        # -- Follower 3-DOF state (ThreeDOF_rigid, CG frame) -----------
        self._x   = float(self.get_parameter('start_x_follow').value)
        self._y   = float(self.get_parameter('start_y_follow').value)
        self._psi = math.radians(self.get_parameter('start_yaw_follow_deg').value)
        self._v   = 0.0   # lateral velocity [m/s]
        self._r   = 0.0   # yaw rate [rad/s]
        self._ux  = 0.0   # longitudinal speed [m/s] -- set by MPC drive command
        self._sa  = 0.0   # steering angle [rad]     -- set by MPC drive command

        # -- Formation path (oldest point first, sliding window) --------
        self._path_pts: collections.deque = collections.deque(maxlen=_MAX_PATH_POINTS)
        self._last_leader_x_at_path: float | None = None
        self._last_leader_y_at_path: float | None = None

        # -- TF broadcaster --------------------------------------------
        self._tf_br = TransformBroadcaster(self)

        # -- Publishers ------------------------------------------------
        self._follow_odom_pub = self.create_publisher(
            Odometry, 'avt_341/odometry', 1)
        self._leader_odom_pub = self.create_publisher(
            Odometry, 'avt_341/leader_odometry', 1)
        self._path_pub = self.create_publisher(
            Path, 'avt_341/global_path', 1)
        self._grid_pub = self.create_publisher(
            OccupancyGrid, 'avt_341/occupancy_grid', 1)
        self._grid_lr_pub = self.create_publisher(
            OccupancyGrid, 'avt_341/occupancy_grid_low_res', 1)
        self._nav_pub = self.create_publisher(
            Int32, 'avt_341/nav_command_state', 1)
        self._speed_pub = self.create_publisher(
            Float64, 'avt_341/speed_setpoint', 1)
        self._lead_stat_pub = self.create_publisher(
            Bool, 'avt_341/leader_status', 1)
        self._follow_stat_pub = self.create_publisher(
            FollowerStatus, 'avt_341/follower_status', 1)
        self._steer_pub = self.create_publisher(
            Float64, 'avt_341/steering_angle', 1)
        self._lead_txt_pub = self.create_publisher(
            Marker, 'avt_341/lead_speed_text', 1)
        self._follow_txt_pub = self.create_publisher(
            Marker, 'avt_341/follow_speed_text', 1)

        # -- Subscriber ------------------------------------------------
        self._drive_sub = self.create_subscription(
            AckermannDriveStamped, 'avt_341/drive', self._drive_cb, 1)

        # -- Timers ----------------------------------------------------
        self._sim_timer  = self.create_timer(self._pub_dt, self._sim_step)
        self._grid_timer = self.create_timer(1.0, self._publish_grid)
        self._slow_timer = self.create_timer(0.1, self._publish_slow)

        self.get_logger().info(
            f'Formation distance test driver started. '
            f'Formation={formation}, motion={leader_motion}, '
            f'leader_speed={self._leader_speed:.1f} m/s, '
            f'x_offset={self._x_offset:.1f} m, '
            f'y_offset={self._y_offset:.1f} m, '
            f'physics_dt={self._physics_dt * 1000:.0f} ms, '
            f'sub-steps per tick: {self._n_substeps}.')

    # ------------------------------------------------------------------
    # MPC drive command callback
    # ------------------------------------------------------------------
    def _drive_cb(self, msg: AckermannDriveStamped):
        self._ux = msg.drive.speed
        self._sa = msg.drive.steering_angle

    # ------------------------------------------------------------------
    # Leader yaw rate
    # ------------------------------------------------------------------
    def _leader_yaw_rate(self, t: float) -> float:
        motion = self._leader_motion
        if motion == 'straight':
            return 0.0
        omega  = 2.0 * math.pi / self._sine_period
        r_sine = self._r_amp * math.sin(omega * t)
        if motion == 'sine':
            return r_sine
        # straight_then_sine
        return 0.0 if t < self._straight_dur else r_sine

    # ------------------------------------------------------------------
    # Physics integration (1 ms sub-steps)
    # ------------------------------------------------------------------
    def _step_leader(self, dt: float):
        """Advance leader kinematic state by dt seconds."""
        r_l          = self._leader_yaw_rate(self._t)
        self._xl    += self._leader_speed * math.cos(self._psi_l) * dt
        self._yl    += self._leader_speed * math.sin(self._psi_l) * dt
        self._psi_l += r_l * dt
        self._t     += dt

    def _step_follower(self, dt: float):
        """Advance follower 3-DOF Pacejka state by dt seconds.

        ux and sa are held constant between MPC drive messages (MPC at 20 Hz).
        The speed command is applied directly -- no powertrain simulation.
        """
        v, r, psi = self._v, self._r, self._psi
        ux, sa    = self._ux, self._sa

        fyf, fyr = _pacejka_lateral_forces(v, r, ux, sa)

        self._x   += (ux * math.cos(psi) - v * math.sin(psi)) * dt
        self._y   += (ux * math.sin(psi) + v * math.cos(psi)) * dt
        self._v   += ((fyf + fyr) / _M - r * ux) * dt
        self._r   += ((_LA * fyf - _LB * fyr) / _IZZ) * dt
        self._psi += r * dt

    # ------------------------------------------------------------------
    # Formation path management
    # ------------------------------------------------------------------
    def _update_path(self):
        """Append a formation target waypoint when the leader has moved enough."""
        if self._last_leader_x_at_path is None:
            self._append_target()
            self._last_leader_x_at_path = self._xl
            self._last_leader_y_at_path = self._yl
            return
        dx = self._xl - self._last_leader_x_at_path
        dy = self._yl - self._last_leader_y_at_path
        if dx * dx + dy * dy >= self._path_spacing ** 2:
            self._append_target()
            self._last_leader_x_at_path = self._xl
            self._last_leader_y_at_path = self._yl

    def _append_target(self):
        """Compute and append the current formation target position."""
        # Leader body frame vectors in the world frame
        vx = [math.cos(self._psi_l),  math.sin(self._psi_l)]   # forward
        vy = [-math.sin(self._psi_l), math.cos(self._psi_l)]   # left

        tx = self._xl + vx[0] * self._x_offset + vy[0] * self._y_offset
        ty = self._yl + vx[1] * self._x_offset + vy[1] * self._y_offset

        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tx
        ps.pose.position.y = ty
        ps.pose.orientation.w = 1.0
        self._path_pts.append(ps)

    # ------------------------------------------------------------------
    # Main simulation timer callback (sim_rate_hz)
    # ------------------------------------------------------------------
    def _sim_step(self):
        dt = self._physics_dt
        for _ in range(self._n_substeps):
            self._step_leader(dt)
            self._step_follower(dt)
        self._update_path()

        now = self.get_clock().now().to_msg()
        self._publish_follower_odom(now)
        self._publish_leader_odom(now)
        self._publish_path(now)
        self._publish_tfs(now)
        self._publish_speed_markers(now)
        self._publish_steering()

    # ------------------------------------------------------------------
    # Per-tick publishers
    # ------------------------------------------------------------------
    def _publish_follower_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp     = stamp
        msg.header.frame_id  = 'odom'
        msg.child_frame_id   = 'base_link'
        msg.pose.pose.position.x    = self._x
        msg.pose.pose.position.y    = self._y
        msg.pose.pose.orientation   = yaw_to_quaternion(self._psi)
        msg.twist.twist.linear.x    = self._ux
        msg.twist.twist.linear.y    = self._v
        msg.twist.twist.angular.z   = self._r
        self._follow_odom_pub.publish(msg)

    def _publish_leader_odom(self, stamp):
        msg = Odometry()
        msg.header.stamp     = stamp
        msg.header.frame_id  = 'odom'
        msg.child_frame_id   = 'lead_base_link'
        msg.pose.pose.position.x    = self._xl
        msg.pose.pose.position.y    = self._yl
        msg.pose.pose.orientation   = yaw_to_quaternion(self._psi_l)
        msg.twist.twist.linear.x    = self._leader_speed
        msg.twist.twist.angular.z   = self._leader_yaw_rate(self._t)
        self._leader_odom_pub.publish(msg)

    def _publish_path(self, stamp):
        if not self._path_pts:
            return
        path = Path()
        path.header.stamp    = stamp
        path.header.frame_id = 'map'
        path.poses = list(self._path_pts)
        self._path_pub.publish(path)

    def _publish_tfs(self, stamp):
        transforms = []
        for child, x, y, yaw in [
            ('base_link',      self._x,  self._y,  self._psi),
            ('lead_base_link', self._xl, self._yl, self._psi_l),
        ]:
            t = TransformStamped()
            t.header.stamp       = stamp
            t.header.frame_id    = 'odom'
            t.child_frame_id     = child
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation      = yaw_to_quaternion(yaw)
            transforms.append(t)
        self._tf_br.sendTransform(transforms)

    def _publish_speed_markers(self, stamp):
        for ns, x, y, speed, pub in [
            ('lead_speed',   self._xl, self._yl, self._leader_speed, self._lead_txt_pub),
            ('follow_speed', self._x,  self._y,  self._ux,           self._follow_txt_pub),
        ]:
            m = Marker()
            m.header.stamp      = stamp
            m.header.frame_id   = 'map'
            m.ns                = ns
            m.id                = 0
            m.type              = Marker.TEXT_VIEW_FACING
            m.action            = Marker.ADD
            m.pose.position.x   = x
            m.pose.position.y   = y
            m.pose.position.z   = 2.0
            m.scale.z           = 1.5
            m.color.r           = 1.0
            m.color.g           = 1.0
            m.color.b           = 1.0
            m.color.a           = 1.0
            m.text              = f'{speed:.1f} m/s'
            pub.publish(m)

    def _publish_steering(self):
        msg      = Float64()
        msg.data = self._sa
        self._steer_pub.publish(msg)

    # ------------------------------------------------------------------
    # Slow / periodic publishers
    # ------------------------------------------------------------------
    def _publish_grid(self):
        """Publish an empty occupancy grid centered on the origin."""
        size_m = self.get_parameter('map_size_m').value
        res    = self.get_parameter('map_resolution_m').value
        ncells = int(size_m / res)

        msg = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'map'
        msg.info.resolution           = res
        msg.info.width                = ncells
        msg.info.height               = ncells
        msg.info.origin.position.x    = -size_m / 2.0
        msg.info.origin.position.y    = -size_m / 2.0
        msg.data                      = [0] * (ncells * ncells)
        self._grid_pub.publish(msg)
        self._grid_lr_pub.publish(msg)

    def _publish_slow(self):
        """Publish slow-changing support topics at 10 Hz."""
        nav      = Int32()
        nav.data = 0   # NavStackState::Active
        self._nav_pub.publish(nav)

        sp      = Float64()
        sp.data = self._mpc_max_speed
        self._speed_pub.publish(sp)

        ls      = Bool()
        ls.data = False  # False = "I am not the leader" -> follower_status=true in MPC
        self._lead_stat_pub.publish(ls)

        unit_x, unit_y = _FORMATION_UNIT_OFFSETS[self._formation]
        fs             = FollowerStatus()
        fs.leader_name = 'leader'
        fs.x_offset    = float(unit_x * self._x_scale)
        fs.y_offset    = float(unit_y * self._y_scale)
        fs.use_leader  = True
        self._follow_stat_pub.publish(fs)


def main(args=None):
    rclpy.init(args=args)
    node = FormationDistanceTestDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
