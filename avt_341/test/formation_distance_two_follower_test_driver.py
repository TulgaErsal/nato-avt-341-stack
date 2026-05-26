#!/usr/bin/env python3
"""
Two-follower wedge formation test driver.

Reproduces the follower-offset swap bug introduced in the
fix-formation-distance-keeping branch: in a WEDGE formation with two
followers, each follower picks the other follower's target location.

Fixed formation layout (WEDGE, from formation_definition.cpp):
  MRZR4 -- follower1: x_offset = -x_scale (behind), y_offset = +y_scale (left)
  MRZR2 -- follower2: x_offset = -x_scale (behind), y_offset = -y_scale (right)

Expected: MRZR4 tracks the position behind-left of the leader;
          MRZR2 tracks the position to the right of the leader.
Bug:      the two go to each other's target instead.

Each follower's full stack (veh_converter, obstacle_processor,
goal_point_processor, mpc_planner) runs in its own ROS2 namespace
(mrzr2 or mrzr4).  This driver publishes all support topics into both
namespaces and subscribes to the drive commands that each MPC outputs.

Published topics (per namespace 'ns' in {mrzr2, mrzr4}):
  ns/avt_341/odometry              -- follower odometry
  ns/avt_341/leader_odometry       -- leader odometry (true, zero noise)
  ns/avt_341/global_path           -- formation target path
  ns/avt_341/occupancy_grid        -- empty grid
  ns/avt_341/occupancy_grid_low_res
  ns/avt_341/nav_command_state     -- NavStackState Active (0)
  ns/avt_341/speed_setpoint        -- MPC max speed [m/s]
  ns/avt_341/leader_status         -- Bool False (follower mode on)
  ns/avt_341/follower_status       -- formation offsets for goal_point_processor
                                      and MPC (the new consumer in this branch)
  ns/avt_341/steering_angle        -- follower steering angle

Subscribed topics:
  mrzr2/avt_341/drive              -- AckermannDriveStamped from MRZR2 MPC
  mrzr4/avt_341/drive              -- AckermannDriveStamped from MRZR4 MPC

TF frames broadcast:
  odom -> mrzr2_base_link
  odom -> mrzr4_base_link
  odom -> lead_base_link

Default starting positions (nominal wedge at x_scale=5, y_scale=5):
  Leader  (0,  0)
  MRZR4  (-5, +5)   [behind, left]
  MRZR2  (-5, -5)   [behind, right]
"""

import math
import collections

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from std_msgs.msg import Float64, Int32, Bool
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster
from avt_341_msgs.msg import FollowerStatus


def _yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q


# ---------------------------------------------------------------------------
# MRZR Pacejka 3-DOF parameters (matching mpc_parameters.jl)
# ---------------------------------------------------------------------------
_G    = 9.81
_LA   = 1.54134
_L    = 2.71534
_LB   = _L - _LA
_M    = 1.269e3
_IZZ  = 1.620e3
_FZ   = _M * _G
_FZF0 = _FZ * _LB / _L / 2.0
_FZR0 = _FZ * _LA / _L / 2.0
_MU   = 0.977706
_P1   = -7.33706
_P2   = 1.11368
_P3   = -1.04179
_EP   = 0.01

_MAX_PATH_PTS = 2000


def _pacejka(v, r, ux, sa):
    x1f = _P1 / _MU * (math.atan2(v + _LA * r, _EP + ux) - sa)
    x1r = _P1 / _MU * math.atan2(v - _LB * r, _EP + ux)
    fyf = 2.0 * _MU * _FZF0 * math.sin(_P2 * math.atan(x1f - _P3 * (x1f - math.atan(x1f))))
    fyr = 2.0 * _MU * _FZR0 * math.sin(_P2 * math.atan(x1r - _P3 * (x1r - math.atan(x1r))))
    return fyf, fyr


# ---------------------------------------------------------------------------
# Per-follower context: state, publishers, path buffer
# ---------------------------------------------------------------------------
class _FollowerContext:
    def __init__(self, node, ns, x_offset, y_offset,
                 start_x, start_y, start_yaw, base_link_frame):
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.base_link_frame = base_link_frame

        # 3-DOF vehicle state
        self.x, self.y, self.psi = start_x, start_y, start_yaw
        self.v = self.r = self.ux = self.sa = 0.0

        # Formation path waypoints
        self.path_pts = collections.deque(maxlen=_MAX_PATH_PTS)
        self.last_lx_at_path = None
        self.last_ly_at_path = None

        def _pub(suffix, msg_type):
            return node.create_publisher(msg_type, f'{ns}/avt_341/{suffix}', 1)

        self.odom_pub       = _pub('odometry',               Odometry)
        self.lead_odom_pub  = _pub('leader_odometry',        Odometry)
        self.path_pub       = _pub('global_path',            Path)
        self.grid_pub       = _pub('occupancy_grid',         OccupancyGrid)
        self.grid_lr_pub    = _pub('occupancy_grid_low_res', OccupancyGrid)
        self.nav_pub        = _pub('nav_command_state',      Int32)
        self.speed_pub      = _pub('speed_setpoint',         Float64)
        self.lead_stat_pub  = _pub('leader_status',          Bool)
        self.follow_stat_pub = _pub('follower_status',       FollowerStatus)
        self.steer_pub      = _pub('steering_angle',         Float64)

    def drive_cb(self, msg: AckermannDriveStamped):
        self.ux = msg.drive.speed
        self.sa = msg.drive.steering_angle


# ---------------------------------------------------------------------------
# Main test driver node
# ---------------------------------------------------------------------------
class FormationDistanceTwoFollowerTestDriver(Node):
    def __init__(self):
        super().__init__('formation_distance_two_follower_test_driver')

        self.declare_parameter('leader_motion',      'straight')
        self.declare_parameter('leader_speed',       3.0)
        self.declare_parameter('sine_yaw_rate_amp',  0.15)
        self.declare_parameter('sine_period',        10.0)
        self.declare_parameter('straight_duration',  10.0)
        self.declare_parameter('x_scale',            5.0)
        self.declare_parameter('y_scale',            5.0)
        self.declare_parameter('start_x_lead',       0.0)
        self.declare_parameter('start_y_lead',       0.0)
        self.declare_parameter('start_yaw_lead_deg', 0.0)
        self.declare_parameter('map_size_m',         200.0)
        self.declare_parameter('map_resolution_m',   1.0)
        self.declare_parameter('path_point_spacing', 0.5)
        self.declare_parameter('sim_rate_hz',        50.0)
        self.declare_parameter('physics_dt',         0.001)
        self.declare_parameter('mpc_max_speed',      8.0)

        self._leader_motion = self.get_parameter('leader_motion').value.lower()
        self._leader_speed  = float(self.get_parameter('leader_speed').value)
        self._r_amp         = float(self.get_parameter('sine_yaw_rate_amp').value)
        self._sine_period   = float(self.get_parameter('sine_period').value)
        self._straight_dur  = float(self.get_parameter('straight_duration').value)
        x_scale             = float(self.get_parameter('x_scale').value)
        y_scale             = float(self.get_parameter('y_scale').value)
        self._path_spacing  = float(self.get_parameter('path_point_spacing').value)
        self._physics_dt    = float(self.get_parameter('physics_dt').value)
        self._mpc_max_speed = float(self.get_parameter('mpc_max_speed').value)
        sim_rate            = float(self.get_parameter('sim_rate_hz').value)
        self._pub_dt        = 1.0 / sim_rate
        self._n_substeps    = max(1, int(round(self._pub_dt / self._physics_dt)))

        # Leader state
        self._xl    = float(self.get_parameter('start_x_lead').value)
        self._yl    = float(self.get_parameter('start_y_lead').value)
        self._psi_l = math.radians(self.get_parameter('start_yaw_lead_deg').value)
        self._t     = 0.0

        # WEDGE formation offsets:
        mrzr4_xo = -1.0 * x_scale
        mrzr4_yo = +1.0 * y_scale
        mrzr2_xo = -1.0 * x_scale
        mrzr2_yo = -1.0 * y_scale

        self._ctx_mrzr4 = _FollowerContext(
            self, 'mrzr4', mrzr4_xo, mrzr4_yo,
            start_x=-x_scale, start_y=+y_scale, start_yaw=0.0,
            base_link_frame='mrzr4_base_link')

        self._ctx_mrzr2 = _FollowerContext(
            self, 'mrzr2', mrzr2_xo, mrzr2_yo,
            start_x=-x_scale, start_y=-y_scale, start_yaw=0.0,
            base_link_frame='mrzr2_base_link')

        self._followers = [self._ctx_mrzr4, self._ctx_mrzr2]

        self._tf_br = TransformBroadcaster(self)

        self.create_subscription(
            AckermannDriveStamped, 'mrzr2/avt_341/drive',
            self._ctx_mrzr2.drive_cb, 1)
        self.create_subscription(
            AckermannDriveStamped, 'mrzr4/avt_341/drive',
            self._ctx_mrzr4.drive_cb, 1)

        self.create_timer(self._pub_dt, self._sim_step)
        self.create_timer(1.0,          self._publish_grids)
        self.create_timer(0.1,          self._publish_slow)

        self.get_logger().info(
            f'Two-follower wedge test driver started. '
            f'MRZR4 offset ({mrzr4_xo:.1f}, {mrzr4_yo:.1f}) m, '
            f'MRZR2 offset ({mrzr2_xo:.1f}, {mrzr2_yo:.1f}) m, '
            f'leader_speed={self._leader_speed:.1f} m/s.')

    # ------------------------------------------------------------------
    def _leader_yaw_rate(self, t: float) -> float:
        motion = self._leader_motion
        if motion == 'straight':
            return 0.0
        omega  = 2.0 * math.pi / self._sine_period
        r_sine = self._r_amp * math.sin(omega * t)
        if motion == 'sine':
            return r_sine
        return 0.0 if t < self._straight_dur else r_sine

    def _step_leader(self, dt: float):
        r_l          = self._leader_yaw_rate(self._t)
        self._xl    += self._leader_speed * math.cos(self._psi_l) * dt
        self._yl    += self._leader_speed * math.sin(self._psi_l) * dt
        self._psi_l += r_l * dt
        self._t     += dt

    def _step_follower(self, ctx: _FollowerContext, dt: float):
        v, r, psi = ctx.v, ctx.r, ctx.psi
        ux, sa    = ctx.ux, ctx.sa
        fyf, fyr  = _pacejka(v, r, ux, sa)
        ctx.x   += (ux * math.cos(psi) - v * math.sin(psi)) * dt
        ctx.y   += (ux * math.sin(psi) + v * math.cos(psi)) * dt
        ctx.v   += ((fyf + fyr) / _M - r * ux) * dt
        ctx.r   += ((_LA * fyf - _LB * fyr) / _IZZ) * dt
        ctx.psi += r * dt

    # ------------------------------------------------------------------
    def _update_path(self, ctx: _FollowerContext):
        if ctx.last_lx_at_path is None:
            self._append_target(ctx)
            ctx.last_lx_at_path = self._xl
            ctx.last_ly_at_path = self._yl
            return
        dx = self._xl - ctx.last_lx_at_path
        dy = self._yl - ctx.last_ly_at_path
        if dx * dx + dy * dy >= self._path_spacing ** 2:
            self._append_target(ctx)
            ctx.last_lx_at_path = self._xl
            ctx.last_ly_at_path = self._yl

    def _append_target(self, ctx: _FollowerContext):
        c, s = math.cos(self._psi_l), math.sin(self._psi_l)
        tx = self._xl + c * ctx.x_offset - s * ctx.y_offset
        ty = self._yl + s * ctx.x_offset + c * ctx.y_offset
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = tx
        ps.pose.position.y = ty
        ps.pose.orientation.w = 1.0
        ctx.path_pts.append(ps)

    # ------------------------------------------------------------------
    def _sim_step(self):
        dt = self._physics_dt
        for _ in range(self._n_substeps):
            self._step_leader(dt)
            for ctx in self._followers:
                self._step_follower(ctx, dt)

        for ctx in self._followers:
            self._update_path(ctx)

        now = self.get_clock().now().to_msg()
        for ctx in self._followers:
            self._publish_follower_odom(ctx, now)
            self._publish_leader_odom(ctx, now)
            self._publish_path(ctx, now)
            self._publish_steering(ctx)
        self._publish_tfs(now)

    # ------------------------------------------------------------------
    def _publish_follower_odom(self, ctx: _FollowerContext, stamp):
        msg = Odometry()
        msg.header.stamp            = stamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = ctx.base_link_frame
        msg.pose.pose.position.x    = ctx.x
        msg.pose.pose.position.y    = ctx.y
        msg.pose.pose.orientation   = _yaw_to_quat(ctx.psi)
        msg.twist.twist.linear.x    = ctx.ux
        msg.twist.twist.linear.y    = ctx.v
        msg.twist.twist.angular.z   = ctx.r
        ctx.odom_pub.publish(msg)

    def _publish_leader_odom(self, ctx: _FollowerContext, stamp):
        msg = Odometry()
        msg.header.stamp            = stamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'lead_base_link'
        msg.pose.pose.position.x    = self._xl
        msg.pose.pose.position.y    = self._yl
        msg.pose.pose.orientation   = _yaw_to_quat(self._psi_l)
        msg.twist.twist.linear.x    = self._leader_speed
        msg.twist.twist.angular.z   = self._leader_yaw_rate(self._t)
        ctx.lead_odom_pub.publish(msg)

    def _publish_path(self, ctx: _FollowerContext, stamp):
        if not ctx.path_pts:
            return
        path = Path()
        path.header.stamp    = stamp
        path.header.frame_id = 'map'
        path.poses = list(ctx.path_pts)
        ctx.path_pub.publish(path)

    def _publish_tfs(self, stamp):
        entries = [(ctx.base_link_frame, ctx.x, ctx.y, ctx.psi)
                   for ctx in self._followers]
        entries.append(('lead_base_link', self._xl, self._yl, self._psi_l))
        transforms = []
        for frame, x, y, yaw in entries:
            t = TransformStamped()
            t.header.stamp       = stamp
            t.header.frame_id    = 'odom'
            t.child_frame_id     = frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation      = _yaw_to_quat(yaw)
            transforms.append(t)
        self._tf_br.sendTransform(transforms)

    def _publish_steering(self, ctx: _FollowerContext):
        msg = Float64()
        msg.data = ctx.sa
        ctx.steer_pub.publish(msg)

    # ------------------------------------------------------------------
    def _publish_grids(self):
        size_m = self.get_parameter('map_size_m').value
        res    = self.get_parameter('map_resolution_m').value
        ncells = int(size_m / res)
        grid = OccupancyGrid()
        grid.header.stamp           = self.get_clock().now().to_msg()
        grid.header.frame_id        = 'map'
        grid.info.resolution        = res
        grid.info.width             = ncells
        grid.info.height            = ncells
        grid.info.origin.position.x = -size_m / 2.0
        grid.info.origin.position.y = -size_m / 2.0
        grid.data                   = [0] * (ncells * ncells)
        for ctx in self._followers:
            ctx.grid_pub.publish(grid)
            ctx.grid_lr_pub.publish(grid)

    def _publish_slow(self):
        nav = Int32()
        nav.data = 0   # NavStackState::Active
        sp = Float64()
        sp.data = self._mpc_max_speed
        ls = Bool()
        ls.data = False   # leader is present -> follower mode on

        for ctx in self._followers:
            ctx.nav_pub.publish(nav)
            ctx.speed_pub.publish(sp)
            ctx.lead_stat_pub.publish(ls)

            fs = FollowerStatus()
            fs.leader_name = 'leader'
            fs.x_offset    = float(ctx.x_offset)
            fs.y_offset    = float(ctx.y_offset)
            fs.use_leader  = True
            ctx.follow_stat_pub.publish(fs)


def main(args=None):
    rclpy.init(args=args)
    node = FormationDistanceTwoFollowerTestDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
