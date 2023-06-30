import os

import launch.conditions
from launch.conditions import IfCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import OpaqueFunction

from launch.substitution import Substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition


class TernarySubstitution(Substitution):

    def __init__(self, true_val: SomeSubstitutionsType, false_val: SomeSubstitutionsType, condition: Condition):
        self.__true_val = true_val
        self.__false_val = false_val
        self.__condition = condition

    def describe(self):
        return 'TernarySubstitution(%s %s %s)' % (self.__true_val.describe(), self.__false_val.describe(), self.__condition.describe())

    def perform(self, context: launch.LaunchContext):
        if self.__condition.evaluate(context):
            return self.__true_val.perform(context)
        else:
            return self.__false_val.perform(context)


class ToUpper(Substitution):

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'ToUpper(%s)' % (self.__sub_val.describe())

    def perform(self, context: launch.LaunchContext):
        return self.__sub_val.perform(context).upper()


class Invert(Substitution):

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'Invert(%s)' % (self.__sub_val.describe())

    def perform(self, context: launch.LaunchContext):
        val = self.__sub_val.perform(context).lower()
        is_true = val in ['true', '1']
        return str(not is_true)


class Concat(Substitution):

        def __init__(self, sub_val: SomeSubstitutionsType, concat_val):
            self.__sub_val = sub_val
            self.__concat_val = concat_val

        def describe(self):
            return 'StringConcate(%s %s)' % (self.__sub_val.describe(), self.__concat_val)

        def perform(self, context: launch.LaunchContext):
            return self.__sub_val.perform(context) + self.__concat_val


class ArrayIndexSubstitution(Substitution):

    def __init__(self, sub_val: SomeSubstitutionsType, idx: int):
        self.__sub_val = sub_val
        self.__idx = idx

    def describe(self):
        return 'ArrayIndexSubstitution(%s %d)' % (self.__sub_val.describe(), self.__idx)

    def perform(self, context: launch.LaunchContext):
        array_val = self.__sub_val.perform(context)
        # array_val is current a string, need to parse
        array_val = array_val.replace('[', '', 1)[::-1].replace(']', '', 1)[::-1].replace(' ', '').replace("'", "").split(',')
        return array_val[self.__idx]


def evaluate_waypoint_parameters(context, *args, **kwargs):
    waypoints_file_path = LaunchConfiguration('waypoints_file').perform(context)
    waypoints_x = "[ ]"
    waypoints_y = "[ ]"
    is_empty_waypoints = not waypoints_file_path
    with open(waypoints_file_path, 'r') as f:
        for line in f.readlines():
            if "waypoints_x" in line:
                waypoints_x = line.split(":")[1]
                is_empty_waypoints = is_empty_waypoints or waypoints_x.replace(' ', '') == '[]'
            if "waypoints_y" in line:
                waypoints_y = line.split(":")[1]
                is_empty_waypoints = is_empty_waypoints or waypoints_y.replace(' ', '') == '[]'

    if is_empty_waypoints:
        waypoints_x = "[ 0.0 ]"
        waypoints_y = "[ 0.0 ]"

    return [
        DeclareLaunchArgument('waypoints_x', description="List of waypoint x coordinates. Will override waypoints_file is specified.", default_value=waypoints_x),
        DeclareLaunchArgument('waypoints_y', description="List of waypoint y coordinates. Will override waypoints_file is specified.", default_value=waypoints_y),
        DeclareLaunchArgument('is_empty_waypoints',
                              description="Parameter set internally to detect if waypoints file empty. ROS2 foxy workaround (https://answers.ros.org/question/396556/what-is-best-practice-for-parameters-which-are-empty-lists-in-ros2/). Do not set manually",
                              default_value=str(is_empty_waypoints).capitalize()),
    ]


def generate_launch_description():

    MAX_VEHICLES = 4
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_launch_rviz = LaunchConfiguration("auto_launch_rviz")
    display_type = LaunchConfiguration('display_type')

    rviz_config_single_vehicle = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_ros2.rviz')
    rviz_config_multi_vehicle = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_multi_vehicle_ros2.rviz')
    rviz_config_two_vehicle = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_two_vehicle_ros2.rviz')

    robot_desc_list = [LaunchConfiguration('robot_description'), LaunchConfiguration('robot_description_veh2'),
                       LaunchConfiguration('robot_description_veh3'), LaunchConfiguration('robot_description_veh4')]

    arg_list = [
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('auto_launch_rviz', default_value='True', description="Automatically launch rviz display window"),
        DeclareLaunchArgument('display_type', default_value='rviz', description="Type of display method to use. Values = [rviz, image]"),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(get_package_share_directory('avt_341'), 'config', 'no_waypoints.yaml'), description="Path to waypoint file to use"),
        DeclareLaunchArgument('robot_description', description="URDF robot description contents"),
        DeclareLaunchArgument('robot_description_veh2', description="URDF robot description contents for vehicle 2", default_value=""),
        DeclareLaunchArgument('robot_description_veh3', description="URDF robot description contents for vehicle 3", default_value=""),
        DeclareLaunchArgument('robot_description_veh4', description="URDF robot description contents for vehicle 4", default_value=""),
        DeclareLaunchArgument('num_vehicles', default_value='1', description="Number of vehicles controlled by navigation stack."),
        DeclareLaunchArgument('namespace_single_vehicle', default_value='False', description="If true, will use vehicle namespace even when only a single vehicle is used (num_vehicles=1)."),

        # Elevation Grid
        DeclareLaunchArgument('use_elevation', default_value='False', description="Elevation grid - To use elevation or slope value when making occupancy grid based on heightmap."),
        DeclareLaunchArgument('slope_threshold', default_value='0.5', description="Elevation grid - Threshold within which next waypoint selected."),
        DeclareLaunchArgument('grid_height', default_value='200.0', description="Elevation grid - Grid height."),
        DeclareLaunchArgument('grid_width', default_value='200.0', description="Elevation grid - Grid width."),
        DeclareLaunchArgument('grid_llx', default_value='-100.0', description="Elevation grid - X coordinate grid bottom left anchor point."),
        DeclareLaunchArgument('grid_lly', default_value='-100.0', description="Elevation grid - Y coordinate grid bottom left anchor point."),
        DeclareLaunchArgument('grid_res', default_value='0.5', description="Elevation grid - Grid resolution in meters."),
        DeclareLaunchArgument('grid_dilate', default_value='False', description="Elevation grid - Whether or not to apply dilation."),
        DeclareLaunchArgument('grid_dilate_x', default_value='2.0', description="Elevation grid - Amount of dilation in x."),
        DeclareLaunchArgument('grid_dilate_y', default_value='2.0', description="Elevation grid - Amount of dilation in y."),
        DeclareLaunchArgument('grid_dilate_proportion', default_value='0.8', description="Elevation grid - Proportion of original grid cell to dilate with."),
        DeclareLaunchArgument('cull_lidar', default_value='False', description="Elevation grid - Cull lidar points flag based on distance from odometry."),
        DeclareLaunchArgument('cull_lidar_dist', default_value='90.0', description="Elevation grid - Maximum distance used to cull lidar points"),
        DeclareLaunchArgument('cull_lidar_dist_min', default_value='0.0', description="Elevation grid - Minimum distance used to cull lidar points"),
        DeclareLaunchArgument('use_registered', default_value='True', description="Elevation grid - If true, assumes lidar points are in world coordinates. Else assumes in robot odom coordinates."),
        DeclareLaunchArgument('stitch_lidar_points', default_value='True', description="Elevation grid - If true, lidar scans will be stitched together. Else, each point cloud 2 message will be independent and the grid will be cleared between messages."),
        DeclareLaunchArgument('filter_highest_lidar', default_value='False', description="Elevation grid - If true, the highest point in each cell will be ignored and the second highest will be used for the slope calculations. If false, the highest point will be used."),
        DeclareLaunchArgument('time_register_window', default_value='0.05', description="Maximum window of time between odom and point cloud time stamps"),
        DeclareLaunchArgument('perception_rate', default_value='20.0', description="Publish rate for perception node"),

        # Elevation Grid - Clearing Methods
        DeclareLaunchArgument('clear_method_type', default_value='none', description="Elevation grid - Costmap clearing method. none | time | raytrace | raytrace_obs_filter."),
        DeclareLaunchArgument('clear_method_visualize', default_value='False', description="Elevation grid - If true, visualizes costmap clearing method."),
        DeclareLaunchArgument('clear_method_visualize_range', default_value='40.0', description="Elevation grid - Visualization range for clear method debugging. If < 0 range limit is ignored and all cells are displayed."),
        DeclareLaunchArgument('clear_method_raytrace_range', default_value='50.0', description="Elevation grid - Maximum range for raytrace clearing."),
        DeclareLaunchArgument('clear_method_use_voxels', default_value='True', description="Elevation grid - Maximum range for raytrace clearing."),
        DeclareLaunchArgument('clear_method_voxel_height_min', default_value='0.0', description="Elevation grid - Voxel grid height min used in raytrace_voxel clearing method."),
        DeclareLaunchArgument('clear_method_voxel_height_res', default_value='0.5', description="Elevation grid - Voxel grid height resolution used in raytrace_voxel clearing method."),
        DeclareLaunchArgument('clear_method_immediate_clear_dilation', default_value='True', description="Elevation grid - If true, dilation will be immediately cleared with cleared cells. If False, dilation will only be removed when dilated cell scanned and re-adjusts its height values."),
        DeclareLaunchArgument('clear_method_obs_filter_range', default_value='1.0', description="Elevation grid - Minimum distance from obstacles for clearing to occur. Only used with raytrace_obs_filter option."),
        DeclareLaunchArgument('clear_method_sampled_threshold', default_value='5', description="Elevation grid - Minimum distance from obstacles for clearing to occur. Only used with raytrace_obs_filter option."),
        DeclareLaunchArgument('clear_method_max_point_age', default_value='5.0', description="Lifetime of a point before it is cleared when clear_method_type = time or no_obs_time"),

        # Global Planner
        DeclareLaunchArgument('goal_dist', default_value='5.0', description="Global planner - Lookahead threshold within which next waypoint selected."),
        DeclareLaunchArgument('debug_visualize', default_value='False', description="Global planner - Enables debug visualization of global planner."),
        DeclareLaunchArgument('w_distance', default_value='1.0', description="Global planner - Weight for distance cost."),
        DeclareLaunchArgument('w_occupancy', default_value='1.0', description="Global planner - Weight for occupancy cost."),
        DeclareLaunchArgument('w_segmentation', default_value='1.0', description="Global planner - Weight for segmentation cost."),
        DeclareLaunchArgument('search_diagonals', default_value='False', description="Global planner - If true, grid search includes adjacent cells that are diagonal. If not, only adjacent cells that are horizontal or vertical are considered."),
        DeclareLaunchArgument('los_max_iterations', default_value='1', description="Global planner - Number of iterations to apply line of sight post smoothing. Applying post smoothing multiple times can help remove excessive control points that are added due to jagged obstacle edges."),
        DeclareLaunchArgument('los_break_on_first', default_value='True', description="Global planner - If true, line of sight post-smoothing only considers first break on line of sight. If false, finds the last line of sight connection."),
        DeclareLaunchArgument('shutdown_behavior', default_value='2', description="Global planner - behavior upon reaching goal (1 - go idle, 2 - stop and shutdown, 3 - hard stop and shutdown)"),
        DeclareLaunchArgument('auto_active_on_new_waypoint', default_value='True', description="Global planner - Automatically set navstsack to active state on new goal received. If false, seperate activate command must be sent in addition to target waypoint."),
        DeclareLaunchArgument('verbose_gp_log', default_value='False', description="Global planner - Activates verbose logging for global planner."),

        # Local Planner
        DeclareLaunchArgument('local_planner_method', default_value='rcc', description="Local planner - Local planner type dwa | rcc | pf."),

        # Local Planner - Road-Centerline Constrained (RCC)
        DeclareLaunchArgument('num_paths', default_value='21', description="Local planner - Number of candidate paths to be generated."),
        DeclareLaunchArgument('path_look_ahead', default_value='30.0', description="Local planner - Planning horizon."),
        DeclareLaunchArgument('vehicle_width', default_value='3.0', description="Local planner - Vehicle width."),
        DeclareLaunchArgument('max_steer_angle', default_value='0.5', description="Local planner - Maximum steer angle in radians. Used to compute rho_max during local planning."),
        DeclareLaunchArgument('dilation_factor', default_value='1', description="Local planner - Dilation to costmap to apply during local planning only."),
        DeclareLaunchArgument('w_c', default_value='0.0', description="Local planner - w_c conformability (minimize curvature) weighting factor"),
        DeclareLaunchArgument('w_s', default_value='0.4', description="Local planner - w_s static safety (avoid collisions) weighting factor"),
        DeclareLaunchArgument('w_d', default_value='0.0', description="Local planner - w_d dynamic safety weighting factor"),
        DeclareLaunchArgument('w_r', default_value='0.2', description="Local planner - w_r rho (minimize rho offset) weighting factor"),
        DeclareLaunchArgument('w_t', default_value='0.0', description="Local planner - w_t segmentation cost weight"),
        DeclareLaunchArgument('use_global_path', default_value='True', description="Local planner - Whether local planner should use path output from global planner for its road centerline or use simple line connecting waypoints"),
        DeclareLaunchArgument('use_blend', default_value='True', description="Local planner - Whether or not to apply blending with adjacent candidate paths during planning"),
        DeclareLaunchArgument('cost_vis', default_value='final', description="Local planner - What type of cost to display on candidate paths: none | final | components | all"),
        DeclareLaunchArgument('cost_vis_text_size', default_value='2.0', description="Local planner - Cost vis text size"),
        DeclareLaunchArgument('ignore_coll_before_dist', default_value='0.0', description="Local planner - Distance before which collisions are ignored in local planner candidate paths."),

        # Local Potential field Planner
        DeclareLaunchArgument("pf_use_global_path", default_value="False"),
        DeclareLaunchArgument("pf_rate", default_value="10.0"),
        DeclareLaunchArgument("pf_obstacle_cost_thresh", default_value="50", description="threshold for making something an obstacle (0-100), primarily for use with segmentation grid"),
        DeclareLaunchArgument("pf_kp", default_value="10.0", description="attractive potential coeff"),
        DeclareLaunchArgument("pf_eta", default_value="7.5", description="repulsive potential coeff"),
        DeclareLaunchArgument("pf_cutoff_dist", default_value="40.0", description="obstacles farther than this are ignored"),
        DeclareLaunchArgument("pf_inner_cutoff_dist", default_value="0.0", description="obstacles closer than this are ignored"),
        DeclareLaunchArgument("pf_motion_model_res", default_value="0.25", description="step size for the motion model in meters"),

        # Local Planner - Dynamic Window Approach Planner
        DeclareLaunchArgument("dwa_model", default_value="ackermann", description="Motion model for the trajectory prediction - may be 'ackermann' for Ackermann-steered AGVs or 'synchro' for synchro drive robots."),
        DeclareLaunchArgument("dwa_wheelbase", default_value="2.72", description="AGV wheelbase. Used for trajectory prediction for the Ackermann motion model."),
        DeclareLaunchArgument("dwa_speed_lin_min", default_value="0.15", description="Minimum value for the linear variable (longitudinal speed for all models) in the prediction window."),
        DeclareLaunchArgument("dwa_speed_lin_max", default_value="4.0", description="Maximum value for the linear variable (longitudinal speed for all models) in the prediction window."),
        DeclareLaunchArgument("dwa_speed_lin_steps", default_value="10", description="Number of intervals to partition the linear variable range into. Higher values allow for finer control, but significantly impact performance."),
        DeclareLaunchArgument("dwa_accel_max", default_value="3.0", description="Maximum rate of change of the linear variable (longitudinal speed)."),
        DeclareLaunchArgument("dwa_speed_ang_min", default_value="-0.58", description="Minimum value for the angular variable (yaw rate for synchro drive, steering angle for Ackermann) in the prediction window."),
        DeclareLaunchArgument("dwa_speed_ang_max", default_value="0.58", description="Maximum value for the angular variable (yaw rate for synchro drive, steering angle for Ackermann) in the prediction window."),
        DeclareLaunchArgument("dwa_speed_ang_steps", default_value="40", description="Number of intervals to partition the angular variable range into. Higher values allow for finer control, but significantly impact performance."),
        DeclareLaunchArgument("dwa_ang_accel_max", default_value="4.0", description="Maximum rate of change of the angular variable (yaw acceleration for synchro drive, steering rate for Ackermann)"),
        DeclareLaunchArgument("dwa_horizon", default_value="adaptive", description="Time prediction horizon - may be `fixed` or `adaptive` for prediction windows proportional to the longitudinal speed of the AGV."),
        DeclareLaunchArgument("dwa_time_span_min", default_value="2.5", description="Minimum time span for the prediction window."),
        DeclareLaunchArgument("dwa_time_span_max", default_value="10.0", description="Maximum time span for the prediction window."),
        DeclareLaunchArgument("dwa_time_span_var", default_value="4.5", description="Size of the variable component of the prediction window."),
        DeclareLaunchArgument("dwa_time_span_gain", default_value="1.1", description="Longitudinal speed proportionality constant for the variable prediction window."),
        DeclareLaunchArgument("dwa_time_step_min", default_value="0.2", description="Minimum time step for motion prediction. Smaller values allow for finer control, but significantly impact performance."),
        DeclareLaunchArgument("dwa_w_cost_goal", default_value="1.0", description="Objective function goal distance cost term weight. Higher values will promote planned trajectories with final pose closer to the goal pose."),
        DeclareLaunchArgument("dwa_w_cost_head", default_value="0.001", description="Objective function heading deviation cost term weight. Higher values will promote planned trajectories with final pose oriented towards the goal."),
        DeclareLaunchArgument("dwa_thresh_obs", default_value="0", description="Occupancy threshold above which a grid cell is considered an obstacle."),
        DeclareLaunchArgument("dwa_collision_radius", default_value="2.25", description="AGV radius used for collision detection (centered in the odometry message frame)."),
        DeclareLaunchArgument("dwa_w_cost_obs", default_value="1.5", description="Objective function obstacle distance cost term weight."),
        DeclareLaunchArgument("dwa_w_cost_speed", default_value="0.0", description="Objective function terminal speed cost term weight. Higher values will promote planned trajectories with higher final velocity."),
        DeclareLaunchArgument("dwa_use_global_path", default_value="False", description="Whether or not to use the global path when selecting the goal waypoint and to evaluate the AGV deviation from it."),
        DeclareLaunchArgument("dwa_w_cost_path", default_value="0.0", description="Objective function global path deviation term weight. Higher values will promote planned trajectories closer to the global path."),
        DeclareLaunchArgument("dwa_use_segmentation", default_value="False", description="Whether or not to inflate the obstacle cost with the segmentation cost."),
        DeclareLaunchArgument("dwa_w_cost_seg", default_value="0.0", description="Objective function segmentation cost term weight. Higher values will promote planned trajectories which stay far from cells with higher cost."),
        DeclareLaunchArgument("dwa_w_cost_dev", default_value="0.75", description="Objective function deviation term weight. Higher values will promote planned trajectories with smaller deviations from the current one."),
        DeclareLaunchArgument("dwa_print_summary", default_value="False", description="Whether or not to print the results of each planner iteration to console. Has an impact on performance and should only be used for debugging purposes."),

        # Pure Pursuit Control
        DeclareLaunchArgument('vehicle_wheelbase', default_value='2.72', description="Pure pursuit controller - vehicle_wheelbase."),
        DeclareLaunchArgument('vehicle_max_steer_angle_degrees', default_value='30.0', description="Pure pursuit controller - max steer angle in degrees."),
        DeclareLaunchArgument('steering_coefficient', default_value='3.5', description="Pure pursuit controller - steering coefficient."),
        DeclareLaunchArgument('vehicle_speed', default_value='7.0', description="Pure pursuit controller - vehicle_speed m/s."),
        DeclareLaunchArgument('throttle_coefficient', default_value='1.0', description="Pure pursuit controller - scale factor for the commanded steering. l.t. 1.0 will make the acceleration less agressive, g.t. 1.0 will make it more agressive"),
        DeclareLaunchArgument('throttle_kp', default_value='0.1129', description="Throttle PID Control - proportional coeff for the PID speed controller"),
        DeclareLaunchArgument('throttle_ki', default_value='0.0', description="Throttle PID Control - integral coeff for the PID speed controller"),
        DeclareLaunchArgument('throttle_kd', default_value='0.0', description="Throttle PID Control - derivative coeff for the PID speed controller"),

        DeclareLaunchArgument('time_to_max_brake', default_value='4.0', description="Time in seconds to go from 0 to maximum braking"),
        DeclareLaunchArgument('time_to_max_throttle', default_value='3.0', description="Time in seconds to go from 0 to maximum throttle"),
        DeclareLaunchArgument('use_feed_forward', default_value='false', description="Set to true to use the feed-forward model in the PID throttle control"),
        DeclareLaunchArgument('ff_a0', default_value='0.2856', description="0th order coeff in the feed-forward model"),
        DeclareLaunchArgument('ff_a1', default_value='0.0321', description="1st order coeff in the feed-forward model"),
        DeclareLaunchArgument('ff_a2', default_value='0.0', description="2nd order coeff in the feed-forward model"),
        DeclareLaunchArgument('max_desired_lateral_g', default_value='0.75', description="Controller will limit the speed to try to keep the lateral g-forces under this amount. In fractional units of 9.806 m/s^2"),
        DeclareLaunchArgument('anti_windup_method', default_value='disabled', description="PID integral windup correction method:  'disabled' | 'reset_on_setpoint' | 'output_clamping'"),
        DeclareLaunchArgument('pid_output_max', default_value='1.0', description="Max value for PID output. Only active when anti_windup_method = 'output_clamping'"),
        DeclareLaunchArgument('pid_output_min', default_value='0.0', description="Min value for PID output. Only active when anti_windup_method = 'output_clamping'"),

        # Global Segmentation Grid
        DeclareLaunchArgument('global_grid_csv_path', default_value=os.path.join(get_package_share_directory('avt_341'), 'config', 'KRC_CP06_Scaled_Transposed.csv'), description="Path to CSV containing KRC global segmentation grid."),

        # MULTIPLE VEHICLES
        # ==============================================================================================================
        # Mission Manager
        DeclareLaunchArgument('follow_scale_x', default_value='15.0', description="Mission Manager - X scale distance between vehicles in formation in vehicle coordinates"),
        DeclareLaunchArgument('follow_scale_y', default_value='10.0', description="Mission Manager - Y Scale distance between vehicles in formation in vehicle coordinates"),
        DeclareLaunchArgument('mission_definition_file', default_value=os.path.join(get_package_share_directory('avt_341'), 'config', 'mission_points.csv'), description="Mission Manager - file describing mission reference points"),
        DeclareLaunchArgument('fsc_type', default_value='slow_down_leader', description="Mission Manager - Type of formation speed control (fsc) to use 'speed_up_follower' | 'slow_down_leader' | 'none'."),
        DeclareLaunchArgument('x_offset_on_path', default_value='False', description="Mission Manager - If true follower x-offset is applied along leader path length. Else x-offset is applied from snap-shot poses of leader."),
        DeclareLaunchArgument('formation_prune_gp', default_value='True', description="Mission Manager - If true prunes the formation controller generated follower global path to the closer point."),
        DeclareLaunchArgument('follow_goal_threshold', default_value='10.0', description="Mission Manager - Terminal goal threshold when to consider follow task done if termination method set to ALL_ARRIVE. Only checked once followed vehicle arrives at goal."),

        DeclareLaunchArgument('oof_threshold', default_value='15.0', description="Mission Manager - Distance threshold after which vehicle considered out of formation."),
        DeclareLaunchArgument('oof_const_term', default_value='0.3', description="Mission Manager - Initial speed factor subtraction if past threshold"),
        DeclareLaunchArgument('oof_lin_slope', default_value='0.03', description="Mission Manager - Slope in linear portion of out of formation speed control"),
        DeclareLaunchArgument('oof_mult', default_value='1.5', description="Mission Manager - Multiplier applied to vehicles out of formation"),
        DeclareLaunchArgument('formation_debug_visualize', default_value='False', description="Mission Manager - Debug visualize formation speed control."),
        DeclareLaunchArgument('offsets_from_leader', default_value='True', description="Mission Manager - If true, formation offsets calculated from lead vehicle. If false, offsets calculated from next vehicle in column formation."),
        DeclareLaunchArgument('follower_dist_break', default_value='10.0', description="Mission Manager - If follower global path distance is less than this amount, follower waits."),
        DeclareLaunchArgument('follower_dot_threshold', default_value='0.0', description="Mission Manager - Heading dot product threshold of heading vectors of follower and lead vehicle below which follower vehicle waits."),
        DeclareLaunchArgument('follower_dot_range', default_value='30.0', description="Mission Manager - Range within which to apply heading dot product filter."),
        DeclareLaunchArgument('vehicle_namespaces', default_value="['agv1', 'agv2', 'cgv1', 'cgv2']", description="Mission Manager - Vehicle namespaces to listen to for odometry callbacks."),

        DeclareLaunchArgument('toi_approach_dist', default_value='15.0', description="Mission Manager - Approach distance to target of interest before encircling."),
        DeclareLaunchArgument('toi_encircle_radius', default_value='10.0', description="Mission Manager - Encircle radius around target of interest."),
        DeclareLaunchArgument('toi_encircle_degrees', default_value='180.0', description="Mission Manager - Encircle degrees around target of interest."),
        DeclareLaunchArgument('toi_encircle_cw', default_value='True', description="Mission Manager - If encircle should be in clock-wise (cw) direction around target of interest."),

        # Formation Control
        DeclareLaunchArgument('use_leader_breadcrumbs', default_value='True', description="Formation Control - If true, follower vehicles will use leader breadcrumbs as global path (disables follower vehicles global planner). "
                                                                                          "If false, followers will use global planner to current formation location."),

        # Comm Node
        DeclareLaunchArgument('host', default_value='localhost', description="Communication Node - Hostname of the communication server"),
        DeclareLaunchArgument('port', default_value='9000', description="Communication Node - port number for connecting to the communication server"),
        DeclareLaunchArgument('disable_socket_comms', default_value='False', description="Communication Node - If true disables tcp socket communication."),
        DeclareLaunchArgument('broadcast_internal', default_value='False', description="Communication Node - If true, echos received messages from comm_messages topic subscription."),
        DeclareLaunchArgument('add_name_id_to_msg', default_value='True', description="Communication Node - If true, adds vehicle name and message count to broadcast messages."),
        DeclareLaunchArgument('verbose_comm_log', default_value='False', description="Communication Node - If true, comm node includes verbose logging."),

    ]
    vehicle_node_list = []
    for idx in range(MAX_VEHICLES):
        vehicle_node_list.append(
            GroupAction(condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > %d' % idx])), actions=[
                PushRosNamespace(
                    condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1 or ', LaunchConfiguration('namespace_single_vehicle')])),
                    namespace=ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx)),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc_list[idx],
                                 'frame_prefix': TernarySubstitution(Concat(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx), '/'),
                                                                     TextSubstitution(text=''),
                                                                     IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1'])))}]
                ),
                Node(
                    package='avt_341',
                    executable='avt_bot_state_publisher_node',
                    name='state_publisher',
                    parameters=[{'frame_prefix': TernarySubstitution(Concat(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx), '/'),
                                                                     TextSubstitution(text=''),
                                                                     IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1'])))}]
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_perception_node',
                    name='perception_node',
                    output='screen',
                    parameters=[{
                        'use_elevation': launch.substitutions.LaunchConfiguration('use_elevation'),
                        'slope_threshold': launch.substitutions.LaunchConfiguration('slope_threshold'),
                        'grid_height': launch.substitutions.LaunchConfiguration('grid_height'),
                        'grid_width': launch.substitutions.LaunchConfiguration('grid_width'),
                        'grid_llx': launch.substitutions.LaunchConfiguration('grid_llx'),
                        'grid_lly': launch.substitutions.LaunchConfiguration('grid_lly'),
                        'grid_res': launch.substitutions.LaunchConfiguration('grid_res'),
                        'overhead_clearance': 7.0,
                        'grid_dilate': launch.substitutions.LaunchConfiguration('grid_dilate'),
                        'grid_dilate_x': launch.substitutions.LaunchConfiguration('grid_dilate_x'),
                        'grid_dilate_y': launch.substitutions.LaunchConfiguration('grid_dilate_y'),
                        'grid_dilate_proportion': launch.substitutions.LaunchConfiguration('grid_dilate_proportion'),
                        'cull_lidar': launch.substitutions.LaunchConfiguration('cull_lidar'),
                        'cull_lidar_dist': launch.substitutions.LaunchConfiguration('cull_lidar_dist'),
                        'cull_lidar_dist_min': launch.substitutions.LaunchConfiguration('cull_lidar_dist_min'),
                        'warmup_time': 5.0,
                        'use_registered': launch.substitutions.LaunchConfiguration('use_registered'),
                        'display': display_type,
                        'stitch_lidar_points': launch.substitutions.LaunchConfiguration('stitch_lidar_points'),
                        'filter_highest_lidar': launch.substitutions.LaunchConfiguration('filter_highest_lidar'),
                        'perception_rate': launch.substitutions.LaunchConfiguration('perception_rate'),

                        'clear_method_max_point_age': launch.substitutions.LaunchConfiguration('clear_method_max_point_age'),
                        'clear_method_type': launch.substitutions.LaunchConfiguration('clear_method_type'),
                        'clear_method_visualize': launch.substitutions.LaunchConfiguration('clear_method_visualize'),
                        'clear_method_visualize_range': launch.substitutions.LaunchConfiguration('clear_method_visualize_range'),
                        'clear_method_raytrace_range': launch.substitutions.LaunchConfiguration('clear_method_raytrace_range'),
                        'clear_method_use_voxels': launch.substitutions.LaunchConfiguration('clear_method_use_voxels'),
                        'clear_method_voxel_height_min': launch.substitutions.LaunchConfiguration('clear_method_voxel_height_min'),
                        'clear_method_voxel_height_res': launch.substitutions.LaunchConfiguration('clear_method_voxel_height_res'),
                        'clear_method_immediate_clear_dilation': launch.substitutions.LaunchConfiguration('clear_method_immediate_clear_dilation'),
                        'clear_method_obs_filter_range': launch.substitutions.LaunchConfiguration('clear_method_obs_filter_range'),
                        'clear_method_sampled_threshold': launch.substitutions.LaunchConfiguration('clear_method_sampled_threshold')
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_control_node',
                    name='vehicle_control_node',
                    output='screen',
                    condition=LaunchConfigurationNotEquals('local_planner_method', 'dwa'),
                    parameters=[{
                        'vehicle_wheelbase': launch.substitutions.LaunchConfiguration('vehicle_wheelbase'),
                        'vehicle_max_steer_angle_degrees': launch.substitutions.LaunchConfiguration('vehicle_max_steer_angle_degrees'),
                        'steering_coefficient': launch.substitutions.LaunchConfiguration('steering_coefficient'),
                        'vehicle_speed': launch.substitutions.LaunchConfiguration('vehicle_speed'),
                        'throttle_coefficient': launch.substitutions.LaunchConfiguration('throttle_coefficient'),
                        'throttle_kp': launch.substitutions.LaunchConfiguration('throttle_kp'),
                        'throttle_ki': launch.substitutions.LaunchConfiguration('throttle_ki'),
                        'throttle_kd': launch.substitutions.LaunchConfiguration('throttle_kd'),
                        'time_to_max_brake': launch.substitutions.LaunchConfiguration('time_to_max_brake'),
                        'max_desired_lateral_g': launch.substitutions.LaunchConfiguration('max_desired_lateral_g')
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_speed_control_node',
                    name='vehicle_control_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'dwa'),
                    parameters=[{
                        'vehicle_max_steer_angle_degrees': launch.substitutions.LaunchConfiguration('vehicle_max_steer_angle_degrees'),
                        'steering_coefficient': launch.substitutions.LaunchConfiguration('steering_coefficient'),
                        'vehicle_speed': launch.substitutions.LaunchConfiguration('vehicle_speed'),
                        'throttle_coefficient': launch.substitutions.LaunchConfiguration('throttle_coefficient'),
                        'throttle_kp': launch.substitutions.LaunchConfiguration('throttle_kp'),
                        'throttle_ki': launch.substitutions.LaunchConfiguration('throttle_ki'),
                        'throttle_kd': launch.substitutions.LaunchConfiguration('throttle_kd'),
                        'use_feed_forward': launch.substitutions.LaunchConfiguration('use_feed_forward'),
                        'ff_a0': launch.substitutions.LaunchConfiguration('ff_a0'),
                        'ff_a1': launch.substitutions.LaunchConfiguration('ff_a1'),
                        'ff_a2': launch.substitutions.LaunchConfiguration('ff_a2'),
                        'time_to_max_brake': launch.substitutions.LaunchConfiguration('time_to_max_brake'),
                        'max_desired_lateral_g': launch.substitutions.LaunchConfiguration('max_desired_lateral_g'),
                        'anti_windup_method': launch.substitutions.LaunchConfiguration('anti_windup_method'),
                        'pid_output_max': launch.substitutions.LaunchConfiguration('pid_output_max'),
                        'pid_output_min': launch.substitutions.LaunchConfiguration('pid_output_min'),
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_global_path_node',
                    name='avt_341_global_path_node',
                    output='screen',
                    parameters=[{
                        'goal_dist': launch.substitutions.LaunchConfiguration('goal_dist'),
                        'global_lookahead': 75.0,
                        'shutdown_behavior': launch.substitutions.LaunchConfiguration('shutdown_behavior'),
                        'auto_active_on_new_waypoint': launch.substitutions.LaunchConfiguration('auto_active_on_new_waypoint'),
                        'display': display_type,
                        'debug_visualize': launch.substitutions.LaunchConfiguration('debug_visualize'),
                        'w_distance': launch.substitutions.LaunchConfiguration('w_distance'),
                        'w_occupancy': launch.substitutions.LaunchConfiguration('w_occupancy'),
                        'w_segmentation': launch.substitutions.LaunchConfiguration('w_segmentation'),
                        'search_diagonals': launch.substitutions.LaunchConfiguration('search_diagonals'),
                        'los_max_iterations': launch.substitutions.LaunchConfiguration('los_max_iterations'),
                        'los_break_on_first': launch.substitutions.LaunchConfiguration('los_break_on_first'),
                        '/waypoints_x': launch.substitutions.LaunchConfiguration('waypoints_x'),
                        '/waypoints_y': launch.substitutions.LaunchConfiguration('waypoints_y'),
                        '/is_empty_waypoints': launch.substitutions.LaunchConfiguration('is_empty_waypoints'),
                        'verbose_gp_log': launch.substitutions.LaunchConfiguration('verbose_gp_log'),
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_local_planner_node',
                    name='local_planner_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'rcc'),
                    parameters=[{
                        'path_look_ahead': launch.substitutions.LaunchConfiguration('path_look_ahead'),
                        'vehicle_width': launch.substitutions.LaunchConfiguration('vehicle_width'),
                        'num_paths': launch.substitutions.LaunchConfiguration('num_paths'),
                        'max_steer_angle': launch.substitutions.LaunchConfiguration('max_steer_angle'),
                        'output_path_step': 0.5,
                        'path_integration_step': 0.35,
                        'dilation_factor': launch.substitutions.LaunchConfiguration('dilation_factor'),
                        'w_c': launch.substitutions.LaunchConfiguration('w_c'),
                        'w_s': launch.substitutions.LaunchConfiguration('w_s'),
                        'w_d': launch.substitutions.LaunchConfiguration('w_d'),
                        'w_r': launch.substitutions.LaunchConfiguration('w_r'),
                        'w_t': launch.substitutions.LaunchConfiguration('w_t'),
                        'rate': 50.0,
                        'trim_path': True,
                        'use_global_path': launch.substitutions.LaunchConfiguration('use_global_path'),
                        'use_blend': launch.substitutions.LaunchConfiguration('use_blend'),
                        'cost_vis': launch.substitutions.LaunchConfiguration('cost_vis'),
                        'cost_vis_text_size': launch.substitutions.LaunchConfiguration('cost_vis_text_size'),
                        'ignore_coll_before_dist': launch.substitutions.LaunchConfiguration('ignore_coll_before_dist'),
                        'display': display_type,
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_dwa_planner_node',
                    name='local_dwa_planner_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'dwa'),
                    parameters=[{
                        'dwa_model': launch.substitutions.LaunchConfiguration('dwa_model'),
                        'dwa_wheelbase': launch.substitutions.LaunchConfiguration('dwa_wheelbase'),
                        'dwa_speed_lin_min': launch.substitutions.LaunchConfiguration('dwa_speed_lin_min'),
                        'dwa_speed_lin_max': launch.substitutions.LaunchConfiguration('dwa_speed_lin_max'),
                        'dwa_speed_lin_steps': launch.substitutions.LaunchConfiguration('dwa_speed_lin_steps'),
                        'dwa_accel_max': launch.substitutions.LaunchConfiguration('dwa_accel_max'),
                        'dwa_speed_ang_min': launch.substitutions.LaunchConfiguration('dwa_speed_ang_min'),
                        'dwa_speed_ang_max': launch.substitutions.LaunchConfiguration('dwa_speed_ang_max'),
                        'dwa_speed_ang_steps': launch.substitutions.LaunchConfiguration('dwa_speed_ang_steps'),
                        'dwa_ang_accel_max': launch.substitutions.LaunchConfiguration('dwa_ang_accel_max'),
                        'dwa_horizon': launch.substitutions.LaunchConfiguration('dwa_horizon'),
                        'dwa_time_span_min': launch.substitutions.LaunchConfiguration('dwa_time_span_min'),
                        'dwa_time_span_var': launch.substitutions.LaunchConfiguration('dwa_time_span_var'),
                        'dwa_time_span_gain': launch.substitutions.LaunchConfiguration('dwa_time_span_gain'),
                        'dwa_time_step_min': launch.substitutions.LaunchConfiguration('dwa_time_step_min'),
                        'dwa_w_cost_goal': launch.substitutions.LaunchConfiguration('dwa_w_cost_goal'),
                        'dwa_w_cost_head': launch.substitutions.LaunchConfiguration('dwa_w_cost_head'),
                        'dwa_thresh_obs': launch.substitutions.LaunchConfiguration('dwa_thresh_obs'),
                        'dwa_collision_radius': launch.substitutions.LaunchConfiguration('dwa_collision_radius'),
                        'dwa_w_cost_obs': launch.substitutions.LaunchConfiguration('dwa_w_cost_obs'),
                        'dwa_w_cost_speed': launch.substitutions.LaunchConfiguration('dwa_w_cost_speed'),
                        'dwa_use_global_path': launch.substitutions.LaunchConfiguration('dwa_use_global_path'),
                        'dwa_w_cost_path': launch.substitutions.LaunchConfiguration('dwa_w_cost_path'),
                        'dwa_use_segmentation': launch.substitutions.LaunchConfiguration('dwa_use_segmentation'),
                        'dwa_w_cost_seg': launch.substitutions.LaunchConfiguration('dwa_w_cost_seg'),
                        'dwa_w_cost_dev': launch.substitutions.LaunchConfiguration('dwa_w_cost_dev'),
                        'dwa_print_summary': launch.substitutions.LaunchConfiguration('dwa_print_summary'),
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_pf_planner_node',
                    name='local_pf_planner_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'pf'),
                    parameters=[{
                        'use_global_path': launch.substitutions.LaunchConfiguration('pf_use_global_path'),
                        'rate': launch.substitutions.LaunchConfiguration('pf_rate'),
                        'obstacle_cost_thresh': launch.substitutions.LaunchConfiguration('pf_obstacle_cost_thresh'),
                        'kp': launch.substitutions.LaunchConfiguration('pf_kp'),
                        'eta': launch.substitutions.LaunchConfiguration('pf_eta'),
                        'cutoff_dist': launch.substitutions.LaunchConfiguration('pf_cutoff_dist'),
                        'motion_model_res': launch.substitutions.LaunchConfiguration('pf_motion_model_res'),
                        'inner_cutoff_dist': launch.substitutions.LaunchConfiguration('pf_inner_cutoff_dist'),
                        'display': display_type
                    }],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_grid_compression_node',
                    name='grid_compression'),
                GroupAction(condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1'])), actions=[
                    Node(
                        package='avt_341',
                        executable='avt_341_mission_manager_node',
                        name='mission_manager_node',
                        output='screen',
                        parameters=[{
                            'name': ToUpper(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx)),
                            'mission_definition_file': launch.substitutions.LaunchConfiguration('mission_definition_file'),
                            'fsc_type': launch.substitutions.LaunchConfiguration('fsc_type'),
                            'follow_scale_x': launch.substitutions.LaunchConfiguration('follow_scale_x'),
                            'follow_scale_y': launch.substitutions.LaunchConfiguration('follow_scale_y'),
                            'oof_threshold': launch.substitutions.LaunchConfiguration('oof_threshold'),
                            'oof_const_term': launch.substitutions.LaunchConfiguration('oof_const_term'),
                            'oof_lin_slope': launch.substitutions.LaunchConfiguration('oof_lin_slope'),
                            'oof_mult': launch.substitutions.LaunchConfiguration('oof_mult'),
                            'formation_debug_visualize': launch.substitutions.LaunchConfiguration('formation_debug_visualize'),
                            'offsets_from_leader': launch.substitutions.LaunchConfiguration('offsets_from_leader'),
                            'follower_dist_break': launch.substitutions.LaunchConfiguration('follower_dist_break'),
                            'follower_dot_threshold': launch.substitutions.LaunchConfiguration('follower_dot_threshold'),
                            'veh_namespaces': launch.substitutions.LaunchConfiguration('vehicle_namespaces'),
                            'toi_approach_dist': launch.substitutions.LaunchConfiguration('toi_approach_dist'),
                            'toi_encircle_radius': launch.substitutions.LaunchConfiguration('toi_encircle_radius'),
                            'toi_encircle_degrees': launch.substitutions.LaunchConfiguration('toi_encircle_degrees'),
                            'toi_encircle_cw': launch.substitutions.LaunchConfiguration('toi_encircle_cw'),
                            'toi_goal_threshold': launch.substitutions.LaunchConfiguration('goal_dist'),
                            'add_name_id_to_msg': Invert(launch.substitutions.LaunchConfiguration('add_name_id_to_msg')),
                            'use_leader_breadcrumbs': launch.substitutions.LaunchConfiguration('use_leader_breadcrumbs'),
                            'x_offset_on_path': launch.substitutions.LaunchConfiguration('x_offset_on_path'),
                            'formation_prune_gp': launch.substitutions.LaunchConfiguration('formation_prune_gp')
                        }]
                    ),
                    Node(
                        package='avt_341',
                        executable='avt_341_comm_node',
                        name='comm_node',
                        output='screen',
                        parameters=[{
                            'host': launch.substitutions.LaunchConfiguration('host'),
                            'port': launch.substitutions.LaunchConfiguration('port'),
                            'name': ToUpper(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx)),
                            'disable_socket_comms': launch.substitutions.LaunchConfiguration('disable_socket_comms'),
                            'broadcast_internal': launch.substitutions.LaunchConfiguration('broadcast_internal'),
                            'add_name_id_to_msg': launch.substitutions.LaunchConfiguration('add_name_id_to_msg'),
                            'verbose_comm_log': launch.substitutions.LaunchConfiguration('verbose_comm_log')
                        }]
                    )
                ])
            ])
        )

    launch_description = LaunchDescription([
        *arg_list,
        OpaqueFunction(function=evaluate_waypoint_parameters),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        *vehicle_node_list,
        # DeclareLaunchArgument('rviz_config', default_value=rviz_config_single_vehicle, description='Full path to rviz config file'),
        # SetLaunchConfiguration('rviz_config', rviz_config_multi_vehicle, condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1']))),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(auto_launch_rviz),
            arguments=["-d", TernarySubstitution(true_val=TextSubstitution(text=rviz_config_multi_vehicle),
                                                 false_val=TernarySubstitution(true_val=TextSubstitution(text=rviz_config_two_vehicle),
                                                                               false_val=TextSubstitution(text=rviz_config_single_vehicle),
                                                                               condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1']))),
                                                 condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 2'])))]
        )
    ])

    return launch_description
