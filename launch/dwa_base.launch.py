import os

import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


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

    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_launch_rviz = LaunchConfiguration("auto_launch_rviz")
    display_type = LaunchConfiguration('display_type')
    waypoints_file = LaunchConfiguration('waypoints_file')
    robot_description = LaunchConfiguration('robot_description')

    rviz_config_path = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_ros2.rviz')

    launch_description = LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('auto_launch_rviz', default_value='True', description="Automatically launch rviz display window"),
        DeclareLaunchArgument('display_type', default_value='rviz', description="Type of display method to use. Values = [rviz, image]"),
        DeclareLaunchArgument('waypoints_file', default_value=os.path.join(get_package_share_directory('avt_341'), 'config', 'no_waypoints.yaml'), description="Path to waypoint file to use"),
        DeclareLaunchArgument('robot_description', description="URDF robot description contents"),

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
        DeclareLaunchArgument('cull_lidar_dist', default_value='90.0', description="Elevation grid - Distance used to cull lidar points"),
        DeclareLaunchArgument('use_registered', default_value='True', description="Elevation grid - If true, assumes lidar points are in world coordinates. Else assumes in robot odom coordinates."),
        DeclareLaunchArgument('stitch_lidar_points', default_value='True', description="Elevation grid - If true, lidar scans will be stitched together. Else, each point cloud 2 message will be independent and the grid will be cleared between messages."),
        DeclareLaunchArgument('filter_highest_lidar', default_value='False', description="Elevation grid - If true, the highest point in each cell will be ignored and the second highest will be used for the slope calculations. If false, the highest point will be used."),
        DeclareLaunchArgument('time_register_window', default_value='0.05', description="Maximum window of time between odom and point cloud time stamps"),
        DeclareLaunchArgument('max_point_age', default_value='0.0', description="Lifetime of a point before it is cleared"),

        # Global Planner
        DeclareLaunchArgument('goal_dist', default_value='5.0', description="Global planner - Lookahead threshold within which next waypoint selected."),
        DeclareLaunchArgument('debug_visualize', default_value='False', description="Global planner - Enables debug visualization of global planner."),
        DeclareLaunchArgument('w_distance', default_value='1.0', description="Global planner - Weight for distance cost."),
        DeclareLaunchArgument('w_occupancy', default_value='1.0', description="Global planner - Weight for occupancy cost."),
        DeclareLaunchArgument('w_segmentation', default_value='1.0', description="Global planner - Weight for segmentation cost."),
        DeclareLaunchArgument('search_diagonals', default_value='False', description="Global planner - If true, grid search includes adjacent cells that are diagonal. If not, only adjacent cells that are horizontal or vertical are considered."),
        DeclareLaunchArgument('los_max_iterations', default_value='1', description="Global planner - Number of iterations to apply line of sight post smoothing. Applying post smoothing multiple times can help remove excessive control points that are added due to jagged obstacle edges."),
        DeclareLaunchArgument('los_break_on_first', default_value='True', description="Global planner - If true, line of sight post-smoothing only considers first break on line of sight. If false, finds the last line of sight connection."),

        # Local Dynamic Window Approach Planner
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

        OpaqueFunction(function=evaluate_waypoint_parameters),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        Node(
            package='avt_341',
            executable='avt_bot_state_publisher_node',
            name='state_publisher'),
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
                'warmup_time': 5.0,
                'use_registered': launch.substitutions.LaunchConfiguration('use_registered'),
                'display': display_type,
                'stitch_lidar_points': launch.substitutions.LaunchConfiguration('stitch_lidar_points'),
                'filter_highest_lidar': launch.substitutions.LaunchConfiguration('filter_highest_lidar')
            }],
        ),
        Node(
            package='avt_341',
            executable='avt_341_speed_control_node',
            name='vehicle_control_node',
            output='screen',
            parameters=[{
                'vehicle_max_steer_angle_degrees': launch.substitutions.LaunchConfiguration('vehicle_max_steer_angle_degrees'),
                'steering_coefficient': launch.substitutions.LaunchConfiguration('steering_coefficient'),
                'vehicle_speed': launch.substitutions.LaunchConfiguration('vehicle_speed'),
                'throttle_coefficient': launch.substitutions.LaunchConfiguration('throttle_coefficient'),
                'throttle_kp': launch.substitutions.LaunchConfiguration('throttle_kp'),
                'throttle_ki': launch.substitutions.LaunchConfiguration('throttle_ki'),
                'throttle_kd': launch.substitutions.LaunchConfiguration('throttle_kd'),
                'time_to_max_brake': launch.substitutions.LaunchConfiguration('time_to_max_brake'),
                'max_desired_lateral_g': launch.substitutions.LaunchConfiguration('max_desired_lateral_g'),
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
                'shutdown_behavior': 2,
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
            }],
        ),

        Node(
            package='avt_341',
            executable='avt_341_dwa_planner_node',
            name='local_dwa_planner_node',
            output='screen',
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=launch.conditions.IfCondition(auto_launch_rviz),
            arguments=["-d", rviz_config_path])
    ])

    return launch_description
