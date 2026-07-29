"""
Test launch for studying formation distance-keeping behavior.

Two simulated vehicles: a scripted leader and a follower controlled by the
MPC local planner.  The leader follows a prescribed trajectory (straight,
sine wave, or a combination).  The follower receives the leader's true
odometry and the formation target path is computed directly in the test
driver -- no mission manager or global path planner is needed.

Nodes started:
  formation_distance_test_driver -- simulates both vehicles; publishes
                                    leader/follower odometry, the formation
                                    global path, occupancy grids, and all
                                    support topics consumed by the stack.
  veh_converter_node             -- converts follower odometry to avt_341/veh
  obstacle_processor_node        -- converts the empty occupancy grid to
                                    obstacle clusters consumed by the MPC
  goal_point_processor_node      -- selects the MPC goal point and headings
                                    from the formation global path
  avt_341_mpc_planner_node       -- MPC local planner (requires Julia runtime)
  rviz2                          -- visualizer with formation-specific config
  static_transform_publisher     -- map -> odom identity transform

Usage
-----
1. Build and source the workspace.
2. Launch with defaults (column formation, straight leader, 3 m/s):
       ros2 launch avt_341_system_tests test_formation_distance.launch.py

   Sine-wave leader:
       ros2 launch avt_341_system_tests test_formation_distance.launch.py \\
           leader_motion:=sine sine_yaw_rate_amp:=0.15 sine_period:=10.0

   Wedge formation (adjust follower start to nominal formation position):
       ros2 launch avt_341_system_tests test_formation_distance.launch.py \\
           formation:=wedge start_x_follow:=-5.0 start_y_follow:=5.0

   Straight then sine:
       ros2 launch avt_341_system_tests test_formation_distance.launch.py \\
           leader_motion:=straight_then_sine straight_duration:=10.0

Launch arguments
----------------
  formation          column | wedge                   (default: column)
  leader_motion      straight | sine | straight_then_sine  (default: straight)
  leader_speed       Leader forward speed [m/s]       (default: 3.0)
  sine_yaw_rate_amp  Yaw rate amplitude for sine [rad/s]  (default: 0.15)
  sine_period        Period of sine wave [s]           (default: 10.0)
  straight_duration  Straight phase before sine [s]   (default: 10.0)
  x_scale            Formation x offset scale [m]     (default: 5.0)
  y_scale            Formation y offset scale [m]     (default: 5.0)
  start_x_follow     Follower initial x [m]           (default: -5.0)
  start_y_follow     Follower initial y [m]           (default:  0.0)
  formation_end_time Sim time [s] to end formation and return solo to origin; -1 = never (default: -1.0)

Note: the Julia library path is read from JULIA_LIB_PATH at the top of this
file.  Adjust if your Julia installation differs from ~/julia/julia-1.5.4.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

JULIA_LIB_PATH = os.path.expanduser('~/julia/julia-1.5.4/lib')


def load_params(path: str) -> dict:
    """Load a flat-format YAML parameter file as a plain dict."""
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341_nav')

    mpc_params = load_params(
        os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'mpc_local_planner.yaml'))
    rviz_config = os.path.join(
        get_package_share_directory('avt_341_system_tests'), 'rviz', 'formation_distance_test.rviz')

    # -- Launch arguments -----------------------------------------------
    formation_arg = DeclareLaunchArgument(
        'formation', default_value='column',
        description='Formation shape: column or wedge')
    leader_motion_arg = DeclareLaunchArgument(
        'leader_motion', default_value='straight',
        description='Leader trajectory: straight, sine, or straight_then_sine')
    leader_speed_arg = DeclareLaunchArgument(
        'leader_speed', default_value='3.0',
        description='Leader forward speed [m/s]')
    sine_speed_amp_arg = DeclareLaunchArgument(
        'sine_speed_amp', default_value='0.0',
        description='Speed variation amplitude [m/s]; 0 = constant speed')
    sine_amp_arg = DeclareLaunchArgument(
        'sine_yaw_rate_amp', default_value='0.15',
        description='Yaw rate amplitude of the sine wave [rad/s]')
    sine_period_arg = DeclareLaunchArgument(
        'sine_period', default_value='10.0',
        description='Period of the sine-wave trajectory [s]')
    straight_dur_arg = DeclareLaunchArgument(
        'straight_duration', default_value='10.0',
        description='Duration of the straight phase before switching to sine [s]')
    x_scale_arg = DeclareLaunchArgument(
        'x_scale', default_value='5.0',
        description='Formation x-offset scale (behind leader) [m]')
    y_scale_arg = DeclareLaunchArgument(
        'y_scale', default_value='5.0',
        description='Formation y-offset scale (lateral, wedge only) [m]')
    start_x_follow_arg = DeclareLaunchArgument(
        'start_x_follow', default_value='-5.0',
        description='Follower initial x position [m]')
    start_y_follow_arg = DeclareLaunchArgument(
        'start_y_follow', default_value='0.0',
        description='Follower initial y position [m]')
    formation_end_time_arg = DeclareLaunchArgument(
        'formation_end_time', default_value='-1.0',
        description='Sim time [s] at which formation ends and ego drives solo to origin; -1 = never')

    return LaunchDescription([
        formation_arg,
        leader_motion_arg,
        leader_speed_arg,
        sine_speed_amp_arg,
        sine_amp_arg,
        sine_period_arg,
        straight_dur_arg,
        x_scale_arg,
        y_scale_arg,
        start_x_follow_arg,
        start_y_follow_arg,
        formation_end_time_arg,

        # Static transform: map -> odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        ),

        # Formation distance test driver: simulates both vehicles and
        # publishes the formation global path, occupancy grids, and all
        # support topics required by the downstream stack nodes.
        Node(
            package='avt_341_system_tests',
            executable='formation_distance_test_driver.py',
            name='formation_distance_test_driver',
            output='screen',
            parameters=[{
                'formation':          LaunchConfiguration('formation'),
                'leader_motion':      LaunchConfiguration('leader_motion'),
                'leader_speed':       LaunchConfiguration('leader_speed'),
                'sine_speed_amp':     LaunchConfiguration('sine_speed_amp'),
                'sine_yaw_rate_amp':  LaunchConfiguration('sine_yaw_rate_amp'),
                'sine_period':        LaunchConfiguration('sine_period'),
                'straight_duration':  LaunchConfiguration('straight_duration'),
                'x_scale':            LaunchConfiguration('x_scale'),
                'y_scale':            LaunchConfiguration('y_scale'),
                'start_x_follow':       LaunchConfiguration('start_x_follow'),
                'start_y_follow':       LaunchConfiguration('start_y_follow'),
                'formation_end_time':   LaunchConfiguration('formation_end_time'),
            }],
        ),

        # Vehicle converter: follower odometry -> avt_341/veh
        Node(
            package='avt_341_nav',
            executable='veh_converter_node',
            name='avt_341_veh_converter_node',
            output='screen',
        ),

        # Obstacle processor: empty occupancy grid -> empty obstacle clusters
        Node(
            package='avt_341_nav',
            executable='obstacle_processor_node',
            name='obstacle_processor_node',
            output='screen',
            parameters=[mpc_params],
        ),

        # Goal point processor: formation global path + veh ->
        #   mpc_goalPoint, mpc_desiredHeading, mpc_final_heading
        Node(
            package='avt_341_nav',
            executable='goal_point_processor_node',
            name='goal_point_processor_node',
            output='screen',
            parameters=[mpc_params],
        ),

        # MPC local planner: requires libjulia.so.1 at runtime.
        Node(
            package='avt_341_nav',
            executable='mpc_planner_node',
            name='mpc_planner_node',
            output='screen',
            parameters=[mpc_params],
            additional_env={
                'LD_LIBRARY_PATH': JULIA_LIB_PATH + ':' + os.environ.get('LD_LIBRARY_PATH', '')
            },
        ),

        # RViz: formation-specific configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
