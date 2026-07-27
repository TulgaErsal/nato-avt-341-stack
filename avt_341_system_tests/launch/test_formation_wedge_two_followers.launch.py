"""
Two-follower WEDGE formation test.

Formation layout (right-vector convention: positive y_offset = RIGHT of leader):
  MRZR4: x_offset = -x_scale (behind), y_offset = -y_scale (left)
  MRZR2: x_offset = -x_scale (behind), y_offset = +y_scale (right)

Each follower's MPC stack runs in its own ROS2 namespace (mrzr2, mrzr4).
The test driver publishes all support topics into both namespaces.

Expected correct behavior:
  MRZR4 tracks the behind-left position of the leader.
  MRZR2 tracks the behind-right position of the leader.

Usage
-----
  ros2 launch avt_341 test_formation_wedge_two_followers.launch.py
  ros2 launch avt_341 test_formation_wedge_two_followers.launch.py \\
      leader_motion:=sine sine_yaw_rate_amp:=0.15 sine_period:=10.0
  ros2 launch avt_341 test_formation_wedge_two_followers.launch.py \\
      leader_motion:=sine sine_yaw_rate_amp:=0.15 sine_period:=10.0 sine_speed_amp:=1.5

Launch arguments
----------------
  leader_motion      straight | sine | straight_then_sine  (default: straight)
  leader_speed       Leader forward speed [m/s]            (default: 3.0)
  sine_speed_amp     Speed variation amplitude [m/s]       (default: 0.0)
  sine_yaw_rate_amp  Yaw rate amplitude [rad/s]            (default: 0.15)
  sine_period        Sine period [s]                       (default: 10.0)
  straight_duration  Straight phase before sine [s]        (default: 10.0)
  x_scale            Formation x-offset scale [m]          (default: 5.0)
  y_scale            Formation y-offset scale [m]          (default: 5.0)
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
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def _follower_stack(ns: str, mpc_params: dict) -> list:
    """Return the four nodes that make up one follower's MPC stack."""
    return [
        Node(
            package='avt_341',
            executable='veh_converter_node',
            name='avt_341_veh_converter_node',
            namespace=ns,
            output='screen',
        ),
        Node(
            package='avt_341',
            executable='obstacle_processor_node',
            name='obstacle_processor_node',
            namespace=ns,
            output='screen',
            parameters=[mpc_params],
        ),
        Node(
            package='avt_341',
            executable='goal_point_processor_node',
            name='goal_point_processor_node',
            namespace=ns,
            output='screen',
            parameters=[mpc_params],
        ),
        Node(
            package='avt_341',
            executable='avt_341_mpc_planner_node',
            name='mpc_planner_node',
            namespace=ns,
            output='screen',
            parameters=[mpc_params],
            additional_env={
                'LD_LIBRARY_PATH': JULIA_LIB_PATH + ':' + os.environ.get('LD_LIBRARY_PATH', '')
            },
        ),
    ]


def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')

    mpc_params  = load_params(
        os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'mpc_local_planner.yaml'))
    rviz_config = os.path.join(
        get_package_share_directory('avt_341_system_tests'), 'rviz', 'formation_wedge_two_follower_test.rviz')

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
        description='Yaw rate amplitude [rad/s]')
    sine_period_arg = DeclareLaunchArgument(
        'sine_period', default_value='10.0',
        description='Sine period [s]')
    straight_dur_arg = DeclareLaunchArgument(
        'straight_duration', default_value='10.0',
        description='Straight phase before sine [s]')
    x_scale_arg = DeclareLaunchArgument(
        'x_scale', default_value='5.0',
        description='Formation x-offset scale [m]')
    y_scale_arg = DeclareLaunchArgument(
        'y_scale', default_value='5.0',
        description='Formation y-offset scale [m]')

    nodes = [
        leader_motion_arg,
        leader_speed_arg,
        sine_speed_amp_arg,
        sine_amp_arg,
        sine_period_arg,
        straight_dur_arg,
        x_scale_arg,
        y_scale_arg,

        # Static transform: map -> odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        ),

        # Two-follower test driver
        Node(
            package='avt_341_system_tests',
            executable='formation_distance_two_follower_test_driver.py',
            name='formation_distance_two_follower_test_driver',
            output='screen',
            parameters=[{
                'leader_motion':     LaunchConfiguration('leader_motion'),
                'leader_speed':      LaunchConfiguration('leader_speed'),
                'sine_speed_amp':    LaunchConfiguration('sine_speed_amp'),
                'sine_yaw_rate_amp': LaunchConfiguration('sine_yaw_rate_amp'),
                'sine_period':       LaunchConfiguration('sine_period'),
                'straight_duration': LaunchConfiguration('straight_duration'),
                'x_scale':           LaunchConfiguration('x_scale'),
                'y_scale':           LaunchConfiguration('y_scale'),
            }],
        ),

        # RViz (reuses single-follower config for now)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ]

    # MRZR4 stack (namespace: mrzr4)
    nodes += _follower_stack('mrzr4', mpc_params)

    # MRZR2 stack (namespace: mrzr2)
    nodes += _follower_stack('mrzr2', mpc_params)

    return LaunchDescription(nodes)
