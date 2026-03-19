"""
Test launch for the MPC terminal heading feature.

Starts the following nodes:
  - mpc_terminal_heading_test_driver  : simulates vehicle kinematics from MPC commands,
                                        publishes odometry and an empty occupancy grid
  - veh_converter_node                : converts odometry → avt_341/veh (required by MPC)
  - obstacle_processor_node           : converts occupancy grid → obstacle clusters (required by MPC)
  - goal_point_processor_node         : selects goal point along global path; publishes
                                        mpc_goalPoint, mpc_desiredHeading, and mpc_final_heading
  - avt_341_global_path_node          : plans a global path from vehicle position to user goal
  - avt_341_mpc_planner_node          : MPC local planner (requires Julia runtime)
  - rviz2                             : visualizer with "2D Goal Pose" tool for interactive goal setting
  - static_transform_publisher        : publishes map → odom identity transform

Usage
-----
1. Build and source the workspace.
2. Launch:
       ros2 launch avt_341 test_terminal_heading.launch.py
3. In RViz, use the "2D Goal Pose" tool (press G) to click a goal position and
   drag to set the desired final heading.  The vehicle will drive to the goal
   and arrive aligned with the heading you specified.

Note: the launch file automatically passes ~/julia/julia-1.5.4/lib to the
MPC planner process via LD_LIBRARY_PATH.  Adjust JULIA_LIB_PATH at the top
of this file if your Julia installation is in a different location.
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

JULIA_LIB_PATH = os.path.expanduser('~/julia/julia-1.5.4/lib')


def load_params(path):
    """Load a flat key-value YAML file as a plain Python dict.

    The parameter YAML files in this project use a flat format (not the
    ros__parameters-nested format).  Passing the file path directly to
    Node(parameters=...) would fail because ROS 2 expects the nested
    format.  Loading via yaml and passing a dict works correctly.
    """
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')

    mpc_params = load_params(
        os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'mpc_local_planner.yaml'))
    global_planner_params = load_params(
        os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'global_planner.yaml'))
    rviz_config = os.path.join(avt_341_dir, 'rviz', 'mpc_terminal_heading_test.rviz')

    return LaunchDescription([

        # Static transform: map → odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),

        # Test driver: simulates vehicle motion and publishes occupancy grid
        Node(
            package='avt_341',
            executable='mpc_terminal_heading_test_driver.py',
            name='mpc_terminal_heading_test_driver',
            output='screen',
            parameters=[{
                'map_width_m': 100.0,
                'map_height_m': 100.0,
                'map_resolution_m': 1.0,
                'start_x': 10.0,
                'start_y': 10.0,
                'start_yaw_deg': 0.0,
            }]
        ),

        # Vehicle converter: odometry → avt_341/veh
        Node(
            package='avt_341',
            executable='veh_converter_node',
            name='avt_341_veh_converter_node',
            output='screen',
        ),

        # Obstacle processor: occupancy grid → obstacle clusters for MPC
        Node(
            package='avt_341',
            executable='obstacle_processor_node',
            name='obstacle_processor_node',
            output='screen',
            parameters=[mpc_params],
        ),

        # Global path planner: goal_pose → global_path
        Node(
            package='avt_341',
            executable='avt_341_global_path_node',
            name='avt_341_global_path_node',
            output='screen',
            parameters=[global_planner_params],
        ),

        # Goal point processor: global_path + veh → mpc_goalPoint + mpc_final_heading
        Node(
            package='avt_341',
            executable='goal_point_processor_node',
            name='goal_point_processor_node',
            output='screen',
            parameters=[mpc_params],
        ),

        # MPC planner: libjulia.so.1 must be visible at runtime.
        # JULIA_LIB_PATH is prepended to LD_LIBRARY_PATH for this process.
        Node(
            package='avt_341',
            executable='avt_341_mpc_planner_node',
            name='mpc_planner_node',
            output='screen',
            parameters=[mpc_params],
            additional_env={
                'LD_LIBRARY_PATH': JULIA_LIB_PATH + ':' + os.environ.get('LD_LIBRARY_PATH', '')
            },
        ),

        # RViz: "2D Goal Pose" tool publishes to avt_341/goal_pose
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
