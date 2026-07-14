"""
Test launch for the MPC adaptive prediction-horizon ("final time as a design
variable") feature.

Starts the following nodes:
  final_time_mpc_test_driver.py -- simulates vehicle kinematics from MPC
                                    commands (3DOF Pacejka bicycle model,
                                    identical to the follower model in
                                    formation_distance_test_driver.py),
                                    generates a random obstacle field, and
                                    drives the scenario end-to-end with no
                                    user interaction required
  veh_converter_node            -- converts odometry -> avt_341/veh
  obstacle_processor_node        -- converts occupancy grid -> obstacle
                                    clusters for the MPC
  avt_341_global_path_node       -- plans a global path from start to goal
  goal_point_processor_node      -- selects the MPC goal point along the
                                    global path
  avt_341_mpc_planner_node        -- MPC local planner with the adaptive
                                    prediction horizon enabled (requires the
                                    Julia runtime)
  rviz2                          -- visualizer, showing obstacles, occupancy
                                    grid, obstacles culled to the MPC,
                                    global path, MPC goal point and MPC path
  Xvfb + ffmpeg                  -- (when record:=True, the default) render
                                    rviz to a virtual display and record it
                                    to an MP4 so the scenario can be watched
                                    from a launch-and-forget SSH session

The scenario needs no interactive input: the driver publishes a one-time
goal_pose once the stack is up, then reports MPC solve-time statistics and
the percentage of optimal solves to the terminal when the vehicle reaches
the goal (or after scenario_timeout_s, whichever comes first).  The driver
process exiting is the signal used to shut the whole launch down, which
finalizes the recording.

Usage
-----
    ros2 launch avt_341 test_final_time_mpc.launch.py

Useful tuning knobs (see launch arguments below for the full list):
    random_seed:=123 num_obstacles:=15 obstacle_min_size_m:=2.0 \
        obstacle_max_size_m:=6.0 mpc_max_speed:=5.0

To compare against the fixed-horizon baseline:
    ros2 launch avt_341 test_final_time_mpc.launch.py use_adaptive_prediction_horizon:=False

To run with a real display instead of recording headlessly:
    ros2 launch avt_341 test_final_time_mpc.launch.py record:=False

Note: the launch file automatically passes ~/julia/julia-1.5.4/lib to the
MPC planner process via LD_LIBRARY_PATH.  Adjust JULIA_LIB_PATH at the top
of this file if your Julia installation is in a different location.
"""

import os
from datetime import datetime

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                             GroupAction, LogInfo, RegisterEventHandler,
                             TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
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
    rviz_config = os.path.join(avt_341_dir, 'rviz', 'final_time_mpc_test.rviz')

    # Overridden at runtime via launch arguments so the adaptive-horizon
    # feature and the obstacle field are both tunable from the command line.
    mpc_params.update({
        'use_adaptive_prediction_horizon': LaunchConfiguration('use_adaptive_prediction_horizon'),
        'min_prediction_horizon_distance': LaunchConfiguration('min_prediction_horizon_distance'),
        'prediction_horizon_time_max':      LaunchConfiguration('prediction_horizon_time_max'),
        'w_prediction_horizon_anchor':       LaunchConfiguration('w_prediction_horizon_anchor'),
        'max_speed':                          LaunchConfiguration('mpc_max_speed'),
    })
    global_planner_params.update({
        'goal_dist': LaunchConfiguration('goal_dist_threshold'),
    })

    # Output video path, computed once at launch-description generation time.
    video_dir = os.path.expanduser(os.path.join('~', 'avt_341_data'))
    os.makedirs(video_dir, exist_ok=True)
    timestamp = datetime.now().strftime('%y%m%d_%H%M%S')
    video_path = os.path.join(video_dir, f'{timestamp}_final_time_mpc.mp4')

    launch_arg_dict = {
        # Obstacle field tuning knobs
        'random_seed':          ['42',   'RNG seed for obstacle placement (< 0 = non-deterministic)'],
        'num_obstacles':        ['10',   'Number of randomly placed square obstacles'],
        'obstacle_min_size_m':  ['3.0',  'Minimum obstacle side length [m]'],
        'obstacle_max_size_m':  ['7.0',  'Maximum obstacle side length [m]'],
        'obstacle_keepout_m':   ['6.0',  'Keep-out radius [m] around the start and goal positions'],
        'mpc_max_speed':        ['4.0',  'Maximum speed handed to the MPC planner [m/s]'],

        # Scenario geometry
        'map_width_m':          ['120.0', 'Occupancy grid width [m]'],
        'map_height_m':         ['120.0', 'Occupancy grid height [m]'],
        'start_x':               ['10.0',  'Vehicle start X position [m]'],
        'start_y':               ['10.0',  'Vehicle start Y position [m]'],
        'goal_x':                 ['110.0', 'Goal X position [m]'],
        'goal_y':                 ['110.0', 'Goal Y position [m]'],
        'goal_dist_threshold':    ['4.0',   'Distance [m] within which the goal is considered reached'],
        'scenario_timeout_s':      ['200.0', 'Scenario is stopped and reported as timed out after this many seconds'],
        'post_arrival_settle_s':    ['3.0',   'Seconds to keep recording after the goal is reached'],

        # Final-time-as-design-variable feature knobs
        'use_adaptive_prediction_horizon': ['True', 'Enable the MPC final time (tf) as a free design variable'],
        'min_prediction_horizon_distance': ['8.0',  'Minimum MPC prediction horizon distance [m]'],
        'prediction_horizon_time_max':     ['10.0', 'Upper bound on the adaptive prediction horizon [s]'],
        'w_prediction_horizon_anchor':      ['5.0',  'Weight pulling the adaptive horizon back to the nominal value'],

        # Recording
        'record':        ['True', 'Record the rviz session headlessly to an MP4 via Xvfb + ffmpeg'],
        'launch_rviz':    ['True', 'Launch rviz2 at all'],
        'video_width':     ['1600', 'Recording width [px]'],
        'video_height':     ['900',  'Recording height [px]'],
        'display_num':       [':99', 'X display number used for the virtual framebuffer'],
    }
    launch_args = [DeclareLaunchArgument(name, default_value=default, description=desc)
                   for name, [default, desc] in launch_arg_dict.items()]

    record = LaunchConfiguration('record')
    launch_rviz = LaunchConfiguration('launch_rviz')
    display_num = LaunchConfiguration('display_num')
    video_width = LaunchConfiguration('video_width')
    video_height = LaunchConfiguration('video_height')

    driver_node = Node(
        package='avt_341',
        executable='final_time_mpc_test_driver.py',
        name='final_time_mpc_test_driver',
        output='screen',
        parameters=[{
            'map_width_m':          LaunchConfiguration('map_width_m'),
            'map_height_m':         LaunchConfiguration('map_height_m'),
            'map_resolution_m':     1.0,
            'start_x':               LaunchConfiguration('start_x'),
            'start_y':               LaunchConfiguration('start_y'),
            'start_yaw_deg':          45.0,
            'goal_x':                 LaunchConfiguration('goal_x'),
            'goal_y':                 LaunchConfiguration('goal_y'),
            'obstacle_keepout_m':      LaunchConfiguration('obstacle_keepout_m'),
            'num_obstacles':            LaunchConfiguration('num_obstacles'),
            'obstacle_min_size_m':       LaunchConfiguration('obstacle_min_size_m'),
            'obstacle_max_size_m':        LaunchConfiguration('obstacle_max_size_m'),
            'random_seed':                  LaunchConfiguration('random_seed'),
            'mpc_max_speed':                 LaunchConfiguration('mpc_max_speed'),
            'scenario_timeout_s':              LaunchConfiguration('scenario_timeout_s'),
            'post_arrival_settle_s':             LaunchConfiguration('post_arrival_settle_s'),
        }],
    )

    nodes = [
        # Static transform: map -> odom (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        ),

        driver_node,

        # Vehicle converter: odometry -> avt_341/veh
        Node(
            package='avt_341',
            executable='veh_converter_node',
            name='avt_341_veh_converter_node',
            output='screen',
        ),

        # Obstacle processor: occupancy grid -> obstacle clusters for MPC
        Node(
            package='avt_341',
            executable='obstacle_processor_node',
            name='obstacle_processor_node',
            output='screen',
            parameters=[mpc_params],
        ),

        # Global path planner: goal_pose -> global_path
        Node(
            package='avt_341',
            executable='avt_341_global_path_node',
            name='avt_341_global_path_node',
            output='screen',
            parameters=[global_planner_params],
        ),

        # Goal point processor: global_path + veh -> mpc_goalPoint
        Node(
            package='avt_341',
            executable='goal_point_processor_node',
            name='goal_point_processor_node',
            output='screen',
            parameters=[mpc_params],
        ),

        # MPC planner: libjulia.so.1 must be visible at runtime.
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

        # Recording pipeline: virtual display + rviz + screen capture.
        GroupAction(condition=IfCondition(launch_rviz), actions=[
            GroupAction(condition=IfCondition(record), actions=[
                LogInfo(msg=f'Recording rviz session to {video_path}'),

                # Virtual framebuffer rviz renders into.
                ExecuteProcess(
                    cmd=['Xvfb', display_num, '-screen', '0',
                         [video_width, 'x', video_height, 'x24']],
                    output='screen',
                ),

                # Give Xvfb a moment to come up before starting rviz on it.
                TimerAction(period=2.0, actions=[
                    Node(
                        package='rviz2',
                        executable='rviz2',
                        name='rviz2',
                        arguments=['-d', rviz_config],
                        output='screen',
                        additional_env={
                            'DISPLAY': display_num,
                            'LIBGL_ALWAYS_SOFTWARE': '1',
                        },
                    ),
                ]),

                # Give rviz a moment to render its first frame before capturing.
                TimerAction(period=5.0, actions=[
                    ExecuteProcess(
                        cmd=['ffmpeg', '-y',
                             '-f', 'x11grab',
                             '-video_size', [video_width, 'x', video_height],
                             '-framerate', '30',
                             '-i', display_num,
                             '-pix_fmt', 'yuv420p',
                             '-vcodec', 'libx264',
                             '-preset', 'veryfast',
                             video_path],
                        output='screen',
                    ),
                ]),
            ]),
            GroupAction(condition=UnlessCondition(record), actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config],
                    output='screen',
                ),
            ]),
        ]),
    ]

    shutdown_on_driver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=driver_node,
            on_exit=[
                LogInfo(msg='Scenario driver exited; shutting the rest of the launch down.'),
                EmitEvent(event=Shutdown(reason='final_time_mpc_test_driver finished')),
            ],
        )
    )

    return LaunchDescription([
        *launch_args,
        *nodes,
        shutdown_on_driver_exit,
    ])
