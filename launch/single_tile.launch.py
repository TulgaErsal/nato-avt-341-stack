import os

import launch
import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.conditions


def generate_launch_description():

    avt_341_package_dir = get_package_share_directory('avt_341')
    urdf = os.path.join(avt_341_package_dir, 'config', 'avt_bot.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    waypoints_file = os.path.join(
        avt_341_package_dir,
        'config',
        'krc_waypoints.yaml'
    )
    avt_341_dir = get_package_share_directory('avt_341')
    base_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'uab_perception.launch.py')),
        launch_arguments={'waypoints_file': waypoints_file,
                          'robot_description': robot_desc,
                          'use_sim_time': 'False',
                          'goal_dist': '7.0',
                          'path_look_ahead': '30.0',
                          'grid_width': '985.5',
                          'grid_height': '1848.0',
                          'grid_llx': '-400.0',
                          'grid_lly': '-1000.0',
                          'grid_res': '0.5',
                          'w_c': '0.2', # comfort
                          'w_s': '0.2', # static safety
                          'w_d': '0.2', # dynamic safety
                          'w_r': '0.2', # path deviation
                          'w_t': '1.0', # terrain (undocumented)
                          'stitch_lidar_points': 'False',
                          'vehicle_speed': '3.0',
                          'vehicle_width': '1.6',
                          'max_steer_angle': '0.8',
                          'max_desired_lateral_g': '3000.0',
                          'shutdown_behavior': '1',
                          'slope_threshold': '1.5',
                          'cull_lidar': 'True',
                          'cull_lidar_dist': '50.0',
                          'cost_vis': 'all'}.items()
    )

    launch_description = LaunchDescription([
        base_launch
    ])

    return launch_description