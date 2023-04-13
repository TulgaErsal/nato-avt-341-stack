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
                          'grid_width': '985.5',
                          'grid_height': '1848.0',
                          'grid_llx': '-400.0',
                          'grid_lly': '-1000.0',
                          'grid_res': '0.5',
                          'grid_dilate': 'True',
                          'grid_dilate_x': '1.0',
                          'grid_dilate_y': '1.0',
                          'max_point_age': '1.0', # todo: swap to ray trace grid clearing
                          'slope_threshold': '0.4',
                          'path_look_ahead': '15.0',
                          'vehicle_speed': '3.0',
                          'goal_dist': '2.0',
                          'vehicle_width': '1.6',
                          'max_steer_angle': '0.8',
                          'max_desired_lateral_g': '3000.0',
                          'shutdown_behavior': '1',
                          'stitch_lidar_points': 'False',
                          'use_registered': 'False',
                          'cost_vis': 'all',
                          'w_t': '0.5', # terrain weight for local planner
                          }.items()
    )

    launch_description = LaunchDescription([
        base_launch
    ])

    return launch_description