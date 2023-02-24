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
        'single_tile_waypoints.yaml'
    )
    avt_341_dir = get_package_share_directory('avt_341')
    base_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'uab_perception.launch.py')),
        launch_arguments={'waypoints_file': waypoints_file,
                          'robot_description': robot_desc,
                          'use_sim_time': 'False',
                          'goal_dist': '7.0',
                          'path_look_ahead': '15.0',
                          'grid_width': '255.0',
                          'grid_height': '80.0',
                          'grid_llx': '-250.0',
                          'grid_lly': '-40.0',
                          'grid_res': '0.5',
                          'w_c': '0.2', # comfort
                          'w_s': '0.2', # static safety
                          'w_d': '0.2', # dynamic safety
                          'w_r': '0.2', # path deviation
                          'w_t': '1.0', # terrain (undocumented)
                          'stitch_lidar_points': 'False',
                          'use_registered': 'False',
                          'slope_threshold': '1.5',
                          'vehicle_width': '5.0',
                          'steering_coefficient': '15.0',
                          'vehicle_max_steer_angle_degrees': '38.7',
                          'vehicle_wheelbase': '3.3',
                          'cull_lidar': 'True',
                          'cull_lidar_dist': '50.0',
                          'cost_vis': 'all'}.items()
    )

    launch_description = LaunchDescription([
        base_launch
    ])

    return launch_description