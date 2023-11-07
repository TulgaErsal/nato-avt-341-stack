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
                        #   'w_segmentation': '1.0', # terrain weight for global planner
                        #   'w_occupancy': '1.0',
                        #   'w_distance': '1.0',
                          'debug_visualize': 'True', # global planner debugging
                          
                        #   spine planner params
                          'cost_vis': 'all',
                          'w_t': '0.5', # terrain weight for local planner
                          'w_d': '0.5',
                          'w_s': '0.5',
                          'w_r': '0.5',
                          'w_c': '0.5',
                          
                        #   DWA planner params
                        #   'dwa_speed_lin_min': '4.0',
                        #   'dwa_speed_lin_max': '10.0',
                        #   'dwa_accel_max': '4.0',
                        #   'dwa_speed_ang_min': '-0.785',
                        #   'dwa_speed_ang_max': '0.785',
                        #   'dwa_ang_accel_max': '10.0',
                        #   'dwa_time_span_min': '4.2',
                        #   'dwa_time_span_max': '10.0',
                        #   'dwa_time_span_var': '4.5',
                        #   'dwa_time_span_gain': '1.1',
                        #   'dwa_w_cost_goal': '1.0',
                        #   'dwa_w_cost_obs': '1.5',
                        #   'dwa_w_cost_speed': '1.0',
                        #   'dwa_w_cost_dev': '0.2',
                        #   'dwa_use_global_path': 'True',
                        #   'dwa_w_cost_path': '0.4',
                        #   'dwa_collision_radius': '1.5',
                          }.items()
    )

    launch_description = LaunchDescription([
        base_launch
    ])

    return launch_description