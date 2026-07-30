"""Minimal example: the nav stack driving a mock simulated vehicle.

sim_test_node provides a simple motion model, odometry and a synthetic lidar
cloud. All optional stack features are disabled; the rcc local planner drives
toward the waypoints in env_data/example/example_waypoints.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

VEHICLE_ID = 'veh1'


def generate_launch_description():

    bringup_dir = get_package_share_directory('avt_341_bringup').replace('\\', '/')

    # The mock simulation publishes no tf, so the example provides the
    # map -> odom -> <vid>/base_link chain itself; robot_state_publisher
    # (spawned by base.launch.py) completes it with <vid>/base_link -> <vid>/lidar.
    sim_group = GroupAction([
        PushRosNamespace(VEHICLE_ID),
        Node(
            package='avt_341_nav',
            executable='sim_test_node',
            name='sim_test_node',
            output='screen',
        ),
        Node(
            package='avt_341_nav',
            executable='odom_to_tf_publisher_node',
            name='odom_to_tf_publisher',
            output='screen',
            parameters=[{'frame_prefix': f'{VEHICLE_ID}/', 'publish_map_to_odom': True}],
        ),
        # Bridge to the unprefixed base_link frame that sim_test_node uses as
        # the odometry child frame (same pattern as the krc launch).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_publisher',
            arguments=['0', '0', '0', '0', '0', '0', f'{VEHICLE_ID}/base_link', 'base_link'],
        ),
    ])

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'base.launch.py')),
        launch_arguments={
            'vehicle_ids':                  f"['{VEHICLE_ID}']",
            'robot_description_files':      f"['{bringup_dir}/urdf/avt_bot.urdf']",
            'ros_param_files':              f"['{bringup_dir}/parameters/example_ros_params.yaml']",
            'rviz_config':                  f'{bringup_dir}/rviz/avt_341_example.rviz',
            'use_sim_time':                 'False',
            'use_lidar_obstacle_detector':  'False',
            'use_dual_costmaps':            'False',
            'use_perception_rms':           'False',
            'use_uab_perception':           'False',
            'use_obj_detector':             'False',
            'use_object_tracker':           'False',
            'use_data_acquisition':         'False',
        }.items()
    )

    return LaunchDescription([sim_group, base_launch])
