import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')
    rviz_config = os.path.join(avt_341_dir, 'rviz', 'avt_341_ros2.rviz')

    return LaunchDescription([
        # The Global Planner
        Node(
            package='avt_341',
            executable='avt_341_global_path_node',
            name='global_path_node',
            output='screen',
            parameters=[{
                'use_fastmarching': True,
                'w_distance': 1.0,
                'w_occupancy': 1.0,
                'w_segmentation': 0.0,
                'goal_dist': 2.0,
                'verbose_gp_log': False
            }]
        ),
        
        # The Test Driver
        Node(
            package='avt_341',
            executable='global_path_test_driver.py',
            name='global_path_test_driver',
            output='screen',
            parameters=[{'verbose': False}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
        
        # Static Transform (map to odom/base_link if needed)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        )
    ])
