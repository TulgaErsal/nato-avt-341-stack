import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve and expand the bag file path
    bag_file = LaunchConfiguration('bag_file').perform(context)
    bag_file = os.path.expanduser(bag_file)
    
    # Retrieve parameter paths
    avt_341_dir = get_package_share_directory('avt_341')
    tracking_params_path = LaunchConfiguration('tracking_params').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)

    # Load YAML parameters manually to avoid ROS 2's strict YAML format requirement
    # (The provided YAML files are simple key-value pairs, which ROS 2's direct parameter file loading doesn't like)
    try:
        with open(tracking_params_path, 'r') as f:
            tracking_params = yaml.safe_load(f)
            # ROS 2 is strict about types. Ensure 'tracker_target_class' is a string.
            if tracking_params and 'tracker_target_class' in tracking_params:
                tracking_params['tracker_target_class'] = str(tracking_params['tracker_target_class'])
            
            # Force bag-specific frames to match the TF tree in the bag
            # We use 'odom' as world frame (found in bag strings)
            # and 'flir_optical' as camera frame (as requested by user)
            tracking_params['world_frame'] = 'odom'
            tracking_params['camera_frame'] = 'flir_optical'
    except Exception as e:
        print(f"Error loading tracking parameters: {e}")
        tracking_params = {}

    # Define Nodes and Processes
    
    # 1. Play the rosbag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--clock', '-l',
             '--remap', '/mrzr2/front_camera/camera_info:=/flir_camera/camera_info',
             '/mrzr2/front_camera/image:=/flir_camera/image_raw',
             '/mrzr2/front_camera/detections_2d:=/mrzr/feda_detector/detections/vision',
             '/mrzr2/avt_341/points:=/ouster/points'],
        output='screen'
    )

    # 2. Object Tracking Node
    tracking_node = Node(
        package='avt_341',
        executable='avt_341_object_tracking_node',
        name='object_tracking_node',
        output='screen',
        parameters=[tracking_params, {'use_sim_time': True}],
        remappings=[
            ('camera_info', '/flir_camera/camera_info'),
            ('image', '/flir_camera/image_raw'),
            ('detection_2d', '/mrzr/feda_detector/detections/vision'),
            ('points/input', '/ouster/points'),
            ('points/fov', 'object_tracking/points/fov'),
            ('points/roi', 'object_tracking/points/roi'),
            ('points/ground', 'object_tracking/points/ground'),
            ('points/cluster', 'object_tracking/points/cluster'),
            ('points/cropbox', 'object_tracking/points/cropbox')
        ]
    )

    # 3. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 4. Static Transform Alias
    # Aliases the bag's camera frame to the stack's expected 'flir_optical' frame
    tf_alias_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_alias_node',
        arguments=['0', '0', '0', '0', '0', '0', 'mrzr2/front_camera', 'flir_optical'],
        parameters=[{'use_sim_time': True}]
    )

    return [
        bag_play,
        tracking_node,
        rviz_node,
        tf_alias_node
    ]

def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')
    
    return LaunchDescription([
        DeclareLaunchArgument('bag_file', description='Path to the rosbag directory or file'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(avt_341_dir, 'rviz', 'avt_341_ros2.rviz')),
        DeclareLaunchArgument('tracking_params', default_value=os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'mrzr_tracking.yaml')),
        OpaqueFunction(function=launch_setup)
    ])
