import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Retrieve and expand the bag file path
    bag_file = LaunchConfiguration('bag_file').perform(context)
    bag_file = os.path.expanduser(bag_file)
    
    # Retrieve parameter paths
    avt_341_dir = get_package_share_directory('avt_341')
    tracking_params_path = LaunchConfiguration('tracking_params').perform(context)
    detector_params_path = LaunchConfiguration('detector_params').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    run_detector = LaunchConfiguration('run_detector').perform(context).lower() == 'true'

    # Load YAML parameters manually to avoid ROS 2's strict YAML format requirement
    # (The provided YAML files are simple key-value pairs, which ROS 2's direct parameter file loading doesn't like)
    try:
        with open(tracking_params_path, 'r') as f:
            tracking_params = yaml.safe_load(f)
            # ROS 2 is strict about types. Ensure 'tracker_target_class' is a string.
            if tracking_params and 'tracker_target_class' in tracking_params:
                tracking_params['tracker_target_class'] = str(tracking_params['tracker_target_class'])
    except Exception as e:
        print(f"Error loading tracking parameters: {e}")
        tracking_params = {}

    try:
        with open(detector_params_path, 'r') as f:
            detector_params = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading detector parameters: {e}")
        detector_params = {}

    # Robot description (URDF)
    urdf_path = os.path.join(avt_341_dir, 'config', 'MRZR.urdf')
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

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

    # 2. Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}]
    )

    # 3. Object Detector Node (Optional)
    # Note: This node requires LibTorch (libc10.so).
    # If it fails to load, ensure LibTorch is installed or LD_LIBRARY_PATH is set.
    detector_node = Node(
        package='avt_341',
        executable='avt_341_object_detector_node',
        name='object_detector_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('run_detector')),
        parameters=[detector_params, {'use_sim_time': True}],
        remappings=[
            ('image', '/flir_camera/image_raw'),
            ('detections/vision', '/mrzr/feda_detector/detections/vision')
        ]
    )

    # 4. Object Tracking Node
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

    # 5. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return [
        bag_play,
        rsp_node,
        detector_node,
        tracking_node,
        rviz_node
    ]

def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')
    
    return LaunchDescription([
        DeclareLaunchArgument('bag_file', description='Path to the rosbag directory or file'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(avt_341_dir, 'rviz', 'avt_341_ros2.rviz')),
        DeclareLaunchArgument('tracking_params', default_value=os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'mrzr_tracking.yaml')),
        DeclareLaunchArgument('detector_params', default_value=os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'object_detector.yaml')),
        DeclareLaunchArgument('run_detector', default_value='True', description='Whether to run the object detector node'),
        OpaqueFunction(function=launch_setup)
    ])
