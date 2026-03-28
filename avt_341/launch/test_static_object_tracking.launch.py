import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    output_bag = LaunchConfiguration('output_bag').perform(context)

    # Load YAML parameters manually to avoid ROS 2's strict YAML format requirement
    # (The provided YAML files are simple key-value pairs, which ROS 2's direct parameter file loading doesn't like)
    try:
        with open(tracking_params_path, 'r') as f:
            tracking_params = yaml.safe_load(f)
            if tracking_params is None:
                tracking_params = {}
            
    except Exception as e:
        print(f"Error loading tracking parameters: {e}")
        tracking_params = {}

    # Define Nodes and Processes
    
    # 1. Play the rosbag
    bag_topics = ['/flir_camera/camera_info', '/flir_camera/image_raw', '/mrzr/detections/vision', '/ouster/points', '/mrzr/avt_341/odometry', '/tf', '/tf_static']
    bag_play = TimerAction(
        period=5.0,  # Delay in seconds
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '--clock', '1000', '--topics', *bag_topics],
            output='screen'
        )]
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
            ('detection_2d', '/mrzr/detections/vision'),
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

    # 4. Mission Task Status Publisher
    # Publishes the task status once to set the target vehicle ID
    task_status_pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '1', '/task', 'avt_341_msgs/msg/MissionTaskStatus', 
             '{tracked_vehicle: "0"}'],
        output='screen'
    )

    # 5. Record Bag (Optional)
    actions = [
        bag_play,
        tracking_node,
        rviz_node,
        task_status_pub
    ]

    if record:
        bag_record = ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', output_bag],
            output='screen'
        )
        actions.append(bag_record)

    return actions

def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')
    
    return LaunchDescription([
        DeclareLaunchArgument('bag_file', description='Path to the rosbag directory or file'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(avt_341_dir, 'rviz', 'static_target_detection_testing.rviz')),
        DeclareLaunchArgument('tracking_params', default_value=os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'static_target_tracking.yaml')),
        DeclareLaunchArgument('record', default_value='false', description='Whether to record a rosbag'),
        DeclareLaunchArgument('output_bag', default_value='output_bag', description='Name/path of the output bag to record'),
        OpaqueFunction(function=launch_setup)
    ])
