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
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    output_bag = LaunchConfiguration('output_bag').perform(context)

    # Load tracking parameters
    try:
        with open(tracking_params_path, 'r') as f:
            tracking_params = yaml.safe_load(f)
            if tracking_params is None:
                tracking_params = {}

            # Apply requested parameter configuration
            tracking_params.update({
                'tracking_rate': 10.0,
                'camera_frame': 'mrzr2/front_camera',
                'world_frame': 'map',
                'publish_detection': True,
                'publish_odometry': True,
                'tracker_use_mission_manager': False,
                'tracker_target_classes': ['0'],
                'filters_kalman_process': 0.01,
                'filters_kalman_measurement': 0.1,
                # Camera-based range estimation parameters
                'camera_target_height': 1.9,
                'camera_bbox_pixel_sigma': 4.0,
                # IMM model probabilities and Markov transition probability
                # CV model is preferred for straight-line driving; CTR kicks
                # in when the likelihood of turning becomes higher.
                'filters_imm_cv_init_prob': 0.33,
                'filters_imm_ctr_init_prob': 0.33,
                'filters_imm_nm_init_prob': 0.33,
                'filters_imm_persistence_prob': 0.9,
                'sync_enable': False,
                'sync_detection': 0.1,
                'sync_use_callback': True,
                # Obstacle detector integration (now embedded in tracking node)
                'od_robot_base_link': 'mrzr2/base_link',
                'obstacle_association_max_dist': 5.0,
            })
    except Exception as e:
        print(f"Error loading tracking parameters: {e}")
        tracking_params = {}

    # Define Nodes and Processes

    # 1. Play the rosbag
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--clock', '1000'],
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
            ('camera_info', '/mrzr2/front_camera/camera_info'),
            ('image', '/mrzr2/front_camera/image'),
            ('detection_2d', '/mrzr2/front_camera/detections_2d'),
            ('points/input', '/mrzr2/avt_341/points')
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

    # 5. Mission Task Status Publisher
    # Publishes the task status once to set the target vehicle ID
    task_status_pub = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '1', '/task', 'avt_341_msgs/msg/MissionTaskStatus',
             '{tracked_vehicle: "0"}'],
        output='screen'
    )

    # 6. Record Bag (Optional)
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
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(avt_341_dir, 'rviz', 'test_object_tracking.rviz')),
        DeclareLaunchArgument('tracking_params', default_value=os.path.join(avt_341_dir, 'config', 'parameters', 'object_tracking.yaml')),
        DeclareLaunchArgument('record', default_value='false', description='Whether to record a rosbag'),
        DeclareLaunchArgument('output_bag', default_value='output_bag', description='Name/path of the output bag to record'),
        OpaqueFunction(function=launch_setup)
    ])
