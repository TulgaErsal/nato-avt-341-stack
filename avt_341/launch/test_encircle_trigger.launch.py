import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    bag_file = LaunchConfiguration('bag_file').perform(context)
    bag_file = os.path.expanduser(bag_file)

    avt_341_dir = get_package_share_directory('avt_341')
    tracking_params_path = LaunchConfiguration('tracking_params').perform(context)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    record = LaunchConfiguration('record').perform(context).lower() == 'true'
    output_bag = LaunchConfiguration('output_bag').perform(context)

    try:
        with open(tracking_params_path, 'r') as f:
            tracking_params = yaml.safe_load(f)
            if tracking_params is None:
                tracking_params = {}
    except Exception as e:
        print(f"Error loading tracking parameters: {e}")
        tracking_params = {}

    # Force autostart so the tracker begins without a mission manager task assignment,
    # and disable mission manager topic mode so the SetTarget service is available if needed.
    # The tracker still publishes to avt_341/target_contacts when tracking starts.
    tracking_params['tracker_use_mission_manager'] = False
    tracking_params['tracker_autostart'] = True

    mission_manager_params = {
        'name': 'mrzr',
        'mission_definition_file': '',
        'mission_paths_file': '',
        'toi_approach_dist': 15.0,
        'toi_encircle_radius': 10.0,
        'toi_encircle_degrees': 180.0,
        'toi_encircle_cw': True,
        'toi_goal_threshold': 5.0,
        'max_speed': 3.0,
        'same_object_distance_threshold': 1.0,
        'follow_scale_x': 15.0,
        'follow_scale_y': 10.0,
        'follow_goal_threshold': 10.0,
        'use_avt_tracker': False,
        'formation_goal_filter': 'none',
        'use_leader_breadcrumbs': False,
        'use_tangent_heading': False,
        'formation_prune_gp': False,
        'x_offset_on_path': False,
        'offsets_from_leader': True,
        'follower_dist_break': 10.0,
        'follower_dot_threshold': 0.0,
        'follower_dot_range': 100.0,
        'fsc_type': 'none',
        'fsc_max_speed_factor': 1.2,
        'formation_debug_visualize': False,
        'vehicle_namespaces': ['mrzr'],
    }

    # 1. Play the rosbag
    bag_topics = [
        '/flir_camera/camera_info',
        '/flir_camera/image_raw',
        '/mrzr/detections/vision',
        '/ouster/points',
        '/mrzr/avt_341/odometry',
        '/tf',
        '/tf_static',
    ]
    bag_play = TimerAction(
        period=0.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag_file, '--clock', '1000', '--topics', *bag_topics],
            output='screen'
        )]
    )

    # 2. Object Tracking Node (uses autostart to begin tracking immediately)
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
        ]
    )

    # 3. Mission Manager Node
    # Subscribes to avt_341/target_contacts; when the tracker publishes a contact
    # the mission manager adds a MoveTo + Encircle task pair.
    mission_manager_node = Node(
        package='avt_341',
        executable='avt_341_mission_manager_node',
        name='mission_manager',
        output='screen',
        parameters=[mission_manager_params, {'use_sim_time': True}],
        remappings=[
            ('avt_341/odometry', '/mrzr/avt_341/odometry'),
        ]
    )

    # 4. Initialize the mission manager navigation state so it accepts contacts.
    # run_state=0 means "active" (anything other than -1 / NotInit).
    nav_state_pub = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-t', '3',
                '/avt_341/state',
                'avt_341_msgs/msg/NavState',
                '{run_state: 0}',
            ],
            output='screen'
        )]
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

    actions = [
        bag_play,
        tracking_node,
        mission_manager_node,
        nav_state_pub,
        rviz_node,
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
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(avt_341_dir, 'rviz', 'static_target_detection_testing.rviz')
        ),
        DeclareLaunchArgument(
            'tracking_params',
            default_value=os.path.join(avt_341_dir, 'parameters', 'config_mrzr', 'static_target_tracking.yaml')
        ),
        DeclareLaunchArgument('record', default_value='false', description='Whether to record a rosbag'),
        DeclareLaunchArgument('output_bag', default_value='output_bag', description='Name/path of the output bag'),
        OpaqueFunction(function=launch_setup)
    ])
