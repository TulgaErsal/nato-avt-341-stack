import os
from datetime import datetime

import launch
import launch.conditions
from launch.conditions import IfCondition, UnlessCondition
from launch.condition import Condition
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.substitution import Substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, GroupAction
from launch_ros.actions import Node


# Global Constants
avt_341_dir = get_package_share_directory('avt_341')
mrzr_tools_dir = get_package_share_directory('mrzr_tools')

vehicle_namespaces = ['mrzr','mrzr2','feda']

class TernarySubstitution(Substitution):

    def __init__(self, true_val: SomeSubstitutionsType, false_val: SomeSubstitutionsType, condition: Condition):
        self.__true_val = true_val
        self.__false_val = false_val
        self.__condition = condition

    def describe(self):
        return 'TernarySubstitution(%s %s %s)' % (self.__true_val.describe(), self.__false_val.describe(), self.__condition.describe())

    def perform(self, context: launch.LaunchContext):
        if self.__condition.evaluate(context):
            return self.__true_val.perform(context)
        else:
            return self.__false_val.perform(context)

# tf publishers
def tf2_nodes(context):
    veh_index = LaunchConfiguration('veh_index')
    idx = int(veh_index.perform(context))
    vehicle_name = vehicle_namespaces[idx]
    return [
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_link_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mrzr_odom_link_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "odom", "mrzr/odom"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='master_link_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "mrzr/base_link"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_link_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "mrzr/os_sensor", "mrzr/lidar"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='os_sensor_ns_fix_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "mrzr/os_sensor", "os_sensor"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ouster_static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "os_lidar", "mrzr/os_lidar"]
        ),
        # LIDAR/CAMERA CALIBRATION
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_calibration_tf_publisher',
            arguments=["-0.283517", "0.0682941", "-0.136282", "1.5783145437948567", "-0.017688723295176803", "-1.481708200033002", "os_lidar", "flir_optical"]
        ), 
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='nad83_link_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "nad83", "epsg_6495"]
        ),
        Node(
            package='mrzr_tools',
            executable='mrzr_tf2_server.py',
            name='mrzr_tf2_server',
            namespace=f'/{vehicle_name}',
            output='screen',
            parameters=[{
                'publish_odom': True,
                'rate': 60.0,
                'odom_topic': 'avt_341/odometry',
            }],
            remappings=[
                ("vectornav/pose_transformed", "/vectornav/pose_transformed"),
                ("steering_status", "/steering_status")
            ]
        )
    ]

def recording_node(context):
    record = LaunchConfiguration('record')
    record_select_topic = LaunchConfiguration('record_select_topic')
    record_topics = LaunchConfiguration('record_topics')
    separate_camera_bag = LaunchConfiguration('separate_camera_bag')
    compress_cameras = LaunchConfiguration('compress_cameras')

    # File naming constants
    time_YYMMDD = datetime.now().strftime('%y%m%d')
    time_HHMMSS = datetime.now().strftime('%H%M%S')
    home_dir = os.path.expanduser('~')

    filename = f"{home_dir}/avt_341_data/{time_YYMMDD}_MRZR_AVT-341_{time_HHMMSS}"
    filename_nav = f"{home_dir}/avt_341_data/{time_YYMMDD}_MRZR_AVT-341_nav_{time_HHMMSS}"
    filename_cams = f"{home_dir}/avt_341_data/{time_YYMMDD}_MRZR_AVT-341_cam_{time_HHMMSS}"

    return [
        GroupAction(condition=IfCondition(record), actions=[
            GroupAction(condition=IfCondition(record_select_topic), actions=[
                ExecuteProcess(
                    cmd=['ros2','bag','record','-o',filename]+record_topics.perform(context).split(' '),
                    output='screen'
                )
            ]),
            GroupAction(condition=UnlessCondition(record_select_topic), actions=[
                GroupAction(condition=IfCondition(separate_camera_bag), actions=[
                    ExecuteProcess(
                        cmd=['ros2','bag','record','-o',filename_nav,'-a','-x','(/flir_rgb/.*|/usb_cam/.*)'],
                        output='screen'
                    ),
                    ExecuteProcess(
                        cmd=['ros2','bag','record','-o',filename_cams,'-e','(/flir_rgb/.*|/usb_cam/.*)'],
                        output='screen',
                        condition=UnlessCondition(compress_cameras)
                    ),
                    ExecuteProcess(
                        cmd=['ros2','bag','record','-o',filename_cams,'-e','(/flir_rgb/.*/compressed.*|/usb_cam/.*/compressed.*)'],
                        output='screen',
                        condition=IfCondition(compress_cameras)
                    )
                ]),
                GroupAction(condition=UnlessCondition(separate_camera_bag), actions=[
                    ExecuteProcess(
                        cmd=['ros2','bag','record','-o',filename,'-a'],
                        output='screen'
                    )
                ])
            ])
        ])
    ]

def launch_setup(context, *args, **kwargs):
    simulation_mode = LaunchConfiguration('simulation_mode')
    waypoint_mode = LaunchConfiguration('waypoint_mode')
    max_speed = LaunchConfiguration('max_speed')
    record = LaunchConfiguration('record')
    record_select_topic = LaunchConfiguration('record_select_topic')
    record_topics = LaunchConfiguration('record_topics')
    enable_logging = LaunchConfiguration('enable_logging')
    use_sim_time = LaunchConfiguration('use_sim_time')
    separate_camera_bag = LaunchConfiguration('separate_camera_bag')
    compress_cameras = LaunchConfiguration('compress_cameras')
    disable_sensor_drivers = LaunchConfiguration('disable_sensor_drivers')
    enable_joystick = LaunchConfiguration('enable_joystick')
    auto_launch_rviz = LaunchConfiguration('auto_launch_rviz')
    veh_index = LaunchConfiguration('veh_index')

    idx = int(veh_index.perform(context))
    vehicle_name = vehicle_namespaces[idx]

    # URDF per vehicle (aligned with vehicle_ids); simulation always uses the
    # UE urdf, matching the old krc_base behavior
    if simulation_mode.perform(context).strip().lower() in ('true', '1'):
        robot_description_files = [f'{avt_341_dir}/config/MRZR_UE.urdf'] * len(vehicle_namespaces)
    else:
        robot_description_files = [f'{avt_341_dir}/config/MRZR.urdf',
                                   f'{avt_341_dir}/config/MRZR_UE.urdf',
                                   '']

    mrzr_nodes = [
        # Transform servers
        *tf2_nodes(context),

        # Recording node
        *recording_node(context),

        # Speed republisher
        Node(
            package='mrzr_tools',
            executable='mrzr_speed_republish_node',
            name='mrzr_speed_republish_node',
            namespace=f'/{vehicle_name}',
            remappings=[
                ("/mrzr/speed_actual", "/speed_actual")
            ]
        ),

        # Occupied cells vizualization
        Node(
            package='mrzr_tools',
            executable='compressed_occupancy_viz_node.py',
            name='compressed_grid_viz_node',
            namespace=f'/{vehicle_name}',
            remappings=[
                ("map_in", "avt_341/occupied_cells"),
                ("markers_out", "avt_341/occupied_cells_markers")
            ]
        ),

        # Logging topic remapping
        Node(
            package='mrzr_tools',
            executable='avt_341_topic_remaps.py',
            name='mrzr_logging_remap_node',
            namespace=f'/{vehicle_name}'
        ),

        # Vectornav transformer
        Node(
            package='mrzr_tools',
            executable='vectornav_transformer.py',
            name='vectornav_transformer',
            namespace=f'/{vehicle_name}',
            parameters=[{
                'vehicle_frame': 'mrzr/base_link',
                'sensor_frame': 'mrzr/vectornav_link',
                'flip_pitch': True,
                'flip_yaw': True,
            }]
        ),

        # Controller
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mrzr_tools_dir, 'launch', 'stack_controller.launch.py')),
            condition=IfCondition(enable_joystick),
            launch_arguments={
                "max_speed":        max_speed.perform(context),
		        "mode":             "0",
		        "joy_topic":        "/joy",
                "joy_config":       TernarySubstitution(TextSubstitution(text="xbox"),TextSubstitution(text="can"), IfCondition(simulation_mode)),
		        "enable_sensors":   "False",
                "ouster_hostname":  "os-122223002379.local",
		        "ns":               "/",
		        "navstack_ns":      "/mrzr/avt_341"
            }.items()
        ),

        # NATO AVT-341 Stack
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'base.launch.py')),
            launch_arguments={
                "use_sim_time":                 use_sim_time.perform(context),
                "auto_launch_rviz":             auto_launch_rviz.perform(context),
                "waypoints_file":               f"{avt_341_dir}/config/krc_VDA_waypoints/loop_2_waypoints_nad83.yaml",
                "robot_description_files":      str(robot_description_files),
                "vehicle_ids":                  str(vehicle_namespaces),
                "spawn_filter_vehicle_ids":     f"['{vehicle_name}']",
                "ros_param_files":                 f"['{avt_341_dir}/parameters/overrides/krc_mrzr.yaml']",
                "node_config_file":                f"{avt_341_dir}/parameters/metadata/krc_mrzr.yaml",
                "rviz_config":                  f"{avt_341_dir}/rviz/avt_341_mrzr.rviz",
                "rviz_mult_config":             f"{avt_341_dir}/rviz/avt_341_multi_vehicle.rviz",
                "use_lidar_obstacle_detector":  "True",
                "local_planner_method":         "mpc",
                "waypoint_mode":                waypoint_mode.perform(context),
                "enable_logging":               enable_logging.perform(context),
            }.items()
        )
    ]

    return mrzr_nodes

def generate_launch_description():
    avt_341_dir = get_package_share_directory('avt_341')

    # Define arguments
    launch_arg_dict = {
        "simulation_mode":	        ["False",	                                    "Set to true if the vehicle is in Unreal Engine"],
        "waypoint_mode":	        ["False",	                                    "Set to true for waypoint following mode"],
        "max_speed":	            ["5.0",	                                        "Maximum speed limit for the vehicle"],
        "record":	                ["False",	                                    "Record all topics to '~/avt_341_data/YYMMDD_MRZR_AVT-341_HHMMSS.bag'"],
        "record_select_topic":	    ["False",	                                    "Record only topics defined in 'record_topics'"],
        "record_topics":	        ["/cmd_vel /vectornav/GPS /mrzr/avt_341/forward_speed", "Topics to record as a space separated list (only for record_select_topic=true)"],
        "enable_logging":           ["False",                                       "Enable standardized vehicle logging for V&V efforts"],
        "use_sim_time":	            ["False",	                                    "Use simulation time, usefull for running stack on recorded telemetry bag file"],
        "separate_camera_bag":	    ["True",	                                    "Separate camera topics into a separate bag file"],
        "compress_cameras":	        ["True",	                                    "Only save compressed camera topics (only for separate_camera_bag=true)"],
        "disable_sensor_drivers":   ["False",	                                    "Disable sensor drivers from launching. Drivers are already disabled if simulation_mode is set to true."],
        "enable_joystick":	        ["True",	                                    "Enable the stack controller joystick. WARNING: This will enable the avt-341 stack to publish directly to /cmd_vel"],
        "auto_launch_rviz":	        ["True",	                                    "Automatically launch rviz"],
        "veh_index":	            ["0",	                                        f"Vehicle index of {vehicle_namespaces}"]
    }
    launch_args = []
    for arg,[default,desc] in launch_arg_dict.items():
        launch_args.append(DeclareLaunchArgument(arg, default_value=default, description=desc))

    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup),
    ])
