import os

import launch
import launch.conditions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import launch.conditions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, SetParameter

# Global Params
global_params = {
    '/map_origin_x': 7885314.3400268555,
    '/map_origin_y': 264132.3708496094
}

# tf publishers
def tf2_nodes():
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
            name='lidar_ns_fix_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "mrzr/os_sensor", "os_sensor"]
        ),
        Node(
            package='mrzr_tools',
            executable='mrzr_tf2_server.py',
            name='mrzr_tf2_server',
            namespace='/mrzr',
            output='screen',
            parameters=[{
                'publish_odom': True,
                'odom_topic': 'avt_341/odometry',
            }],
            remappings=[
                ("/mrzr/vectornav/GPS", "/vectornav/GPS"),
                ("/mrzr/vectornav/Odom", "/vectornav/Odom"),
                ("/mrzr/steering_status", "/steering_status")
            ]
        )
    ]

def launch_setup(context, *args, **kwargs):
    simulation_mode = LaunchConfiguration('simulation_mode')
    waypoint_mode = LaunchConfiguration('waypoint_mode')
    use_global_path = LaunchConfiguration('use_global_path')
    max_speed = LaunchConfiguration('max_speed')
    record = LaunchConfiguration('record')
    record_select_topic = LaunchConfiguration('record_select_topic')
    record_topics = LaunchConfiguration('record_topics')
    use_sim_time = LaunchConfiguration('use_sim_time')
    separate_camera_bag = LaunchConfiguration('separate_camera_bag')
    compress_cameras = LaunchConfiguration('compress_cameras')
    disable_sensor_drivers = LaunchConfiguration('disable_sensor_drivers')
    enable_joystick = LaunchConfiguration('enable_joystick')
    auto_launch_rviz = LaunchConfiguration('auto_launch_rviz')

    avt_341_dir = get_package_share_directory('avt_341')
    mrzr_nodes = [
        # Transform servers
        *tf2_nodes(),

        # Speed republisher
        Node(
            package='mrzr_tools',
            executable='mrzr_speed_republish_node',
            name='mrzr_speed_republish_node',
            namespace='/mrzr',
            remappings=[
                ("/mrzr/speed_actual", "/speed_actual")
            ]
        ),

        # NATO AVT-341 Stack
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'krc_base.launch.py')),
            launch_arguments={
                "use_sim_time":	                use_sim_time.perform(context),
                "auto_launch_rviz":	            auto_launch_rviz.perform(context),
                "display_type":	                "rviz",
                "waypoints_file":	            f"{avt_341_dir}/config/krc_VDA_waypoints/side_slope.yaml",
                "robot_description_file":	    f"{avt_341_dir}/config/MRZR.urdf",
                "robot_description_veh2_file":	"",
                "robot_description_veh3_file":	"",
                "robot_description_veh4_file":	"",
                "num_vehicles":	                "1",
                "namespace_single_vehicle":	    "True",
                "vehicle_namespaces":	        "['mrzr']",
                "vehicle_config_folders":	    f"['{avt_341_dir}/parameters/config_mrzr']",
                "veh_index":	                "0",
                "use_rqt_display":	            "False",
                "rviz_config":	                f"{avt_341_dir}/rviz/avt_341_mrzr.rviz",
                "rviz_mult_config":	            f"{avt_341_dir}/rviz/avt_341_multi_vehicle.rviz",
                "use_lidar_obstacle_detector":  "True",
                "local_planner_method":	        "rcc",
                "waypoint_mode":	            waypoint_mode.perform(context),
                "simulation_mode":	            simulation_mode.perform(context),
                "use_global_path":	            use_global_path.perform(context),
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
        "use_global_path":	        ["True",	                                    "Set to true to use global path, else local path follows points directly"],
        "max_speed":	            ["5.0",	                                        "Maximum speed limit for the vehicle"],
        "record":	                ["False",	                                    "Record all topics to '~/avt_341_data/YYMMDD_MRZR_AVT-341_HHMMSS.bag'"],
        "record_select_topic":	    ["False",	                                    "Record only topics defined in 'record_topics'"],
        "record_topics":	        ["/cmd_vel /vectornav/GPS /mrzr/mrzr_velocity", "Topics to record as a space separated list (only for record_select_topic=true)"],
        "use_sim_time":	            ["False",	                                    "Use simulation time, usefull for running stack on recorded telemetry bag file"],
        "separate_camera_bag":	    ["True",	                                    "Separate camera topics into a separate bag file"],
        "compress_cameras":	        ["True",	                                    "Only save compressed camera topics (only for separate_camera_bag=true)"],
        "disable_sensor_drivers":   ["False",	                                    "Disable sensor drivers from launching. Drivers are already disabled if simulation_mode is set to true."],
        "enable_joystick":	        ["True",	                                    "Enable the stack controller joystick. WARNING: This will enable the avt-341 stack to publish directly to /cmd_vel"],
        "auto_launch_rviz":	        ["True",	                                    "Automatically launch rviz"]
    }
    launch_args = []
    for arg,[default,desc] in launch_arg_dict.items():
        launch_args.append(DeclareLaunchArgument(arg, default_value=default, description=desc))

    # Set global parameters
    global_param_setters = []
    for param,val in global_params.items():
        global_param_setters.append(SetParameter(name=param, value=val),)

    return LaunchDescription([
        *launch_args,
        *global_param_setters,
        OpaqueFunction(function=launch_setup),
    ])
