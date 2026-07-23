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
from launch_ros.actions import Node, SetParameter


# Global Constants
avt_341_dir = get_package_share_directory('avt_341')
# mrzr_tools_dir = get_package_share_directory('mrzr_tools') #SKIP

# Global Params
global_params = {
    '/map_origin_x': 7885314.3400268555, #7885721.697, #7885314.3400268555, NAD83 m  epsg6495 authority ID for coordinate frame
    '/map_origin_y': 264132.3708496094, #265528.894, #264132.3708496094     NAD83 m
    '/grid_height': 2290.0, #877.0 #2290.0                       # Grid height.
    '/grid_width': 2955.0 #759.0 #2955.0                        # Grid width.
}
vehicle_namespaces = ['mrzr2','mrzr','feda']
vehicle_config_folders = [f'{avt_341_dir}/parameters/config_um_mrzr']

class ArrayIndexSubstitution(Substitution):

    def __init__(self, sub_val: SomeSubstitutionsType, idx: int):
        self.__sub_val = sub_val
        self.__idx = idx

    def describe(self):
        return 'ArrayIndexSubstitution(%s %d)' % (self.__sub_val.describe(), self.__idx)

    def perform(self, context: launch.LaunchContext):
        array_val = self.__sub_val.perform(context)
        # array_val is current a string, need to parse
        array_val = array_val.replace('[', '', 1)[::-1].replace(']', '', 1)[::-1].replace(' ', '').replace("'", "").split(',')
        return array_val[self.__idx]

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
            name='lidar_to_lidar_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "os_lidar", "lidar"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_nad83_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "nad83"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "cg_link", "mrzr/base_link"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cg_to_cage_publisher',
            arguments=["0.8", "0", "1.2", "0", "0", "0", "cg_link", "cage_link"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cage_to_cage_pitched_publisher',
            arguments=["0", "0", "0", "0", "0.209440", "0", "cage_link", "cage_link_pitched"] # 12 degrees
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='caged_pitched_to_plate_publisher',
            arguments=["0", "0", "0.04", "0", "0", "0", "cage_link_pitched", "plate_link"]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='plate_to_ouster_publisher',
            arguments=["0", "-0.0425", "0.0735", "0", "0", "0", "plate_link", "os_sensor"]
        ),
        # MEASUREMENTS FROM HARDWARE
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='plate_to_flir_publisher',
        #     arguments=["0.046", "0.0615", "0.03", "0", "0", "0", "plate_link", "flir_sensor_link"]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='flir_to_flir_rgb_publisher',
        #     arguments=["0", "0", "0", "-1.570796", "0", "-1.570796", "flir_sensor_link", "flir_rgb_link"]
        # ),
        # MEASUREMENTS FROM LIDAR/CAMERA CALIBRATION
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_flir_publisher',
            arguments=['0.114996', '-0.090527', '-0.047696', '0.497703', '0.500851', '-0.500116', '0.501323', 'os_lidar', 'flir_camera']
        ), 
        # SKIP Node(
        #     package='mrzr_tools',
        #     executable='mrzr_tf2_server.py',
        #     name='mrzr_tf2_server',
        #     namespace=f'/{vehicle_name}',
        #     output='screen',
        #     parameters=[{
        #         'publish_odom': True,
        #         'rate': 60.0,
        #         'odom_topic': 'avt_341/odometry',
        #     }],
        #     remappings=[
        #         ("vectornav/gnss", "/vectornav/gnss"),
        #         ("vectornav/pose", "/vectornav/pose"),
        #         ("steering_status", "/steering_status")
        #     ]
        # )
    ]

def recording_node(context):
    record = LaunchConfiguration('record')
    record_select_topic = LaunchConfiguration('record_select_topic')
    record_topics = LaunchConfiguration('record_topics')

    # File naming constants
    time_YYMMDD = datetime.now().strftime('%y%m%d')
    time_HHMMSS = datetime.now().strftime('%H%M%S')
    home_dir = os.path.expanduser('~')

    filename = f"{home_dir}/bags/avt_341_data/{time_YYMMDD}_MRZR_AVT-341_{time_HHMMSS}"
    filename_nav = f"{home_dir}/bags/avt_341_data/{time_YYMMDD}_MRZR_AVT-341_nav_{time_HHMMSS}"

    return [
        GroupAction(condition=IfCondition(record), actions=[
            GroupAction(condition=IfCondition(record_select_topic), actions=[
                ExecuteProcess(
                    cmd=['ros2','bag','record','-o',filename]+record_topics.perform(context).split(' '),
                    output='screen'
                )
            ]),
            GroupAction(condition=UnlessCondition(record_select_topic), actions=[
                ExecuteProcess(
                    cmd=['ros2','bag','record','-o',filename_nav,'-e','/oxts/.*'],
                    output='screen'
                )
            ])
        ])
    ]

def launch_setup(context, *args, **kwargs):
    simulation_mode = LaunchConfiguration('simulation_mode')
    waypoint_mode = LaunchConfiguration('waypoint_mode')
    use_global_path = LaunchConfiguration('use_global_path')
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

    mrzr_nodes = [
        # Transform servers
        *tf2_nodes(context),

        # Recording nodes
        *recording_node(context),

        # SKIP Speed republisher
        # Node(
        #     package='mrzr_tools',
        #     executable='mrzr_speed_republish_node',
        #     name='mrzr_speed_republish_node',
        #     namespace=f'/{vehicle_name}',
        #     remappings=[
        #         ("/mrzr/speed_actual", "/speed_actual")
        #     ]
        # ),

        # SKIP Occupied cells vizualization
        # Node(
        #     package='mrzr_tools',
        #     executable='compressed_occupancy_viz_node.py',
        #     name='compressed_grid_viz_node',
        #     namespace=f'/{vehicle_name}',
        #     remappings=[
        #         ("map_in", "avt_341/occupied_cells"),
        #         ("markers_out", "avt_341/occupied_cells_markers")
        #     ]
        # ),

        # SKIP Logging topic remapping
        # Node(
        #     package='mrzr_tools',
        #     executable='avt_341_topic_remaps.py',
        #     name='mrzr_logging_remap_node',
        #     namespace=f'/{vehicle_name}'
        # ),

        # SKIP Controller
        # launch.actions.IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(mrzr_tools_dir, 'launch', 'stack_controller.launch.py')),
        #     condition=IfCondition(enable_joystick),
        #     launch_arguments={
        #         "max_speed":        max_speed.perform(context),
		#         "mode":             "0",
		#         "joy_topic":        "/joy",
        #         "joy_config":       TernarySubstitution(TextSubstitution(text="xbox"),TextSubstitution(text="can"), IfCondition(simulation_mode)),
		#         "enable_sensors":   "False",
        #         "ouster_hostname":  "os-122223002379.local",
		#         "ns":               "/",
		#         "navstack_ns":      "/mrzr/avt_341"
        #     }.items()
        # ),

        # Logging Republisher
        Node(
            package='um_mrzr_tools',
            executable='logging_remaps.py',
            name='logging_remaps',
        ),

        # NATO AVT-341 Stack
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(avt_341_dir, 'launch', 'um_base.launch.py')),
            launch_arguments={
                "use_sim_time":	                use_sim_time.perform(context),
                "auto_launch_rviz":	            auto_launch_rviz.perform(context),
                "display_type":	                "rviz",
                "waypoints_file":	            f"{avt_341_dir}/config/waypoints.yaml",
                "robot_description_file":	    f"{avt_341_dir}/config/MRZR.urdf",
                "robot_description_veh2_file":	"",
                "robot_description_veh3_file":	"",
                "robot_description_veh4_file":	"",
                "num_vehicles":	                "1",
                "namespace_single_vehicle":	    "False",
                "vehicle_namespaces":	        str(vehicle_namespaces),
                "vehicle_config_folders":	    str(vehicle_config_folders),
                "veh_index":	                f"{veh_index.perform(context)}",
                "use_rqt_display":	            "False",
                "rviz_config":	                f"{avt_341_dir}/rviz/avt_341_mrzr_ros2.rviz",
                "rviz_mult_config":	            f"{avt_341_dir}/rviz/avt_341_multi_vehicle_ros2.rviz",
                "use_lidar_obstacle_detector":  "True",
                "local_planner_method":	        "mpc",
                "global_planner_method":        "a_star",
                "waypoint_mode":	            waypoint_mode.perform(context),
                "simulation_mode":	            simulation_mode.perform(context),
                "use_global_path":	            use_global_path.perform(context),
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
        "use_global_path":	        ["True",	                                    "Set to true to use global path, else local path follows points directly"],
        # "max_speed":	            ["5.0",	                                        "Maximum speed limit for the vehicle"],
        "record":	                ["False",	                                    "Record all topics to '~/avt_341_data/YYMMDD_MRZR_AVT-341_HHMMSS.bag'"],
        "record_select_topic":	    ["False",	                                    "Record only topics defined in 'record_topics'"],
        "record_topics":	        ["/cmd_vel /vectornav/GPS /mrzr/avt_341/forward_speed", "Topics to record as a space separated list (only for record_select_topic=true)"],
        "enable_logging":           ["False",                                       "Enable standardized vehicle logging for V&V efforts"],
        "use_sim_time":	            ["False",	                                    "Use simulation time, usefull for running stack on recorded telemetry bag file"],
        "separate_camera_bag":	    ["True",	                                    "Separate camera topics into a separate bag file"],
        "compress_cameras":	        ["True",	                                    "Only save compressed camera topics (only for separate_camera_bag=true)"],
        "disable_sensor_drivers":   ["True",	                                    "Disable sensor drivers from launching. Drivers are already disabled if simulation_mode is set to true."],
        # "enable_joystick":	    ["True",	                                    "Enable the stack controller joystick. WARNING: This will enable the avt-341 stack to publish directly to /cmd_vel"],
        "auto_launch_rviz":	        ["True",	                                    "Automatically launch rviz"],
        "veh_index":	            ["0",	                                        f"Vehicle index of {vehicle_namespaces}"]
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
