import os
from datetime import datetime

import launch
import launch.conditions
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, GroupAction
from launch_ros.actions import Node

from avt_341_param_lib.runtime.launch_expressions import TernarySubstitution


# Global Constants
avt_341_bringup_dir = get_package_share_directory('avt_341_bringup')

vehicle_namespaces = ['mrzr2', 'mrzr', 'feda']


# tf publishers
def tf2_nodes(context):
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
            arguments=["0", "0", "0", "0", "0.209440", "0", "cage_link", "cage_link_pitched"]  # 12 degrees
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
        # Lidar/camera calibration
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_to_flir_publisher',
            arguments=['0.114996', '-0.090527', '-0.047696', '0.497703', '0.500851', '-0.500116', '0.501323', 'os_lidar', 'flir_camera']
        ),
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

    filename = f"{home_dir}/avt_341_data/{time_YYMMDD}_MRZR2_AVT-341_{time_HHMMSS}"
    filename_nav = f"{home_dir}/avt_341_data/{time_YYMMDD}_MRZR2_AVT-341_nav_{time_HHMMSS}"
    filename_cams = f"{home_dir}/avt_341_data/{time_YYMMDD}_MRZR2_AVT-341_cam_{time_HHMMSS}"

    return [
        GroupAction(condition=IfCondition(record), actions=[
            GroupAction(condition=IfCondition(record_select_topic), actions=[
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '-o', filename] + record_topics.perform(context).split(' '),
                    output='screen'
                )
            ]),
            GroupAction(condition=UnlessCondition(record_select_topic), actions=[
                GroupAction(condition=IfCondition(separate_camera_bag), actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', filename_nav, '-a', '-x', '(/flir_camera/.*|/flir_camera_downsized/.*)'],
                        output='screen'
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', filename_cams, '-e', '(/flir_camera/.*|/flir_camera_downsized/.*)'],
                        output='screen',
                        condition=UnlessCondition(compress_cameras)
                    ),
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', filename_cams, '-e', '(/flir_camera/.*/compressed.*|/flir_camera_downsized/.*/compressed.*)'],
                        output='screen',
                        condition=IfCondition(compress_cameras)
                    )
                ]),
                GroupAction(condition=UnlessCondition(separate_camera_bag), actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'bag', 'record', '-o', filename, '-a'],
                        output='screen'
                    )
                ])
            ])
        ])
    ]


def resolve_uab_mcr_root():
    mcr_root = os.environ.get('MCR_ROOT', '/usr/local/MATLAB/MATLAB_Runtime/R2023a')
    if not os.path.isdir(mcr_root):
        raise RuntimeError(
            f"uab_perception_node requires MATLAB Runtime R2023a, but the resolved "
            f"MCR_ROOT '{mcr_root}' does not exist on this machine. If MCR_ROOT is set "
            f"in the environment, verify it points at a MATLAB Runtime R2023a install; "
            f"otherwise install MATLAB Runtime R2023a at the default path above."
        )
    return mcr_root


def launch_setup(context, *args, **kwargs):
    resolve_uab_mcr_root()

    simulation_mode = LaunchConfiguration('simulation_mode')
    record = LaunchConfiguration('record')
    record_select_topic = LaunchConfiguration('record_select_topic')
    record_topics = LaunchConfiguration('record_topics')
    enable_logging = LaunchConfiguration('enable_logging')
    use_sim_time = LaunchConfiguration('use_sim_time')
    separate_camera_bag = LaunchConfiguration('separate_camera_bag')
    compress_cameras = LaunchConfiguration('compress_cameras')
    auto_launch_rviz = LaunchConfiguration('auto_launch_rviz')
    veh_index = LaunchConfiguration('veh_index')

    idx = int(veh_index.perform(context))

    if simulation_mode.perform(context).strip().lower() in ('true', '1'):
        robot_description_files = [f'{avt_341_bringup_dir}/urdf/MRZR_UE.urdf'] * len(vehicle_namespaces)
    else:
        robot_description_files = [f'{avt_341_bringup_dir}/urdf/UM_MRZR2.urdf', '', '']

    um_nodes = [
        # Transform servers
        *tf2_nodes(context),

        # Recording node
        *recording_node(context),

        # Logging topic remapping
        Node(
            package='um_mrzr_tools',
            executable='logging_remaps.py',
            name='logging_remaps',
        ),

        # NATO AVT-341 Stack
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(avt_341_bringup_dir, 'launch', 'base.launch.py')),
            launch_arguments={
                "use_sim_time":                 use_sim_time.perform(context),
                "auto_launch_rviz":              auto_launch_rviz.perform(context),
                "robot_description_files":      str(robot_description_files),
                "vehicle_ids":                  str(vehicle_namespaces),
                "spawn_filter_vehicle_ids":     f"['{vehicle_namespaces[idx]}']",
                "ros_param_files":              f"['{avt_341_bringup_dir}/parameters/um_ros_params.yaml']",
                "node_config_file":             f"{avt_341_bringup_dir}/parameters/um_node_config.yaml",
                "rviz_config":                  f"{avt_341_bringup_dir}/rviz/avt_341_um_mrzr.rviz",
                "use_lidar_obstacle_detector":  "True",
                "local_planner_method":         "mpc",
                "enable_logging":               enable_logging.perform(context),
            }.items()
        )
    ]

    return um_nodes


def generate_launch_description():

    # Define arguments
    launch_arg_dict = {
        "simulation_mode":       ["False",                                                     "Set to true if the vehicle is in Unreal Engine"],
        "record":                ["False",                                                     "Record all topics to '~/avt_341_data/YYMMDD_MRZR2_AVT-341_HHMMSS.bag'"],
        "record_select_topic":   ["False",                                                     "Record only topics defined in 'record_topics'"],
        "record_topics":         ["/cmd_vel /oxts/ins/roll /oxts/ins/pitch /oxts/ins/yaw",      "Topics to record as a space separated list (only for record_select_topic=true)"],
        "enable_logging":        ["False",                                                      "Enable standardized vehicle logging for V&V efforts"],
        "use_sim_time":          ["False",                                                      "Use simulation time, useful for running stack on recorded telemetry bag file"],
        "separate_camera_bag":   ["True",                                                       "Separate camera topics into a separate bag file"],
        "compress_cameras":      ["True",                                                       "Only save compressed camera topics (only for separate_camera_bag=true)"],
        "auto_launch_rviz":      ["True",                                                       "Automatically launch rviz"],
        "veh_index":             ["0",                                                          f"Vehicle index of {vehicle_namespaces}"]
    }
    launch_args = []
    for arg, [default, desc] in launch_arg_dict.items():
        launch_args.append(DeclareLaunchArgument(arg, default_value=default, description=desc))

    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup),
    ])
