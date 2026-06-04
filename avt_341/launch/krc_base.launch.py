import os
import yaml
import json

import launch.conditions
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetLaunchConfiguration, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import OpaqueFunction

from launch.substitution import Substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition

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

class ToUpper(Substitution):

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'ToUpper(%s)' % (self.__sub_val.describe())

    def perform(self, context: launch.LaunchContext):
        return self.__sub_val.perform(context).upper()

class Invert(Substitution):

    def __init__(self, sub_val: SomeSubstitutionsType):
        self.__sub_val = sub_val

    def describe(self):
        return 'Invert(%s)' % (self.__sub_val.describe())

    def perform(self, context: launch.LaunchContext):
        val = self.__sub_val.perform(context).lower()
        is_true = val in ['true', '1']
        return str(not is_true)

class Concat(Substitution):

        def __init__(self, sub_val: SomeSubstitutionsType, concat_val):
            self.__sub_val = sub_val
            self.__concat_val = concat_val

        def describe(self):
            return 'StringConcate(%s %s)' % (self.__sub_val.describe(), self.__concat_val)

        def perform(self, context: launch.LaunchContext):
            return self.__sub_val.perform(context) + self.__concat_val

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

def evaluate_waypoint_parameters(context, *args, **kwargs):
    waypoints_file_path = LaunchConfiguration('waypoints_file').perform(context)
    waypoint_mode = LaunchConfiguration('waypoint_mode')

    waypoints_x = "[ ]"
    waypoints_y = "[ ]"
    is_empty_waypoints = not waypoints_file_path
    with open(waypoints_file_path, 'r') as f:
        for line in f.readlines():
            if "waypoints_x" in line:
                waypoints_x = line.split(":")[1]
                is_empty_waypoints = is_empty_waypoints or waypoints_x.replace(' ', '') == '[]'
            if "waypoints_y" in line:
                waypoints_y = line.split(":")[1]
                is_empty_waypoints = is_empty_waypoints or waypoints_y.replace(' ', '') == '[]'

    if is_empty_waypoints:
        waypoints_x = "[ 0.0 ]"
        waypoints_y = "[ 0.0 ]"

    return [
        # waypoint_mode enabled
        DeclareLaunchArgument('waypoints_x', description="List of waypoint x coordinates. Will override waypoints_file is specified.", default_value=waypoints_x,
                              condition=IfCondition(waypoint_mode)),
        DeclareLaunchArgument('waypoints_y', description="List of waypoint y coordinates. Will override waypoints_file is specified.", default_value=waypoints_y,
                              condition=IfCondition(waypoint_mode)),
        DeclareLaunchArgument('is_empty_waypoints',
                              description="Parameter set internally to detect if waypoints file empty. ROS2 foxy workaround (https://answers.ros.org/question/396556/what-is-best-practice-for-parameters-which-are-empty-lists-in-ros2/). Do not set manually",
                              default_value=str(is_empty_waypoints).capitalize(),
                              condition=IfCondition(waypoint_mode)),
        # waypoint_mode disabled
        DeclareLaunchArgument('waypoints_x', description="List of waypoint x coordinates. Will override waypoints_file is specified.", default_value="[ 0.0 ]",
                              condition=UnlessCondition(waypoint_mode)),
        DeclareLaunchArgument('waypoints_y', description="List of waypoint y coordinates. Will override waypoints_file is specified.", default_value="[ 0.0 ]",
                              condition=UnlessCondition(waypoint_mode)),
        DeclareLaunchArgument('is_empty_waypoints',
                              description="Parameter set internally to detect if waypoints file empty. ROS2 foxy workaround (https://answers.ros.org/question/396556/what-is-best-practice-for-parameters-which-are-empty-lists-in-ros2/). Do not set manually",
                              default_value="True",
                              condition=UnlessCondition(waypoint_mode)),
    ]

def evaluate_speed_controller(params, context, *args, **kwargs):
    simulation_mode = LaunchConfiguration('simulation_mode').perform(context)
    local_planner_method = LaunchConfiguration('local_planner_method').perform(context)

    if local_planner_method == 'dwa' or local_planner_method == 'mpc':
        return [Node(
                    package='avt_341',
                    executable='avt_341_speed_control_node',
                    name='vehicle_control_node',
                    output='screen',
                    parameters=[
                        {k: LaunchConfiguration(f'speed_control_{k}') for k in params['speed_control'].keys()}
                    ],
        )]
    else:
        return [Node(
                    package='avt_341',
                    executable='avt_341_control_node',
                    name='vehicle_control_node',
                    output='screen',
                    parameters=[
                        {k: LaunchConfiguration(f'control_{k}') for k in params['control'].keys()}
                    ],
        )]

def evaluate_global_planner(params, context, *args, **kwargs):
    display_type = LaunchConfiguration('display_type').perform(context)
    global_planner_method = LaunchConfiguration('global_planner_method').perform(context)

    if global_planner_method == 'a_star':
        return [Node(
                    package='avt_341',
                    executable='avt_341_global_path_node',
                    name='avt_341_global_path_node',
                    output='screen',
                    parameters=[
                        {
                            'display': display_type,
                            '/waypoints_x': LaunchConfiguration('waypoints_x'),
                            '/waypoints_y': LaunchConfiguration('waypoints_y'),
                            '/is_empty_waypoints': LaunchConfiguration('is_empty_waypoints'),
                        },
                        {k: LaunchConfiguration(f'global_planner_{k}') for k in params['global_planner'].keys()}
                    ],
                )]
    else: # global_planner_method == 'pf':
        return [Node(
                package='avt_341',
                executable='avt_341_pf_global_path_node',
                name='pf_global_path_node',
                output='screen',
                parameters=[
                    {k: LaunchConfiguration(f'pf_global_planner_{k}') for k in params['pf_global_planner'].keys()}
                ],
        )]

def evaluate_local_planner(params, context, *args, **kwargs):
    display_type = LaunchConfiguration('display_type').perform(context)
    local_planner_method = LaunchConfiguration('local_planner_method').perform(context)
    simulation_mode = LaunchConfiguration('simulation_mode').perform(context)

    if local_planner_method == 'rcc':
        return [Node(
                    package='avt_341',
                    executable='avt_341_local_planner_node',
                    name='local_planner_node',
                    output='screen',
                    parameters=[
                        {'display': display_type},
                        {k: LaunchConfiguration(f'rcc_local_planner_{k}') for k in params['rcc_local_planner'].keys()}
                    ],
        )]
    elif local_planner_method == 'dwa':
        return [Node(
                    package='avt_341',
                    executable='avt_341_dwa_planner_node',
                    name='local_dwa_planner_node',
                    output='screen',
                    parameters=[
                        {k: LaunchConfiguration(f'dwa_local_planner_{k}') for k in params['dwa_local_planner'].keys()}
                    ],
        )]
    elif local_planner_method == 'mpc':
        return [
            # MPC Planner
            Node(
                package='avt_341',
                executable='avt_341_mpc_planner_node',
                name='mpc_planner_node',
                output='screen',
                #prefix=['xterm -e gdb -ex run --args'],
                parameters=[{k: LaunchConfiguration(f'mpc_local_planner_{k}') for k in params['mpc_local_planner'].keys()}],
            ),
            # Obstacle Processor
            Node(
                package='avt_341',
                executable='obstacle_processor_node',
                name='obstacle_processor_node',
                output='screen',
                parameters=[{k: LaunchConfiguration(f'mpc_local_planner_{k}') for k in params['mpc_local_planner'].keys()}],
            ),
            # Segmentation Grid Processor
            Node(
                package='avt_341',
                executable='segmentation_grid_processor_node',
                name='segmentation_grid_processor_node',
                output='screen',
                parameters=[{k: LaunchConfiguration(f'mpc_local_planner_{k}') for k in params['mpc_local_planner'].keys()}],
            ),
            # Vehicle Converter
            Node(
                package='avt_341',
                executable='veh_converter_node',
                name='avt_341_veh_converter_node',
                output='screen'
            ),
            # Goal Point Processor
            Node(
                package='avt_341',
                executable='goal_point_processor_node',
                name='goal_point_processor_node',
                output='screen',
                parameters=[{k: LaunchConfiguration(f'mpc_local_planner_{k}') for k in params['mpc_local_planner'].keys()}],
            )
        ]
    else: # local_planner_method == 'pf':
        return [Node(
                package='avt_341',
                executable='avt_341_pf_planner_node',
                name='local_pf_planner_node',
                output='screen',
                parameters=[
                    {'display': display_type},
                    {k: LaunchConfiguration(f'pf_local_planner_{k}') for k in params['pf_local_planner'].keys()}
                ],
        )]

def launch_setup(context, *args, **kwargs):
    # General args
    use_sim_time = LaunchConfiguration('use_sim_time')                                  # Use simulation time (mainly for rosbag data replay)
    auto_launch_rviz = LaunchConfiguration('auto_launch_rviz')                          # Automatically launch rviz display window
    display_type = LaunchConfiguration('display_type')                                  # Type of display method to use. Values = [rviz, image]
    waypoints_file = LaunchConfiguration('waypoints_file')                              # Path to waypoint file to use
    robot_desc_list = [ LaunchConfiguration('robot_description_file'),                  # URDF robot description contents
                        LaunchConfiguration('robot_description_veh2_file'),             # URDF robot description contents for vehicle 2
                        LaunchConfiguration('robot_description_veh3_file'),             # URDF robot description contents for vehicle 3
                        LaunchConfiguration('robot_description_veh4_file')]             # URDF robot description contents for vehicle 4
    num_vehicles = LaunchConfiguration('num_vehicles')                                  # Number of vehicles controlled by navigation stack
    namespace_single_vehicle = LaunchConfiguration('namespace_single_vehicle')          # If true, will use vehicle namespace even when only a single vehicle is used (num_vehicles=1)
    vehicle_namespaces = LaunchConfiguration('vehicle_namespaces')                      # Vehicle namespaces
    vehicle_config_folders = LaunchConfiguration('vehicle_config_folders')              # Directories containing all config files for each vehicle
    veh_index = LaunchConfiguration('veh_index')                                        # Current index value from vehicle_namespaces and vehicle_config_folders of the vehicle being launched
    use_rqt_display = LaunchConfiguration('use_rqt_display')                            # Launch rviz within rqt window
    rviz_config = LaunchConfiguration('rviz_config')                                    # Single vehicle rviz config file
    rviz_mult_config = LaunchConfiguration('rviz_mult_config')                          # Multiple vehicle rviz config file
    use_lidar_obstacle_detector = LaunchConfiguration('use_lidar_obstacle_detector')    # Enable lidar obstacle segmentation
    local_planner_method = LaunchConfiguration('local_planner_method')                  # Local planner method. Values = ["rcc", "dwa", "pf"]
    waypoint_mode = LaunchConfiguration('waypoint_mode')                                # Set to true for waypoint following mode
    simulation_mode = LaunchConfiguration('simulation_mode')                            # Set to true for UE simulation
    use_global_path = LaunchConfiguration('use_global_path')                            # Set to true to use global path, else local path follows points directly
    global_planner_method = LaunchConfiguration('global_planner_method')                # Global planner method. Values = ["a_star", "pf"]
    enable_logging = LaunchConfiguration('enable_logging')                              # Enable standardized vehicle logging for V&V efforts
    logging_path = LaunchConfiguration('logging_path')                                  # Save path for vehicle logging

    # Vehicle index
    idx = int(veh_index.perform(context))

    # Read robot description
    if simulation_mode.perform(context) in ["True", "true"]:
        with open(robot_desc_list[1].perform(context), 'r') as infp:
            robot_desc = infp.read()
    else:
        with open(robot_desc_list[idx].perform(context), 'r') as infp:
            robot_desc = infp.read()

    # Load vehicle config files
    vehicle_config_folders_arr = json.loads(vehicle_config_folders.perform(context).replace("'","\""))
    param_dir = vehicle_config_folders_arr[idx]
    param_files = {f[:-len('.yaml')]: os.path.join(param_dir, f) for f in os.listdir(param_dir) if f.endswith('.yaml')}
    params = {}
    param_refs = {}
    for k, v in param_files.items():
        with open(v) as f:
            params[k] = yaml.load(f, Loader=yaml.FullLoader)
            param_refs[k] = {}
        keys_list = list(params[k].keys())
        # Flatten sub-dictionaries
        for ki in keys_list:
            vi = params[k][ki]
            if type(vi) is dict:
                for kii, vii in vi.items():
                    params[k]['_'.join([ki, kii])] = vii
                del params[k][ki]
        for ki, vi in params[k].items():
            if type(vi) is str and vi.startswith('$Python:'):
                python_str = vi[len('$Python:'):]
                params[k][ki] = eval(python_str)
            if type(vi) is str and vi.startswith("$val{"):
                key_sub = vi[5:-1].split(':')
                params[k][ki] = params[key_sub[0]][key_sub[1]]
            if type(vi) is str and vi.startswith("$ref{"):
                param_refs[k][ki] = vi[5:-1]
        for ki in param_refs[k].keys():
            del params[k][ki]
    arg_list = [DeclareLaunchArgument(f'{k}_{ki}', default_value=str(vi)) for k, v in params.items() for ki, vi in v.items()]

    # Load waypoints
    arg_list.extend(evaluate_waypoint_parameters(context=context, args=args, kwargs=kwargs))
    
    vehicle_node_list = ([
        # Define robot namespace
        PushRosNamespace(
            condition=IfCondition(PythonExpression([num_vehicles, ' > 1 or ', namespace_single_vehicle])),
            namespace=ArrayIndexSubstitution(vehicle_namespaces, idx)
        ),
        
        # Static transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc,
                            'frame_prefix': TernarySubstitution(Concat(ArrayIndexSubstitution(vehicle_namespaces, idx), '/'),
                                                                TextSubstitution(text=''),
                                                                IfCondition(PythonExpression([num_vehicles, ' > 1 or ', namespace_single_vehicle])))}]
        ),

        # Lidar Perception Algorithms
        Node(
            package='avt_341',
            executable='avt_341_perception_node',
            name='perception_local_node',
            output='screen',
            parameters=[{k: LaunchConfiguration(f'perception_local_{k}') for k in params['perception_local'].keys()}],
            remappings=[
                ('avt_341/terrain_slope', 'avt_341/terrain_slope_local'),
                ('avt_341/terrain_rms', 'avt_341/terrain_rms_local'),
            ]
        ),
        Node(
            package='avt_341',
            executable='avt_341_perception_node',
            name='perception_global_node',
            output='screen',
            parameters=[{k: LaunchConfiguration(f'perception_global_{k}') for k in params['perception_global'].keys()}],
            remappings=[
                ('avt_341/terrain_slope', 'avt_341/terrain_slope_global'),
                ('avt_341/terrain_rms', 'avt_341/terrain_rms_global'),
                ('avt_341/occupancy_grid', 'avt_341/occupancy_grid_low_res'),
                ('avt_341/occupancy_grid_updates', 'avt_341/occupancy_grid_low_res_updates'),
            ]
        ),
        Node(
            package='avt_341',
            executable='avt_341_perception_node',
            name='perception_rms_node',
            output='screen',
            parameters=[{k: LaunchConfiguration(f'perception_rms_{k}') for k in params['perception_rms'].keys()}],
            remappings=[
                ('avt_341/occupancy_grid', 'avt_341/rms_perception/occupancy_grid'),
                ('avt_341/segmentation_grid', 'avt_341/rms_perception/segmentation_grid'),
            ]
        ),
        Node(
            package='avt_341',
            executable='avt_341_lidar_normal_estimation_node',
            name='lidar_normal_estimation_node',
            output='screen',
            parameters=[{k: LaunchConfiguration(f'normal_estimation_{k}') for k in params['normal_estimation'].keys()}]
        ),
        GroupAction(condition=IfCondition(use_lidar_obstacle_detector), actions=[
            Node(
                package='avt_341',
                executable='avt_341_lidar_obstacle_detector_node',
                name='lidar_obstacle_detector_node',
                output='screen',
                parameters=[{k: LaunchConfiguration(f'obstacle_detector_{k}') for k in params['obstacle_detector'].keys()}]
            )
        ]),

        # Grid Commpression
        Node(
            package='avt_341',
            executable='avt_341_grid_compression_node',
            name='grid_compression_local'
        ),

        # Grid Commpression (Global)
        Node(
            package='avt_341',
            executable='avt_341_grid_compression_node',
            name='grid_compression_global',
            remappings=[
                ('avt_341/occupancy_grid', 'avt_341/occupancy_grid_low_res'),
                ('avt_341/occupied_cells', 'avt_341/occupied_cells_low_res')
            ]
        ),

        # Static Grid
        #Node(
        #    package='avt_341',
        #    executable='avt_341_geotiff_map_publisher_node',
        #    name='static_grid_publisher_node',
        #    parameters=[{k: LaunchConfiguration(f'static_grid_{k}') for k in params['static_grid'].keys()}]
        #),

        # Speed Controller
        *evaluate_speed_controller(params, context=context, args=args, kwargs=kwargs),

        # Global Planner
        *evaluate_global_planner(params, context=context, args=args, kwargs=kwargs),

        # Local Planner
        *evaluate_local_planner(params, context=context, args=args, kwargs=kwargs),

        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(auto_launch_rviz),
            arguments=["-d", TernarySubstitution(true_val=TextSubstitution(text=rviz_mult_config.perform(context)),
                                                false_val=TextSubstitution(text=rviz_config.perform(context)),
                                                condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1 or ', LaunchConfiguration('namespace_single_vehicle')])))]
        ),

        # Data Acquisition
        Node(
            package='avt_341',
            executable='data_acquisition_node',
            name='data_acquisition_node',
            output='log',
            parameters=[{k: LaunchConfiguration(f'data_acquisition_{k}') for k in params['data_acquisition'].keys()}]
        ),

        # Mission Manager
        Node(
            package='avt_341',
            executable='avt_341_mission_manager_node',
            name='mission_manager_node',
            output='screen',
            parameters=[
                {
                    'name': ToUpper(ArrayIndexSubstitution(vehicle_namespaces, idx)),
                    'vehicle_namespaces': vehicle_namespaces,
                    "max_speed": LaunchConfiguration(f'mpc_local_planner_max_speed')
                },
                {k: LaunchConfiguration(f'mission_manager_{k}') for k in params['mission_manager'].keys()},
                #{k: LaunchConfiguration(v) for k, v in param_refs['mission_manager'].items()}
            ],
            remappings=[
                ('avt_341/comm_messages','/avt_341/comm_messages'),
            ],
        ),

        # Socket Communication
        Node(
            package='avt_341',
            executable='avt_341_comm_node',
            name='comm_node',
            output='screen',
            parameters=[
                {'name': ToUpper(ArrayIndexSubstitution(vehicle_namespaces, idx))},
                {k: LaunchConfiguration(f'socket_comms_{k}') for k in params['socket_comms'].keys()}
            ]
        ),

        # Speed Zones
        Node(
            package='avt_341',
            executable='avt_341_speed_zones_node',
            name='speed_zones_node',
            output='screen',
            parameters=[
                {k: LaunchConfiguration(f'speed_zones_{k}') for k in params['speed_zones'].keys()}
            ],
            remappings=[
                ('avt_341/comm_messages','/avt_341/comm_messages'),
            ],
        ),

        # UAB Perception
        Node(
            package='avt_341',
            executable='uab_perception_node',
            name='uab_perception_node',
            parameters=[
                {k: LaunchConfiguration(f'uab_perception_{k}') for k in params['uab_perception'].keys()}
            ],
            remappings=[
                ('avt_341/points','/ouster/points'),
                ('avt_341/camera/image_raw','/flir_camera/image_raw'),
                ('avt_341/camera/camera_info','/flir_camera/camera_info'),
                ('avt_341/odom','avt_341/odometry'),
                ('avt_341/occupancy_grid','avt_341/terrain_seg/occupancy_grid'),
                ('avt_341/segmentation_grid','avt_341/terrain_seg/segmentation_grid'),
            ],
            output='screen'
        ),

        # Obstacle Detection
        Node(
            package='avt_341',
            executable='avt_341_object_detector_node',
            name='object_detector_node',
            namespace='toi',
            parameters=[
                {k: LaunchConfiguration(f'object_detector_{k}') for k in params['object_detector'].keys()}
            ],
            remappings=[
                ('image','/flir_camera/image_raw'),
            ],
            output='screen'
        ),

        # FEDA Detection
        Node(
            package='avt_341',
            executable='avt_341_object_detector_node',
            name='feda_detector_node',
            namespace='feda_detector',
            parameters=[
                {k: LaunchConfiguration(f'feda_detector_{k}') for k in params['feda_detector'].keys()}
            ],
            remappings=[
                ('image','/flir_camera/image_raw'),
            ],
            output='screen'
        ),

        # Object Tracking
        Node(
            package='avt_341',
            executable='avt_341_object_tracking_node',
            name='object_tracking_node',
            namespace='toi',
            parameters=[
                {k: LaunchConfiguration(f'object_tracking_{k}') for k in params['object_tracking'].keys()}
            ],
            remappings=[
                # Subscribers
                ('camera_info','/flir_camera/camera_info'),
                ('image','/flir_camera/image_raw'),
                ('points/input','/ouster/points'),
                ('detection_2d', 'detections/vision'),
                ('avt_341/reset', '/mrzr/avt_341/reset'),
                ('task','/mrzr/avt_341/mission_task_state'),
                # Publishers
                ('avt_341/odometry/estimated/odom','odometry/estimated'),
                ('avt_341/reset_ack','/mrzr/avt_341/reset_ack'),
            ],
            output='screen'
        ),

        # Vehicle Logging
        GroupAction(condition=IfCondition(enable_logging), actions=[
            ExecuteProcess(
                cmd=[
                    'ros2','run','avt_341','vehicle_logging.py',
                    f"{get_package_share_directory('avt_341')}/parameters/config_mrzr/vehicle_logging.yaml",
                    logging_path.perform(context),
                    '--bag_format', 'mcap'
                ],
                output='screen'
            )
        ])

    ])
    
    return [*arg_list, *vehicle_node_list]

def generate_launch_description():
    launch_arg_defaults = {
        "use_sim_time":                 "False",
        "auto_launch_rviz":             "True",
        "display_type":                 "rviz",
        "waypoints_file":               f"{get_package_share_directory('avt_341')}/config/no_waypoints.yaml",
        "robot_description_file":       f"{get_package_share_directory('avt_341')}/config/MRZR.urdf",
        "robot_description_veh2_file":  "",
        "robot_description_veh3_file":  "",
        "robot_description_veh4_file":  "",
        "veh_index":                    "0",
        "num_vehicles":                 "1",
        "namespace_single_vehicle":     "False",
        "vehicle_namespaces":           "['agv1', 'agv2', 'cgv1', 'cgv2']",
        "vehicle_config_folders":       f"['{get_package_share_directory('avt_341')}/parameters/config_mrzr', '{get_package_share_directory('avt_341')}/parameters/config_mrzr', '{get_package_share_directory('avt_341')}/parameters/config_mrzr', '{get_package_share_directory('avt_341')}/parameters/config_mrzr']",
        "use_rqt_display":              "False",
        "rviz_config":                  f"{get_package_share_directory('avt_341')}/rviz/avt_341_ros2.rviz",
        "rviz_mult_config":             f"{get_package_share_directory('avt_341')}/rviz/avt_341_multi_vehicle_ros2.rviz",
        "use_lidar_obstacle_detector":  "False",
        "local_planner_method":         "rcc",
        "waypoint_mode":                "False",
        "simulation_mode":              "False",
        "use_global_path":              "True",
        "global_planner_method":        "a_star",
        "enable_logging":               "False",
        "logging_path":                 os.path.join(os.path.expanduser('~'),"avt_341_data"),
    }
    launch_args = []
    for arg,default in launch_arg_defaults.items():
        launch_args.append(DeclareLaunchArgument(arg, default_value=default))

    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup),
    ])