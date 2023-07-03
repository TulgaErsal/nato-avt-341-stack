import os

import launch.conditions
from launch.conditions import IfCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import OpaqueFunction

from launch.substitution import Substitution
from launch.some_substitutions_type import SomeSubstitutionsType
from launch.condition import Condition
import yaml


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
        DeclareLaunchArgument('waypoints_x', description="List of waypoint x coordinates. Will override waypoints_file is specified.", default_value=waypoints_x),
        DeclareLaunchArgument('waypoints_y', description="List of waypoint y coordinates. Will override waypoints_file is specified.", default_value=waypoints_y),
        DeclareLaunchArgument('is_empty_waypoints',
                              description="Parameter set internally to detect if waypoints file empty. ROS2 foxy workaround (https://answers.ros.org/question/396556/what-is-best-practice-for-parameters-which-are-empty-lists-in-ros2/). Do not set manually",
                              default_value=str(is_empty_waypoints).capitalize()),
    ]


PYTHON_EVAL_STR = '$Python:'


def generate_launch_description():

    MAX_VEHICLES = 4
    use_sim_time = LaunchConfiguration('use_sim_time')
    auto_launch_rviz = LaunchConfiguration("auto_launch_rviz")
    display_type = LaunchConfiguration('display_type')

    rviz_config_single_vehicle = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_ros2.rviz')
    rviz_config_multi_vehicle = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_multi_vehicle_ros2.rviz')
    rviz_config_two_vehicle = os.path.join(get_package_share_directory('avt_341'), 'rviz', 'avt_341_two_vehicle_ros2.rviz')

    robot_desc_list = [LaunchConfiguration('robot_description'), LaunchConfiguration('robot_description_veh2'),
                       LaunchConfiguration('robot_description_veh3'), LaunchConfiguration('robot_description_veh4')]

    param_dir = os.path.join(get_package_share_directory('avt_341'), 'config', 'parameters')
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
            if type(vi) is str and vi.startswith(PYTHON_EVAL_STR):
                python_str = vi[len(PYTHON_EVAL_STR):]
                params[k][ki] = eval(python_str)
            if type(vi) is str and vi.startswith("$val{"):
                key_sub = vi[5:-1].split(':')
                params[k][ki] = params[key_sub[0]][key_sub[1]]
            if type(vi) is str and vi.startswith("$ref{"):
                param_refs[k][ki] = vi[5:-1]
        for ki in param_refs[k].keys():
            del params[k][ki]

    arg_list = [DeclareLaunchArgument(ki, default_value=str(vi)) for k, v in params.items() for ki, vi in v.items()]

    vehicle_node_list = []
    for idx in range(MAX_VEHICLES):
        vehicle_node_list.append(
            GroupAction(condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > %d' % idx])), actions=[
                PushRosNamespace(
                    condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1 or ', LaunchConfiguration('namespace_single_vehicle')])),
                    namespace=ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx)),
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc_list[idx],
                                 'frame_prefix': TernarySubstitution(Concat(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx), '/'),
                                                                     TextSubstitution(text=''),
                                                                     IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1'])))}]
                ),
                Node(
                    package='avt_341',
                    executable='avt_bot_state_publisher_node',
                    name='state_publisher',
                    parameters=[{'frame_prefix': TernarySubstitution(Concat(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx), '/'),
                                                                     TextSubstitution(text=''),
                                                                     IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1'])))}]
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_perception_node',
                    name='perception_node',
                    output='screen',
                    parameters=[
                        {'display': display_type},
                        {k: launch.substitutions.LaunchConfiguration(k) for k in params['perception'].keys()}],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_control_node',
                    name='vehicle_control_node',
                    output='screen',
                    condition=LaunchConfigurationNotEquals('local_planner_method', 'dwa'),
                    parameters=[{k: launch.substitutions.LaunchConfiguration(k) for k in params['control'].keys()}],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_speed_control_node',
                    name='vehicle_control_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'dwa'),
                    parameters=[{k: launch.substitutions.LaunchConfiguration(k) for k in params['control'].keys()}],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_global_path_node',
                    name='avt_341_global_path_node',
                    output='screen',
                    parameters=[
                        {
                            'display': display_type,
                            '/waypoints_x': launch.substitutions.LaunchConfiguration('waypoints_x'),
                            '/waypoints_y': launch.substitutions.LaunchConfiguration('waypoints_y'),
                            '/is_empty_waypoints': launch.substitutions.LaunchConfiguration('is_empty_waypoints'),
                        },
                        {k: launch.substitutions.LaunchConfiguration(k) for k in params['global_planner'].keys()}],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_local_planner_node',
                    name='local_planner_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'rcc'),
                    parameters=[
                        {'display': display_type},
                        {k: launch.substitutions.LaunchConfiguration(k) for k in params['local_planner'].keys()}
                    ],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_dwa_planner_node',
                    name='local_dwa_planner_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'dwa'),
                    parameters=[{k: launch.substitutions.LaunchConfiguration(k) for k in params['local_planner'].keys()}],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_pf_planner_node',
                    name='local_pf_planner_node',
                    output='screen',
                    condition=LaunchConfigurationEquals('local_planner_method', 'pf'),
                    parameters=[
                        {'display': display_type},
                        {k: launch.substitutions.LaunchConfiguration(k) for k in params['local_planner'].keys()}
                    ],
                ),
                Node(
                    package='avt_341',
                    executable='avt_341_grid_compression_node',
                    name='grid_compression'),
                GroupAction(condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1'])), actions=[
                    Node(
                        package='avt_341',
                        executable='avt_341_mission_manager_node',
                        name='mission_manager_node',
                        output='screen',
                        parameters=[{
                            'name': ToUpper(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx)),
                            },
                            {k: launch.substitutions.LaunchConfiguration(k) for k in params['mission_manager'].keys()},
                            {k: launch.substitutions.LaunchConfiguration(v) for k, v in param_refs['mission_manager'].items()}
                        ]
                    ),
                    Node(
                        package='avt_341',
                        executable='avt_341_comm_node',
                        name='comm_node',
                        output='screen',
                        parameters=[{
                            'name': ToUpper(ArrayIndexSubstitution(LaunchConfiguration('vehicle_namespaces'), idx)),
                            },
                            {k: launch.substitutions.LaunchConfiguration(k) for k in params['socket_comms'].keys()}
                        ]
                    )
                ])
            ])
        )

    launch_description = LaunchDescription([
        *arg_list,
        OpaqueFunction(function=evaluate_waypoint_parameters),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]),
        *vehicle_node_list,
        # DeclareLaunchArgument('rviz_config', default_value=rviz_config_single_vehicle, description='Full path to rviz config file'),
        # SetLaunchConfiguration('rviz_config', rviz_config_multi_vehicle, condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1']))),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            condition=IfCondition(auto_launch_rviz),
            arguments=["-d", TernarySubstitution(true_val=TextSubstitution(text=rviz_config_multi_vehicle),
                                                 false_val=TernarySubstitution(true_val=TextSubstitution(text=rviz_config_two_vehicle),
                                                                               false_val=TextSubstitution(text=rviz_config_single_vehicle),
                                                                               condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 1']))),
                                                 condition=IfCondition(PythonExpression([LaunchConfiguration('num_vehicles'), ' > 2'])))]
        )
    ])

    return launch_description
