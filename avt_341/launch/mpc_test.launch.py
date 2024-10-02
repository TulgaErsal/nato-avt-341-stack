import os
import yaml
import json

import launch.conditions
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationNotEquals, LaunchConfigurationEquals
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetLaunchConfiguration
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

def launch_setup(context, *args, **kwargs):
    # General args
    vehicle_config_folders = LaunchConfiguration('vehicle_config_folders')              # Directories containing all config files for each vehicle
    veh_index = LaunchConfiguration('veh_index')                                        # Current index value from vehicle_namespaces and vehicle_config_folders of the vehicle being launched

    # Vehicle index
    idx = int(veh_index.perform(context))

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
    
    vehicle_node_list = ([
        # Define robot namespace
        PushRosNamespace(
            namespace='mrzr'
        ),
        Node(
            package='avt_341',
            executable='avt_341_mpc_planner_node',
            name='mpc_planner_node',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            remappings=[
                ('avt_341/local_path', 'avt_341/local_path2'),
                ('avt_341/desired_speed', 'avt_341/desired_speed2'),
                ('avt_341/cmd_steer', 'avt_341/cmd_steer2')
            ],
            parameters=[{k: LaunchConfiguration(f'mpc_local_planner_{k}') for k in params['mpc_local_planner'].keys()}],
        ),

    ])
    
    return [*arg_list, *vehicle_node_list]

def generate_launch_description():
    launch_arg_defaults = { "veh_index":                    "0",
                            "vehicle_config_folders":       f"['{get_package_share_directory('avt_341')}/parameters/config_mrzr', '{get_package_share_directory('avt_341')}/parameters/config_mrzr', '{get_package_share_directory('avt_341')}/parameters/config_mrzr', '{get_package_share_directory('avt_341')}/parameters/config_mrzr']",
    }
    launch_args = []
    for arg,default in launch_arg_defaults.items():
        launch_args.append(DeclareLaunchArgument(arg, default_value=default))

    return LaunchDescription([
        *launch_args,
        OpaqueFunction(function=launch_setup),
    ])