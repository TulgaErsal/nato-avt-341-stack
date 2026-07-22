"""Base launch file demonstrating hierarchical parameter overriding.

Per-node override priority (later wins, per parameter):
  1. runtime yaml files given via ``params_files``, in list order (agent-specific
     sections are expressed inside the files with namespace wildcards)
  2. explicitly provided global launch arguments (``nav/...``, ``sensor/...``)
  3. agent-specific command line overrides (``<vehicle_id>_overrides:='{...}'``)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace

from avt_341_param_lib.launch_params import (
    ParameterCollection,
    perform_yaml,
    vehicle_overrides,
)

SHARE_DIR = get_package_share_directory('avt_341_param_lib_example').replace('\\', '/')

pargs = ParameterCollection.from_template_folder(os.path.join(SHARE_DIR, 'parameters'))


def _node(executable, name, parameter_layers):
    return Node(
        package='avt_341_param_lib_example',
        executable=executable,
        name=name,
        output='screen',
        parameters=parameter_layers,
    )


def _spawn_vehicles(context, *args, **kwargs):
    pargs.validate_explicit(context)

    vehicles = perform_yaml(context, 'vehicle_namespaces')
    params_files = perform_yaml(context, 'params_files')
    nav_cli = pargs.explicit_overrides(context, 'nav')
    sensor_cli = pargs.explicit_overrides(context, 'sensor')

    actions = []
    for vid in vehicles:
        veh_cli = vehicle_overrides(context, vid)

        def layers(partition_cli):
            result = [*params_files]
            if partition_cli:
                result.append(dict(partition_cli))
            if veh_cli:
                result.append(dict(veh_cli))
            return result

        actions.append(GroupAction([
            PushRosNamespace(vid),
            _node('planner_node', 'planner', layers(nav_cli)),
            _node('controller_node', 'controller', layers(nav_cli)),
            _node('sensor_node', 'sensor', layers(sensor_cli)),
        ]))
    return actions


def generate_launch_description():
    default_params_files = '[{}, {}]'.format(
        f'{SHARE_DIR}/parameters_override/global_params.yaml',
        f'{SHARE_DIR}/parameters_override/agent_params.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_namespaces', default_value='[veh1, veh2]',
            description='List of agent namespaces; the node set is replicated per agent'),
        DeclareLaunchArgument(
            'params_files', default_value=default_params_files,
            description='Ordered list of runtime parameter yaml files (later files override earlier ones)'),
        *pargs.declare_parameters(),
        OpaqueFunction(function=_spawn_vehicles),
    ])
