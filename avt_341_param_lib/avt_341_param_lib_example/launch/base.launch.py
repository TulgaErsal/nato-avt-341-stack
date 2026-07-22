"""Base launch file demonstrating hierarchical parameter overriding.

Command line parameter overrides use the same node-selector syntax as the
runtime parameter yaml files. Examples:

  **/cruise_speed:=9.0                     all nodes of all agents
  planner/cruise_speed:=8.0                planner node of all agents
  veh1/planner/cruise_speed:=7.0           planner node of one agent
  veh1:='{cruise_speed: 3.3}'              all nodes of one agent (mapping form)
  /veh1/planner:='{planner_mode: graph}'   absolute selector, mapping form

Per-node override priority (later wins, per parameter):
  1. runtime yaml files given via ``params_files``, in list order
  2. command line overrides, in command line order
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace

from avt_341_param_lib.launch_params import (
    ParameterCollection,
    perform_yaml,
    relevant_params_files,
)

SHARE_DIR = get_package_share_directory('avt_341_param_lib_example').replace('\\', '/')
PARAMS_DIR = os.path.join(SHARE_DIR, 'parameters')

# node name -> (executable, parameter template); the templates double as the
# source of the documented override launch arguments (ros2 launch ... -s)
NODES = {
    'planner': ('planner_node', os.path.join(PARAMS_DIR, 'nav.yaml')),
    'controller': ('controller_node', os.path.join(PARAMS_DIR, 'nav.yaml')),
    'sensor': ('sensor_node', os.path.join(PARAMS_DIR, 'sensor.yaml')),
}

pargs = ParameterCollection.from_node_templates(
    {name: template for name, (_, template) in NODES.items()})


def _spawn_vehicles(context, *args, **kwargs):
    vehicles = perform_yaml(context, 'vehicle_ids')
    params_files = perform_yaml(context, 'params_files')
    overrides = pargs.resolve_overrides(context, vehicles)

    actions = []
    for vid in vehicles:
        nodes = []
        for name, (executable, _) in NODES.items():
            fqn = f"/{str(vid).strip('/')}/{name}"
            # only hand each node the inputs that can actually apply to it
            layers = list(relevant_params_files(params_files, fqn))
            cli_params = overrides.for_node(fqn)
            if cli_params:
                layers.append(cli_params)
            nodes.append(Node(
                package='avt_341_param_lib_example',
                executable=executable,
                name=name,
                output='screen',
                parameters=layers,
            ))
        actions.append(GroupAction([PushRosNamespace(vid), *nodes]))
    return actions


def generate_launch_description():
    default_params_files = '[{}, {}]'.format(
        f'{SHARE_DIR}/parameters_override/global_params.yaml',
        f'{SHARE_DIR}/parameters_override/agent_params.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_ids',
            description='List of agent ids used as namespaces; the node set is '
                        'replicated per agent. No default: the including launch '
                        'file (e.g. client.launch.py) supplies the list.'),
        DeclareLaunchArgument(
            'params_files', default_value=default_params_files,
            description='Ordered list of runtime parameter yaml files (later files override earlier ones)'),
        *pargs.declare_parameters(),
        OpaqueFunction(function=_spawn_vehicles),
    ])
