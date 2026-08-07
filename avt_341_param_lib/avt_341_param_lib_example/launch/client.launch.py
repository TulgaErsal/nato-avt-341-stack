"""Client launch file that includes base.launch.py.

The client owns the deployment: it supplies the list of vehicle ids to the
base file (which declares ``vehicle_ids`` without a default) while remaining
overridable from the command line (``vehicle_ids:=[veh1, veh3]``).

It also demonstrates the optional client-side re-declaration pattern: the
parameter override arguments are re-declared here (so this file documents them
explicitly) with a scrub that removes non-provided defaults again before the
include, keeping the base file's explicitness detection exact. The scrub only
touches this instance's argument names; arguments of any other included launch
files keep the default launch behavior.

A client that does not care about re-declaring can simply drop the
client_declare_arguments() line: command line arguments pass through a bare
include untouched, and `ros2 launch ... -s` lists the base file's arguments via
recursion anyway.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from avt_341_param_lib.runtime.launch_params import ParameterCollection

SHARE_DIR = get_package_share_directory('avt_341_param_lib_example').replace('\\', '/')
PARAMS_DIR = os.path.join(SHARE_DIR, 'parameters')

# mirrors the node -> template assignment of base.launch.py
pargs = ParameterCollection.from_node_templates({
    'planner': os.path.join(PARAMS_DIR, 'nav.yaml'),
    'controller': os.path.join(PARAMS_DIR, 'nav.yaml'),
    'sensor': os.path.join(PARAMS_DIR, 'sensor.yaml'),
    'mixin_ex': os.path.join(PARAMS_DIR, 'mixin_ex.yaml'),
})


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_ids', default_value='[veh1, veh2]',
            description='List of agent ids used as namespaces; the node set is '
                        'replicated per agent'),
        *pargs.client_declare_arguments(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SHARE_DIR, 'launch', 'base.launch.py')),
            launch_arguments=[('vehicle_ids', LaunchConfiguration('vehicle_ids'))]),
    ])
