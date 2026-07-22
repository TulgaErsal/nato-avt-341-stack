"""Client launch file that includes base.launch.py.

Demonstrates the optional client-side re-declaration pattern: the parameter
override arguments are re-declared here (so this file documents them
explicitly) with a scrub that removes non-provided defaults again before the
include, keeping the base file's explicitness detection exact. The scrub only
touches this instance's argument names; arguments of any other included launch
files keep the default launch behavior.

A client that does not care about re-declaring can simply drop the
client_declare_actions() line: command line arguments pass through a bare
include untouched, and `ros2 launch ... -s` lists the base file's arguments via
recursion anyway.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from avt_341_param_lib.launch_params import ParameterCollection

SHARE_DIR = get_package_share_directory('avt_341_param_lib_example').replace('\\', '/')

pargs = ParameterCollection.from_template_files([
    os.path.join(SHARE_DIR, 'parameters', 'nav_params.yaml'),
    os.path.join(SHARE_DIR, 'parameters', 'sensor.yaml'),
])


def generate_launch_description():
    return LaunchDescription([
        *pargs.client_declare_actions(),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SHARE_DIR, 'launch', 'base.launch.py'))),
    ])
