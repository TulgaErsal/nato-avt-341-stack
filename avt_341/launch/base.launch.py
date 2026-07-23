import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter

from avt_341_param_lib.launch_params import (
    ParameterCollection,
    perform_yaml,
    relevant_params_files,
)
from avt_341_param_lib.launch_metadata import MetadataCollection
from avt_341_param_lib.parse_runtime_yaml import resolve_params_files

SHARE_DIR = get_package_share_directory('avt_341').replace('\\', '/')
TEMPLATES_DIR = os.path.join(SHARE_DIR, 'parameters', 'templates')


def _template(name):
    return os.path.join(TEMPLATES_DIR, name)


def _is_true(text):
    return str(text).strip().lower() in ('true', '1')


class NodeSpec:
    """Static launch specification for one node replicated per vehicle."""

    def __init__(self, executable, template=None, condition=None, sub_ns=None,
                 extra_params=None, output='screen'):
        self.executable = executable
        self.template = template
        self.condition = condition          # cfg dict -> bool; None = always
        self.sub_ns = sub_ns                # sub-namespace below the vehicle namespace
        self.extra_params = extra_params    # callable(vid, vehicles) -> dict of launch-computed params
        self.output = output


NODES = {
    # Lidar perception (three instances of one executable; per-instance values
    # come from the params_files override sections)
    'perception_local_node': NodeSpec(
        'avt_341_perception_node', _template('perception.yaml')),
    'perception_global_node': NodeSpec(
        'avt_341_perception_node', _template('perception.yaml')),
    'perception_rms_node': NodeSpec(
        'avt_341_perception_node', _template('perception.yaml')),
    'lidar_obstacle_detector_node': NodeSpec(
        'avt_341_lidar_obstacle_detector_node', _template('obstacle_detector.yaml'),
        condition=lambda cfg: _is_true(cfg['use_lidar_obstacle_detector'])),
    'grid_compression_local': NodeSpec('avt_341_grid_compression_node'),
    'grid_compression_global': NodeSpec('avt_341_grid_compression_node'),

    # Controllers (selected by local planner method)
    'speed_control_node': NodeSpec(
        'avt_341_speed_control_node', _template('speed_control.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] in ('dwa', 'mpc')),
    'control_node': NodeSpec(
        'avt_341_control_node', _template('control.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] in ('rcc', 'pf')),

    # Global planners
    'avt_341_global_path_node': NodeSpec(
        'avt_341_global_path_node', _template('global_planner.yaml'),
        condition=lambda cfg: cfg['global_planner_method'] == 'a_star',
        extra_params=lambda vid, vehicles: _waypoint_params()),
    'pf_global_path_node': NodeSpec(
        'avt_341_pf_global_path_node', _template('pf_global_planner.yaml'),
        condition=lambda cfg: cfg['global_planner_method'] == 'pf',
        extra_params=lambda vid, vehicles: _waypoint_params()),

    # Local planners (selected by local planner method)
    'local_planner_node': NodeSpec(
        'avt_341_local_planner_node', _template('rcc_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'rcc'),
    'local_dwa_planner_node': NodeSpec(
        'avt_341_dwa_planner_node', _template('dwa_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'dwa'),
    'local_pf_planner_node': NodeSpec(
        'avt_341_pf_planner_node', _template('pf_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'pf'),
    'mpc_planner_node': NodeSpec(
        'avt_341_mpc_planner_node', _template('mpc_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'mpc'),
    'obstacle_processor_node': NodeSpec(
        'obstacle_processor_node', _template('mpc_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'mpc'),
    'segmentation_grid_processor_node': NodeSpec(
        'segmentation_grid_processor_node', _template('mpc_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'mpc'),
    'goal_point_processor_node': NodeSpec(
        'goal_point_processor_node', _template('mpc_local_planner.yaml'),
        condition=lambda cfg: cfg['local_planner_method'] == 'mpc'),
    'avt_341_veh_converter_node': NodeSpec(
        'veh_converter_node',
        condition=lambda cfg: cfg['local_planner_method'] == 'mpc'),

    # Data acquisition, mission management and communication
    'data_acquisition_node': NodeSpec(
        'data_acquisition_node', _template('data_acquisition.yaml'), output='log'),
    'mission_manager_node': NodeSpec(
        'avt_341_mission_manager_node', _template('mission_manager.yaml'),
        extra_params=lambda vid, vehicles: {
            'name': str(vid).upper(), 'vehicle_namespaces': list(vehicles)}),
    'comm_node': NodeSpec(
        'avt_341_comm_node', _template('socket_comms.yaml'),
        extra_params=lambda vid, vehicles: {
            'name': str(vid).upper(), 'vehicle_namespaces': list(vehicles)}),
    'speed_zones_node': NodeSpec(
        'avt_341_speed_zones_node', _template('speed_zones.yaml')),

    # UAB terrain segmentation
    'uab_perception_node': NodeSpec(
        'uab_perception_node', _template('uab_perception.yaml'),
        extra_params=lambda vid, vehicles: {'frame_prefix': f'{vid}/'}),

    # Object detection and tracking
    'object_detector_node': NodeSpec(
        'avt_341_object_detector_node', _template('object_detector.yaml')),
    'object_tracking_node': NodeSpec(
        'avt_341_object_tracking_node', _template('object_tracking.yaml'),
        extra_params=lambda vid, vehicles: {
            'target_selection.formation_vehicle_ids': list(vehicles)}),
}

pargs = ParameterCollection.from_node_templates(
    {name: spec.template for name, spec in NODES.items() if spec.template})

_waypoint_params_cache = {}


def _waypoint_params():
    """Waypoint parameters loaded from the waypoints file; {} when empty/disabled."""
    return dict(_waypoint_params_cache)


def _load_waypoint_params(context):
    _waypoint_params_cache.clear()
    if not _is_true(LaunchConfiguration('waypoint_mode').perform(context)):
        return
    waypoints_file = LaunchConfiguration('waypoints_file').perform(context)
    if not waypoints_file or not os.path.isfile(waypoints_file):
        return
    with open(waypoints_file) as f:
        doc = yaml.safe_load(f) or {}
    waypoints_x = doc.get('waypoints_x') or []
    waypoints_y = doc.get('waypoints_y') or []
    if waypoints_x and waypoints_y:
        # never inject empty lists; the template default [] means "no waypoints"
        _waypoint_params_cache['waypoints_x'] = [float(x) for x in waypoints_x]
        _waypoint_params_cache['waypoints_y'] = [float(y) for y in waypoints_y]


def _make_node(name, spec, vid, vehicles, params_files, overrides, metadata):
    node_name = name.rsplit('/', 1)[-1]
    namespace_parts = [str(vid).strip('/')]
    if spec.sub_ns:
        namespace_parts.extend(str(spec.sub_ns).strip('/').split('/'))
    fqn = '/' + '/'.join([*namespace_parts, node_name])
    layers = list(relevant_params_files(params_files, fqn))
    if spec.extra_params is not None:
        extra = spec.extra_params(vid, vehicles)
        if extra:
            layers.append(extra)
    cli_params = overrides.for_node(fqn)
    if cli_params:
        layers.append(cli_params)
    node_remappings = metadata.get_remappings(fqn)
    node_additional_env = metadata.get_additional_env(fqn)
    node = Node(
        package='avt_341',
        executable=spec.executable,
        name=node_name,
        output=spec.output,
        parameters=layers or None,
        remappings=node_remappings or None,
        additional_env=node_additional_env or None,
    )
    if spec.sub_ns:
        return GroupAction([PushRosNamespace(spec.sub_ns), node])
    return node


def _robot_state_publisher(vid, vehicle_index, context, metadata):
    description_files = perform_yaml(context, 'robot_description_files') or []
    if not description_files:
        return None
    index = vehicle_index if vehicle_index < len(description_files) else 0
    description_file = str(description_files[index])
    if not description_file or not os.path.isfile(description_file):
        return None
    with open(description_file) as f:
        robot_description = f.read()
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'frame_prefix': f'{vid}/'}],
        remappings=metadata.get_remappings(
            f'/{vid}/robot_state_publisher') or None,
        additional_env=metadata.get_additional_env(
            f'/{vid}/robot_state_publisher') or None,
    )


def _spawn_vehicles(context, *args, **kwargs):
    vehicles = [str(vid).strip('/') for vid in perform_yaml(context, 'vehicle_ids')]
    params_files = resolve_params_files(
        perform_yaml(context, 'params_files') or [], vehicles)
    metadata = MetadataCollection(
        LaunchConfiguration('metadata_file').perform(context))
    overrides = pargs.resolve_cli_overrides(context, vehicles)
    cfg = {
        key: LaunchConfiguration(key).perform(context)
        for key in ('local_planner_method', 'global_planner_method',
                    'use_lidar_obstacle_detector')
    }
    _load_waypoint_params(context)

    actions = [SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))]
    for vehicle_index, vid in enumerate(vehicles):
        group = [PushRosNamespace(vid)]
        for name, spec in NODES.items():
            if spec.condition is not None and not spec.condition(cfg):
                continue
            group.append(_make_node(
                name, spec, vid, vehicles, params_files, overrides, metadata))
        state_publisher = _robot_state_publisher(
            vid, vehicle_index, context, metadata)
        if state_publisher is not None:
            group.append(state_publisher)
        actions.append(GroupAction(group))

    # Visualization (single instance, multi-vehicle config when several vehicles)
    if _is_true(LaunchConfiguration('auto_launch_rviz').perform(context)):
        rviz_arg = 'rviz_mult_config' if len(vehicles) > 1 else 'rviz_config'
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration(rviz_arg).perform(context)],
            remappings=metadata.get_remappings('/rviz2') or None,
            additional_env=metadata.get_additional_env('/rviz2') or None,
        ))

    # Standardized vehicle logging for V&V efforts
    if _is_true(LaunchConfiguration('enable_logging').perform(context)):
        actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'avt_341', 'vehicle_logging.py',
                f'{SHARE_DIR}/parameters/bag_config/rw_bag_config.yaml',
                LaunchConfiguration('logging_path').perform(context),
                '--bag_format', 'mcap',
            ],
            output='screen',
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_ids',
            description='List of vehicle ids used as namespaces; the node set is '
                        'replicated per vehicle. No default: the including launch '
                        'file (e.g. krc_mrzr_client.launch.py) supplies the list.'),
        DeclareLaunchArgument(
            'params_files',
            default_value=f"['{SHARE_DIR}/parameters/overrides/krc_mrzr.yaml']",
            description='Ordered list of runtime parameter yaml files (later files '
                        'override earlier ones; $python{}/$ref{} templating supported)'),
        DeclareLaunchArgument(
            'metadata_file',
            default_value=f'{SHARE_DIR}/parameters/metadata/krc_mrzr.yaml',
            description='One launch-only node metadata YAML file containing topic '
                        'remappings and additional environment variables; use an '
                        'empty value to disable node metadata'),
        DeclareLaunchArgument('use_sim_time', default_value='False',
                              description='Use simulation time (mainly for rosbag data replay)'),
        DeclareLaunchArgument('local_planner_method', default_value='rcc',
                              description='Local planner method. Values = [rcc, dwa, mpc, pf]'),
        DeclareLaunchArgument('global_planner_method', default_value='a_star',
                              description='Global planner method. Values = [a_star, pf]'),
        DeclareLaunchArgument('use_lidar_obstacle_detector', default_value='False',
                              description='Enable lidar obstacle segmentation'),
        DeclareLaunchArgument('waypoint_mode', default_value='False',
                              description='Set to true for waypoint following mode'),
        DeclareLaunchArgument('waypoints_file',
                              default_value=f'{SHARE_DIR}/config/no_waypoints.yaml',
                              description='Path to waypoint yaml file (waypoints_x/waypoints_y lists)'),
        DeclareLaunchArgument('robot_description_files',
                              default_value=f"['{SHARE_DIR}/config/MRZR.urdf']",
                              description='List of URDF files, one per vehicle (first entry reused '
                                          'when fewer files than vehicles are given)'),
        DeclareLaunchArgument('auto_launch_rviz', default_value='True',
                              description='Automatically launch rviz display window'),
        DeclareLaunchArgument('rviz_config',
                              default_value=f'{SHARE_DIR}/rviz/avt_341_ros2.rviz',
                              description='Single vehicle rviz config file'),
        DeclareLaunchArgument('rviz_mult_config',
                              default_value=f'{SHARE_DIR}/rviz/avt_341_multi_vehicle_ros2.rviz',
                              description='Multiple vehicle rviz config file'),
        DeclareLaunchArgument('enable_logging', default_value='False',
                              description='Enable standardized vehicle logging for V&V efforts'),
        DeclareLaunchArgument('logging_path',
                              default_value=os.path.join(os.path.expanduser('~'), 'avt_341_data'),
                              description='Save path for vehicle logging'),
        *pargs.declare_arguments(),
        OpaqueFunction(function=_spawn_vehicles),
    ])
