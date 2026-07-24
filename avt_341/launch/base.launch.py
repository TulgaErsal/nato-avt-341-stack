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


def is_cfg(key, negate=False):
    """Condition: launch configuration `key` is true (false when negate=True)."""
    def condition(context):
        value = _is_true(LaunchConfiguration(key).perform(context))
        return not value if negate else value
    return condition

def is_not_cfg(key):
    return is_cfg(key, negate=True)

def is_local_planner(*methods):
    """Condition: the local_planner_method launch configuration is one of `methods`."""
    def condition(context):
        return LaunchConfiguration('local_planner_method').perform(context) in methods
    return condition


class NodeSpec:
    """Static launch specification for one node replicated per vehicle."""

    def __init__(self, executable, template=None, condition=None, sub_ns=None,
                 extra_params=None, output='screen', autonomy=True):
        self.executable = executable
        self.template = template
        self.condition = condition          # callable(context) -> bool; None = always
        self.sub_ns = sub_ns                # sub-namespace below the vehicle namespace
        self.extra_params = extra_params    # callable(vid, vehicles) -> dict of launch-computed params
        self.output = output
        self.autonomy = autonomy            # autonomy nodes are skipped for manual_control_vehicles


NODES = {

    # Mission management
    'mission_manager_node':             NodeSpec('avt_341_mission_manager_node',         _template('mission_manager.yaml'),                                                       extra_params=lambda vid, vehicles: {'name': str(vid).upper(), 'vehicle_namespaces': list(vehicles)},               autonomy=False),
    'comm_node':                        NodeSpec('avt_341_comm_node',                    _template('socket_comms.yaml'),                                                          extra_params=lambda vid, vehicles: {'name': str(vid).upper(), 'vehicle_namespaces': list(vehicles)},               autonomy=False),
    'speed_zones_node':                 NodeSpec('avt_341_speed_zones_node',             _template('speed_zones.yaml'),       condition=is_cfg('use_speed_zones')),

    # Perception - Costmaps
    'perception_local_node':            NodeSpec('avt_341_perception_node',              _template('perception.yaml'),        condition=is_cfg('use_dual_costmaps')),
    'perception_global_node':           NodeSpec('avt_341_perception_node',              _template('perception.yaml'),        condition=is_cfg('use_dual_costmaps')),
    'perception_node':                  NodeSpec('avt_341_perception_node',              _template('perception.yaml'),        condition=is_not_cfg('use_dual_costmaps')),
    'perception_rms_node':              NodeSpec('avt_341_perception_node',              _template('perception.yaml'),        condition=is_cfg('use_perception_rms')),

    # Perception - Terrain segmentation
    'uab_perception_node':              NodeSpec('uab_perception_node',                  _template('uab_perception.yaml'),    condition=is_cfg('use_uab_perception'),             extra_params=lambda vid, vehicles: {'frame_prefix': f'{vid}/'}),

    # Perception - Object detection and tracking
    'object_detector_node':             NodeSpec('avt_341_object_detector_node',         _template('object_detector.yaml'),   condition=is_cfg('use_obj_detector')),
    'object_tracking_node':             NodeSpec('avt_341_object_tracking_node',         _template('object_tracking.yaml'),   condition=is_cfg('use_object_tracker'),             extra_params=lambda vid, vehicles: {'target_selection.formation_vehicle_ids': list(vehicles)}),
    'lidar_obstacle_detector_node':     NodeSpec('avt_341_lidar_obstacle_detector_node', _template('obstacle_detector.yaml'), condition=is_cfg('use_lidar_obstacle_detector')),

    # Global planners
    'avt_341_global_path_node':         NodeSpec('avt_341_global_path_node',             _template('global_planner.yaml'),                                                        extra_params=lambda vid, vehicles: _waypoint_params()),

    # Local planners
    'local_planner_node':               NodeSpec('avt_341_local_planner_node',           _template('rcc_local_planner.yaml'), condition=is_local_planner('rcc')),
    'local_dwa_planner_node':           NodeSpec('avt_341_dwa_planner_node',             _template('dwa_local_planner.yaml'), condition=is_local_planner('dwa')),
    'local_pf_planner_node':            NodeSpec('avt_341_pf_planner_node',              _template('pf_local_planner.yaml'),  condition=is_local_planner('pf')),
    'mpc_planner_node':                 NodeSpec('avt_341_mpc_planner_node',             _template('mpc_local_planner.yaml'), condition=is_local_planner('mpc')),

    # local planners - MPC supporting nodes
    'obstacle_processor_node':          NodeSpec('obstacle_processor_node',              _template('mpc_local_planner.yaml'), condition=is_local_planner('mpc')),
    'segmentation_grid_processor_node': NodeSpec('segmentation_grid_processor_node',     _template('mpc_local_planner.yaml'), condition=is_local_planner('mpc')),
    'goal_point_processor_node':        NodeSpec('goal_point_processor_node',            _template('mpc_local_planner.yaml'), condition=is_local_planner('mpc')),
    'avt_341_veh_converter_node':       NodeSpec('veh_converter_node',                                                        condition=is_local_planner('mpc')),

    # Controllers (selected by local planner method)
    'speed_control_node':               NodeSpec('avt_341_speed_control_node',           _template('speed_control.yaml'),     condition=is_local_planner('dwa', 'mpc')),
    'control_node':                     NodeSpec('avt_341_control_node',                 _template('control.yaml'),           condition=is_local_planner('rcc', 'pf')),

    # State pre-processing
    'data_acquisition_node':            NodeSpec('data_acquisition_node',                _template('data_acquisition.yaml'),  condition=is_cfg('use_data_acquisition'),                                                                                                                output='log', autonomy=False),
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
        perform_yaml(context, 'ros_param_files') or [], vehicles)
    metadata = MetadataCollection(
        LaunchConfiguration('node_config_file').perform(context))
    overrides = pargs.resolve_cli_overrides(context, vehicles)
    spawn_filter = {
        str(vid).strip('/')
        for vid in perform_yaml(context, 'spawn_filter_vehicle_ids') or []}
    manual_vehicles = {
        str(vid).strip('/')
        for vid in perform_yaml(context, 'manual_control_vehicles') or []}
    publish_urdf_to_tf = _is_true(
        LaunchConfiguration('publish_urdf_to_tf').perform(context))
    _load_waypoint_params(context)

    actions = [SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time'))]
    spawned_vehicles = [
        vid for vid in vehicles if not spawn_filter or vid in spawn_filter]
    for vehicle_index, vid in enumerate(vehicles):
        if vid not in spawned_vehicles:
            continue
        group = [PushRosNamespace(vid)]
        for name, spec in NODES.items():
            if spec.autonomy and vid in manual_vehicles:
                continue
            if spec.condition is not None and not spec.condition(context):
                continue
            group.append(_make_node(
                name, spec, vid, vehicles, params_files, overrides, metadata))
        if publish_urdf_to_tf:
            state_publisher = _robot_state_publisher(
                vid, vehicle_index, context, metadata)
            if state_publisher is not None:
                group.append(state_publisher)
        actions.append(GroupAction(group))

    # Visualization (single instance, multi-vehicle config when several vehicles)
    if _is_true(LaunchConfiguration('auto_launch_rviz').perform(context)):
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config').perform(context)],
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

        # Vehicle selection
        DeclareLaunchArgument('vehicle_ids',                                                                                      description='List of all vehicle ids in formation.'),
        DeclareLaunchArgument('spawn_filter_vehicle_ids',    default_value='[]',                                                  description='Subset of vehicle_ids to actually create nodes for. If empty, spawns all vehicles in vehicle_ids'),
        DeclareLaunchArgument('manual_control_vehicles',     default_value='[]',                                                  description='List of vehicle ids under manual human control.'),
        DeclareLaunchArgument('robot_description_files',     default_value=f"['{SHARE_DIR}/config/MRZR.urdf']",                   description='List of URDF files, one per vehicle (first entry reused when fewer files than vehicles are given)'),

        # Parameter files
        DeclareLaunchArgument('ros_param_files',             default_value='[]',                                                  description='Parameter files used to override default ROS arguments'),
        DeclareLaunchArgument('node_config_file',            default_value='',                                                    description='Additional node configuration parameters (topic remappings, env. variables, etc.)'),

        DeclareLaunchArgument('use_sim_time',                default_value='False',                                               description='Use simulation time (mainly for rosbag data replay)'),

        # Feature flags
        DeclareLaunchArgument('local_planner_method',        default_value='rcc',                                                 description='Local planner method. Values = [rcc, dwa, mpc, pf]'),
        DeclareLaunchArgument('use_lidar_obstacle_detector', default_value='False',                                               description='Enable lidar obstacle segmentation'),
        DeclareLaunchArgument('use_dual_costmaps',           default_value='True',                                                description='Spawn the dual global and local costmap perception nodes; when false a single perception_node is spawned instead'),
        DeclareLaunchArgument('use_perception_rms',          default_value='True',                                                description='Enable the RMS perception node'),
        DeclareLaunchArgument('use_speed_zones',             default_value='True',                                                description='Enable speed zones'),
        DeclareLaunchArgument('use_uab_perception',          default_value='True',                                                description='Enable the UAB terrain segmentation perception node'),
        DeclareLaunchArgument('use_obj_detector',            default_value='True',                                                description='Enable 2d bounding box detection of static objects using deep neural network inference'),
        DeclareLaunchArgument('use_object_tracker',          default_value='True',                                                description='Enable the object tracking node'),
        DeclareLaunchArgument('use_data_acquisition',        default_value='True',                                                description='Enable the data acquisition node'),
        DeclareLaunchArgument('publish_urdf_to_tf',          default_value='True',                                                description='Publish the robot URDF description to tf via robot_state_publisher'),

        # Waypoints
        DeclareLaunchArgument('waypoint_mode',               default_value='False',                                               description='Set to true for waypoint following mode'),
        DeclareLaunchArgument('waypoints_file',              default_value=f'{SHARE_DIR}/config/no_waypoints.yaml',               description='Path to waypoint yaml file (waypoints_x/waypoints_y lists)'),

        # Logging and visualization
        DeclareLaunchArgument('auto_launch_rviz',            default_value='True',                                                description='Automatically launch rviz display window'),
        DeclareLaunchArgument('rviz_config',                 default_value=f'{SHARE_DIR}/rviz/avt_341_ros2.rviz',                 description='Single vehicle rviz config file'),
        DeclareLaunchArgument('enable_logging',              default_value='False',                                               description='Enable standardized vehicle logging for V&V efforts'),
        DeclareLaunchArgument('logging_path',                default_value=os.path.join(os.path.expanduser('~'), 'avt_341_data'), description='Save path for vehicle logging'),

        *pargs.declare_arguments(),
        OpaqueFunction(function=_spawn_vehicles),
    ])
