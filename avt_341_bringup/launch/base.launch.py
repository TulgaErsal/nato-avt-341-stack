import os

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
from avt_341_param_lib.launch_node_config import NodeConfigCollection
from avt_341_param_lib.parse_runtime_yaml import resolve_params_files

# Parameter templates ship with the avt_341_nav source package; the deployment
# assets (urdf, rviz, bagging configuration) ship with avt_341_bringup.
AVT_341_DIR = get_package_share_directory('avt_341_nav').replace('\\', '/')
BRINGUP_DIR = get_package_share_directory('avt_341_bringup').replace('\\', '/')
TEMPLATES_DIR = os.path.join(AVT_341_DIR, 'parameters')


def _templates(*names):
    """Paths of the given parameter templates (bare stems; .yaml is appended)."""
    return [os.path.join(TEMPLATES_DIR, f'{name}.yaml') for name in names]


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
        # one template path or a list of paths (a node whose executable links
        # several generated parameter services); normalized to a list
        self.templates = [template] if isinstance(template, str) else list(template or [])
        self.condition = condition          # callable(context) -> bool; None = always
        self.sub_ns = sub_ns                # sub-namespace below the vehicle namespace
        self.extra_params = extra_params    # callable(vid, vehicles) -> dict of launch-computed params
        self.output = output
        self.autonomy = autonomy            # autonomy nodes are skipped for manual_control_vehicles


NODES = {

    # Mission management
    'mission_manager_node':             NodeSpec('mission_manager_node',             _templates('mission_manager'),                                                    extra_params=lambda vid, vehicles: {'name': str(vid).upper(), 'vehicle_namespaces': list(vehicles)},               autonomy=False),
    'comm_node':                        NodeSpec('comm_node',                        _templates('socket_comms'),                                                       extra_params=lambda vid, vehicles: {'name': str(vid).upper(), 'vehicle_namespaces': list(vehicles)},               autonomy=False),
    'speed_zones_node':                 NodeSpec('speed_zones_node',                 _templates('speed_zones'),       condition=is_cfg('use_speed_zones')),

    # Perception - Costmaps
    'perception_local_node':            NodeSpec('perception_node',                  _templates('perception'),        condition=is_cfg('use_dual_costmaps')),
    'perception_global_node':           NodeSpec('perception_node',                  _templates('perception'),        condition=is_cfg('use_dual_costmaps')),
    'perception_node':                  NodeSpec('perception_node',                  _templates('perception'),        condition=is_not_cfg('use_dual_costmaps')),
    'perception_rms_node':              NodeSpec('perception_node',                  _templates('perception'),        condition=is_cfg('use_perception_rms')),

    # Perception - Terrain segmentation
    'uab_perception_node':              NodeSpec('uab_perception_node',              _templates('uab_perception'),    condition=is_cfg('use_uab_perception'),          extra_params=lambda vid, vehicles: {'frame_prefix': f'{vid}/'}),

    # Perception - Object detection and tracking
    'object_detector_node':             NodeSpec('object_detector_node',             _templates('object_detector'),   condition=is_cfg('use_obj_detector')),
    'object_tracking_node':             NodeSpec('object_tracking_node',             _templates('object_tracking'),   condition=is_cfg('use_object_tracker'),          extra_params=lambda vid, vehicles: {'target_selection.formation_vehicle_ids': list(vehicles)}),
    'lidar_obstacle_detector_node':     NodeSpec('lidar_obstacle_detector_node',     _templates('obstacle_detector'), condition=is_cfg('use_lidar_obstacle_detector')),

    # Global planners
    'global_planner_node':              NodeSpec('global_planner_node',              _templates('global_planner')),

    # Local planners
    'rcc_planner_node':                 NodeSpec('rcc_planner_node',                 _templates('rcc_local_planner'), condition=is_local_planner('rcc')),
    'local_dwa_planner_node':           NodeSpec('dwa_planner_node',                 _templates('dwa_local_planner'), condition=is_local_planner('dwa')),
    'local_pf_planner_node':            NodeSpec('pf_planner_node',                  _templates('pf_local_planner'),  condition=is_local_planner('pf')),
    'mpc_planner_node':                 NodeSpec('mpc_planner_node',                 _templates('mpc_local_planner'), condition=is_local_planner('mpc')),

    # local planners - MPC supporting nodes
    'obstacle_processor_node':          NodeSpec('obstacle_processor_node',          _templates('mpc_local_planner'), condition=is_local_planner('mpc')),
    'segmentation_grid_processor_node': NodeSpec('segmentation_grid_processor_node', _templates('mpc_local_planner'), condition=is_local_planner('mpc')),
    'goal_point_processor_node':        NodeSpec('goal_point_processor_node',        _templates('mpc_local_planner'), condition=is_local_planner('mpc')),
    'veh_converter_node':               NodeSpec('veh_converter_node',                                                condition=is_local_planner('mpc')),

    # Controllers (selected by local planner method)
    'speed_control_node':               NodeSpec('speed_control_node',               _templates('speed_control'),     condition=is_local_planner('dwa', 'mpc')),
    'control_node':                     NodeSpec('control_node',                     _templates('control'),           condition=is_local_planner('rcc', 'pf')),

    # State pre-processing
    'data_acquisition_node':            NodeSpec('data_acquisition_node',            _templates('data_acquisition'),  condition=is_cfg('use_data_acquisition'),                                                                                                             output='log', autonomy=False),
}

pargs = ParameterCollection.from_node_templates(
    {name: spec.templates for name, spec in NODES.items() if spec.templates})

def _make_node(name, spec, vid, vehicles, params_files, cli_overrides, node_config):
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
    cli_params = cli_overrides.for_node(fqn)
    if cli_params:
        layers.append(cli_params)
    node_remappings = node_config.get_remappings(fqn)
    node_additional_env = node_config.get_additional_env(fqn)
    node = Node(
        package='avt_341_nav',
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


def _robot_state_publisher(vid, vehicle_index, context, node_config):
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
        remappings=node_config.get_remappings(f'/{vid}/robot_state_publisher') or None,
        additional_env=node_config.get_additional_env(f'/{vid}/robot_state_publisher') or None,
    )


def _spawn_vehicles(context, *args, **kwargs):
    vehicles = [str(vid).strip('/') for vid in perform_yaml(context, 'vehicle_ids')]
    params_files = resolve_params_files(
        perform_yaml(context, 'ros_param_files') or [], vehicles)
    node_config = NodeConfigCollection(
        LaunchConfiguration('node_config_file').perform(context))
    cli_overrides = pargs.resolve_cli_overrides(context, vehicles)
    spawn_filter = {
        str(vid).strip('/')
        for vid in perform_yaml(context, 'spawn_filter_vehicle_ids') or []}
    manual_vehicles = {
        str(vid).strip('/')
        for vid in perform_yaml(context, 'manual_control_vehicles') or []}
    publish_urdf_to_tf = _is_true(
        LaunchConfiguration('publish_urdf_to_tf').perform(context))

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
                name, spec, vid, vehicles, params_files, cli_overrides, node_config))
        if publish_urdf_to_tf:
            state_publisher = _robot_state_publisher(
                vid, vehicle_index, context, node_config)
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
            remappings=node_config.get_remappings('/rviz2') or None,
            additional_env=node_config.get_additional_env('/rviz2') or None,
        ))

    # Standardized vehicle logging for V&V efforts
    if _is_true(LaunchConfiguration('enable_logging').perform(context)):
        actions.append(ExecuteProcess(
            cmd=[
                'ros2', 'run', 'avt_341_bringup', 'vehicle_logging.py',
                f'{BRINGUP_DIR}/bagging/config/rw_bag_config.yaml',
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
        DeclareLaunchArgument('robot_description_files',     default_value=f"['{BRINGUP_DIR}/urdf/MRZR.urdf']",                   description='List of URDF files, one per vehicle (first entry reused when fewer files than vehicles are given)'),

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

        # Logging and visualization
        DeclareLaunchArgument('auto_launch_rviz',            default_value='True',                                                description='Automatically launch rviz display window'),
        DeclareLaunchArgument('rviz_config',                 default_value=f'{BRINGUP_DIR}/rviz/avt_341.rviz',                    description='Single vehicle rviz config file'),
        DeclareLaunchArgument('enable_logging',              default_value='False',                                               description='Enable standardized vehicle logging for V&V efforts'),
        DeclareLaunchArgument('logging_path',                default_value=os.path.join(os.path.expanduser('~'), 'avt_341_data'), description='Save path for vehicle logging'),

        *pargs.declare_arguments(),
        OpaqueFunction(function=_spawn_vehicles),
    ])
