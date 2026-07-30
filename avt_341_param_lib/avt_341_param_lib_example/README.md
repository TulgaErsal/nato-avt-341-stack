# avt_341_param_lib_example

ROS2 package demonstrating the use of the avt_341_param_lib package: generated
parameter libraries (partially taken from
[generate_parameter_library](https://github.com/pickNikRobotics/generate_parameter_library))
plus the hierarchical parameter override launch helpers.

Four dummy nodes are replicated per agent namespace: `planner` and
`controller` share the `nav.yaml` template, `sensor` uses `sensor.yaml`, and
`mixin_ex` uses `mixin_ex.yaml`.

## Mixins

`mixin_ex.yaml` demonstrates both ways of reusing a mixin from
`parameters/mixins/`:

| | `__inherit_mixins: inherited_mixin` | `__include_mixins: composed_mixin` |
|---|---|---|
| Allowed at | directly under `ros__parameters` only | any level |
| Generated C++ | `struct Params : public example_demo::mixins::InheritedParams` | `example_demo::mixins::Extents extents;` member |
| C++ access | `params.rate_hz` | `params.region.extents.width` |
| ROS parameter name | `rate_hz` | `region.extents.width` |

Both mixins live in one `code_namespace`, so each declares a distinct
`class_name`: a template referencing both pulls both generated headers into a
single translation unit, and two classes with the same qualified name would
collide.

## Runtime parameter files

`parameters/` holds the build-time code generation templates; `parameters_override/`
holds the runtime ROS 2 parameter files that layer over the generated defaults.
The launch files pass the latter through the `params_files` argument, and ROS
applies them in list order with later files winning per parameter.

A runtime file can also declare the files it overrides, with a top-level
`__overrides` key naming one path or a yaml list of paths:

| File | Declares | Effect |
|---|---|---|
| `global_params.yaml` | -- | the base layer |
| `agent_params.yaml` | `__overrides: global_params.yaml` | relative to the declaring file |
| `experiment_params.yaml` | `__overrides: $pkg_path{avt_341_param_lib_example}/...` | `$pkg_path{}` reaches into any package's share tree |

Every named file is handed to the nodes *before* the declaring file, so the
declaring file wins. A named file already in `params_files` is moved ahead of
its declarer rather than loaded twice; one that is absent is loaded as well, so
passing `experiment_params.yaml` alone brings in all three. Paths are absolute,
`$pkg_path{}` expansions, or relative to the declaring file. Cycles are an
error.

Command line parameter overrides use the same node-selector syntax as the
runtime parameter yaml files: namespaces and node names are `/`-delimited
(`**` matches any number of tokens, `*` exactly one) and parameter names use
`.` for nesting. Relative selectors are resolved against the agent list: a
first segment naming an agent scopes the override to that agent, anything else
applies across all agents. Per-node override priority (later wins, per
parameter):

1. runtime yaml files (`params_files` launch argument) in their effective
   order: the list as given, with each file's `__overrides` targets ahead of it
2. command line overrides, in command line order

## Examples

```
# Defaults: veh1 gets the global yaml values, veh2 the agent-specific ones
ros2 launch avt_341_param_lib_example client.launch.py

# One file, whole stack: experiment_params.yaml declares __overrides, so
# agent_params.yaml and global_params.yaml are loaded ahead of it
SHARE=$(ros2 pkg prefix --share avt_341_param_lib_example)
ros2 launch avt_341_param_lib_example client.launch.py params_files:="[$SHARE/parameters_override/experiment_params.yaml]"

# Order-proof: agent_params.yaml declares that it overrides global_params.yaml,
# so it still wins even when the list puts it first
ros2 launch avt_341_param_lib_example client.launch.py params_files:="[$SHARE/parameters_override/agent_params.yaml, $SHARE/parameters_override/global_params.yaml]"

# Global override: all nodes of all agents
ros2 launch avt_341_param_lib_example client.launch.py **/cruise_speed:=9.0

# One node type across all agents (equivalent to /*/planner/cruise_speed)
ros2 launch avt_341_param_lib_example client.launch.py planner/cruise_speed:=8.0

# One node of one agent; later command line entries win
ros2 launch avt_341_param_lib_example client.launch.py **/cruise_speed:=9.0 veh1/planner/cruise_speed:=3.3

# Mapping form: the selector's ros__parameters body as a yaml mapping
ros2 launch avt_341_param_lib_example client.launch.py veh1:="{cruise_speed: 3.3, planner_mode: graph}"

# List the available override arguments (each name is valid override syntax)
ros2 launch avt_341_param_lib_example client.launch.py -s
```
