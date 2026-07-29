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

Command line parameter overrides use the same node-selector syntax as the
runtime parameter yaml files: namespaces and node names are `/`-delimited
(`**` matches any number of tokens, `*` exactly one) and parameter names use
`.` for nesting. Relative selectors are resolved against the agent list: a
first segment naming an agent scopes the override to that agent, anything else
applies across all agents. Per-node override priority (later wins, per
parameter):

1. runtime yaml files (`params_files` launch argument, in list order)
2. command line overrides, in command line order

## Examples

```
# Defaults: veh1 gets the global yaml values, veh2 the agent-specific ones
ros2 launch avt_341_param_lib_example client.launch.py

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
