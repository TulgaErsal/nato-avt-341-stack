# avt_341_param_lib_example

ROS2 package demonstrating the use of the avt_341_param_lib package: generated
parameter libraries (partially taken from
[generate_parameter_library](https://github.com/pickNikRobotics/generate_parameter_library))
plus the hierarchical parameter override launch helpers.

Three dummy nodes are replicated per agent namespace: `planner` and
`controller` share the `nav_params.yaml` template, `sensor` uses `sensor.yaml`.
Per-node override priority (later wins, per parameter):

1. runtime yaml files (`params_files` launch argument, in list order;
   agent-specific sections use namespace wildcards such as `/veh2/**:`)
2. explicitly provided global launch arguments (`nav/...`, `sensor/...`)
3. agent-specific command line overrides (`<vehicle_id>_overrides`)

## Examples

```
# Defaults: veh1 gets the global yaml values, veh2 the agent-specific ones
ros2 launch avt_341_param_lib_example client.launch.py

# Global command line override: applies to all agents, beats agent yaml
ros2 launch avt_341_param_lib_example client.launch.py nav/cruise_speed:=9.0

# Agent-specific command line override: beats everything, for one agent only
ros2 launch avt_341_param_lib_example client.launch.py nav/cruise_speed:=9.0 veh1_overrides:="{cruise_speed: 3.3}"

# List the available override arguments
ros2 launch avt_341_param_lib_example client.launch.py -s
```
