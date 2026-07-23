# AVT-341 Parameter Library

## Generated C++ DTO and service headers

Each C++ parameter template produces two headers and two CMake interface
targets. For a template named `nav.yaml`, the generated interfaces are:

* `nav_params_dto.hpp` / `nav_params_dto`
* `nav_params_service.hpp` / `nav_params_service`

The DTO header contains `Params`, its nested structures, `StackParams`,
dynamic parameter maps, defaults, and `ParamsStamp`. It uses only the C++17
standard library, so code that only stores or transforms parameter values can
remain independent of ROS:

```cpp
#include <my_package/nav_params_dto.hpp>
```

```cmake
target_link_libraries(my_dto_consumer nav_params_dto)
```

`ParamsStamp` is a small value type containing a signed 32-bit `sec` field
and an unsigned 32-bit `nanosec` field. It is zero-initialized and supports
equality and inequality comparisons.

The service header includes its sibling DTO header and adds `ParamsListener`,
ROS parameter declarations and updates, validation, callbacks, logging, and
synchronization. ROS nodes should normally include and link this interface:

```cpp
#include <my_package/nav_params_service.hpp>
```

```cmake
target_link_libraries(my_node nav_params_service)
```

`avt_341_generate_cpp_parameters()` uses the YAML stem as the base name.
`NAME_SUFFIX` is appended to that stem before the fixed `_params_dto` and
`_params_service` suffixes. The explicit
`avt_341_generate_cpp_parameter_file()` macro follows the same naming rules.
The former `_parameters.hpp` header and `_parameters` target are not
generated.

## Launch node metadata

`avt_341_param_lib.launch_metadata.MetadataCollection` loads one optional
launch-only YAML file and returns the topic remappings and additional process
environment that apply to a node:

```python
from avt_341_param_lib.launch_metadata import MetadataCollection

metadata = MetadataCollection('/path/to/metadata.yaml')
tracker_remappings = metadata.get_remappings('/veh1/object_tracking_node')
tracker_environment = metadata.get_additional_env('/veh1/object_tracking_node')
```

This file is separate from ROS parameter files. It is consumed by a Python
launch file and must not be passed to a node as a `--params-file`.

Each node selector may contain a source-to-target `remappings` mapping, an
`additional_env` mapping, or both:

```yaml
/**/object_tracking_node:
  remappings:
    camera_info: /flir_camera/camera_info
    image: /flir_camera/image_rect_color
    task: avt_341/mission_task_state

/**/uab_perception_node:
  remappings:
    points: /ouster/points
  additional_env:
    LOG_LEVEL: debug
    LD_LIBRARY_PATH:
      separator: ":"
      values:
        - "$env_var{MCR_ROOT:-/usr/local/MATLAB/Runtime}/runtime/glnxa64"
        - "$env_var{LD_LIBRARY_PATH:-}"
```

Selectors use the ROS 2 parameter-file path convention. `*` matches exactly
one path token and `**` matches zero or more path tokens. Wildcards must be
complete tokens. A selector such as `object_tracking_node` addresses only the
root node `/object_tracking_node`; use `/**/object_tracking_node` to match that
node name in any namespace.

Nested namespace mappings are also accepted:

```yaml
/**:
  object_tracking_node:
    remappings:
      points/input: /ouster/points
```

Simple environment assignments are scalar strings. List-style assignments
contain an explicit `separator` and a non-empty `values` list; the separator
is inserted between the resolved values to create the process environment
string.

Environment strings support embedded launch-context lookups:

* `$env_var{NAME}` requires `NAME` to exist.
* `$env_var{NAME:-default}` uses `default` when `NAME` is absent. The default
  may be empty.

The metadata loader compiles these expressions to ROS
`EnvironmentVariable` substitutions. Plain `$NAME` and `${NAME}` strings are
not expanded by `Node.additional_env` and remain literal.

All matching sections are merged in YAML document order. When the same
remapping source or environment variable occurs more than once, its last
matching definition wins. Selectors that do not match a node remain dormant,
which permits one file to cover optional nodes and several vehicle
configurations.

`MetadataCollection(None)` and `MetadataCollection('')` create an empty
collection. The API intentionally accepts only one file; file stacking,
parameter-file preprocessing expressions, and per-rule launch arguments are
not supported.

## Logical float parameters

Parameter templates may use `float` and `float_array` when generated C++
parameter fields should use single-precision storage:

```yaml
ros__parameters:
  gain:
    type: float
    default_value: 1.25
  samples:
    type: float_array
    default_value: [0.25, 0.5]
```

ROS 2 has no distinct single-precision parameter type. These values are
declared, loaded, validated, updated, and exposed through ROS as `double` and
`double_array`. The generated C++ listener immediately narrows them to `float`
and `std::vector<float>`. ROS parameter inspection therefore continues to
report double types, and normal C++ narrowing behavior applies to values
outside the representable float range.

The Python generator also accepts the logical float types, but both map to
Python's ordinary double-precision `float` and lists of `float`.
