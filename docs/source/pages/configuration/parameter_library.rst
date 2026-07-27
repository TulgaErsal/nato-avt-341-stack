Parameter Library
=================

The ``avt_341_param_lib`` package generates code files matching input yaml
template files, and parses input yaml files and command lines at runtime. A
node's parameters are declared once in a yaml template; from that template the
library generates the C++ or Python interfaces used by the node, and provides
the launch-time helpers that resolve parameter files, launch arguments and
command line overrides.

Package layout
--------------

The Python package is split by *when* its modules run:

.. list-table::
   :header-rows: 1
   :widths: 20 15 65

   * - Sub-package
     - Runs at
     - Contents
   * - ``avt_341_param_lib.codegen``
     - build time
     - The generators invoked by the CMake macros
       (``python -m avt_341_param_lib.codegen.generate_*``), the yaml-to-code
       engine ``parse_yaml``, the C++/Python conversion tables, the jinja
       templates, and ``python_validators`` (which the generated Python modules
       import).
   * - ``avt_341_param_lib.runtime``
     - launch time
     - The launch-file helpers: ``launch_params``, ``launch_node_config``,
       ``parse_runtime_yaml``, ``node_selectors``, ``launch_expressions``.
   * - ``avt_341_param_lib.common``
     - both
     - ``template_yaml``, which owns the parameter template format itself --
       the root elements (``ros__parameters``, ``code_namespace``,
       ``class_name``), the ``__include_mixins`` expansion and
       ``YAMLSyntaxError``.

``common`` deliberately depends on neither of the other two, and neither of the
other two depends on the other. In particular the launch-time helpers do not
import the code generator, so ``ros2 launch`` does not pull in jinja2 or the
conversion tables.

Generated C++ DTO and service headers
-------------------------------------

Each C++ parameter template produces two headers and two CMake interface
targets. For a template named ``nav.yaml``, the generated interfaces are:

* ``nav_params_dto.hpp`` / ``nav_params_dto``
* ``nav_params_service.hpp`` / ``nav_params_service``

The DTO header contains ``Params``, its nested structures, ``StackParams``,
dynamic parameter maps, defaults, and ``ParamsStamp``. It uses only the C++17
standard library, so code that only stores or transforms parameter values can
remain independent of ROS:

.. code-block:: cpp

    #include <my_package/nav_params_dto.hpp>

.. code-block:: cmake

    target_link_libraries(my_dto_consumer nav_params_dto)

``ParamsStamp`` is a small value type containing a signed 32-bit ``sec`` field
and an unsigned 32-bit ``nanosec`` field. It is zero-initialized and supports
equality and inequality comparisons.

The service header includes its sibling DTO header and adds ``ParamsListener``,
ROS parameter declarations and updates, validation, callbacks, logging, and
synchronization. ROS nodes should normally include and link this interface:

.. code-block:: cpp

    #include <my_package/nav_params_service.hpp>

.. code-block:: cmake

    target_link_libraries(my_node nav_params_service)

``avt_341_generate_cpp_parameters()`` uses the yaml stem as the base name.
``NAME_SUFFIX`` is appended to that stem before the fixed ``_params_dto`` and
``_params_service`` suffixes. The explicit
``avt_341_generate_cpp_parameter_file()`` macro follows the same naming rules.
The former ``_parameters.hpp`` header and ``_parameters`` target are not
generated.

Template mixins
---------------

A template's ``ros__parameters`` tree may splice in shared parameter
definitions from one or more mixin files with an ``__include_mixins`` entry:

.. code-block:: yaml

    ros__parameters:
      __include_mixins: costmap_geometry_mixin, costmap_publish_mixin
      # a single stem or a yaml list ([a_mixin, b_mixin]) are also accepted

The value names one or more mixin files by bare stem, resolved to
``mixins/<stem>.yaml`` next to the including template. A mixin file uses the
same format as a template; its ``ros__parameters`` content is merged at the
location of the ``__include_mixins`` entry before any further processing, so
code generation, launch arguments, and documentation all see the expanded
template. A key provided both by the including mapping and a mixin (or by two
mixins) is an error, and mixins cannot include other mixins. Keep mixin files
in the ``mixins/`` subfolder: the generation glob does not match subfolders, so
they are never processed as node templates themselves.

For C++ generation each mixin additionally produces one shared type-only DTO
header (``<stem>_params_dto.hpp``) defining a struct per root-level parameter
group in the mixin's ``code_namespace`` (which is therefore required). The
including template's generated DTO references those shared structs (e.g.
``avt_341::params::core::Geometry geometry;``) instead of re-defining them
inline, so several nodes can pass the same parameter class to common code.
Root-level leaf parameters of a mixin splice as plain fields into the including
struct. Struct (group) names must stay unique within a shared ``code_namespace``
across mixins, and dynamically mapped (``__map_``) parameters are not supported
in mixins.

Launch node configuration
-------------------------

``avt_341_param_lib.runtime.launch_node_config.NodeConfigCollection`` loads one
optional launch-only yaml file and returns the topic remappings and additional
process environment that apply to a node:

.. code-block:: python

    from avt_341_param_lib.runtime.launch_node_config import NodeConfigCollection

    node_config = NodeConfigCollection('/path/to/node_config.yaml')
    tracker_remappings = node_config.get_remappings('/veh1/object_tracking_node')
    tracker_environment = node_config.get_additional_env('/veh1/object_tracking_node')

.. note::

    This file is separate from ROS parameter files. It is consumed by a Python
    launch file and must not be passed to a node as a ``--params-file``.

Each node selector may contain a source-to-target ``remappings`` mapping, an
``additional_env`` mapping, or both:

.. code-block:: yaml

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

Selectors use the ROS 2 parameter-file path convention. ``*`` matches exactly
one path token and ``**`` matches zero or more path tokens. Wildcards must be
complete tokens. A selector such as ``object_tracking_node`` addresses only the
root node ``/object_tracking_node``; use ``/**/object_tracking_node`` to match
that node name in any namespace.

Nested namespace mappings are also accepted:

.. code-block:: yaml

    /**:
      object_tracking_node:
        remappings:
          points/input: /ouster/points

Simple environment assignments are scalar strings. List-style assignments
contain an explicit ``separator`` and a non-empty ``values`` list; the separator
is inserted between the resolved values to create the process environment
string.

Environment strings support embedded launch-context lookups:

* ``$env_var{NAME}`` requires ``NAME`` to exist.
* ``$env_var{NAME:-default}`` uses ``default`` when ``NAME`` is absent. The
  default may be empty.

The node configuration loader compiles these expressions to ROS
``EnvironmentVariable`` substitutions. Plain ``$NAME`` and ``${NAME}`` strings
are not expanded by ``Node.additional_env`` and remain literal.

All matching sections are merged in yaml document order. When the same
remapping source or environment variable occurs more than once, its last
matching definition wins. Selectors that do not match a node remain dormant,
which permits one file to cover optional nodes and several vehicle
configurations.

``NodeConfigCollection(None)`` and ``NodeConfigCollection('')`` create an empty
collection. The API intentionally accepts only one file; file stacking,
parameter-file preprocessing expressions, and per-rule launch arguments are not
supported.
