Parameter Library
=================

.. note::

    **Acknowledgement:** The code-generation portion of this library was modified from the
    existing `generate_parameter_library <https://github.com/pickNikRobotics/generate_parameter_library>`_ ROS2 package.
    For more details, refer to the :ref:`Comparison to generate_parameter_library <comparison-to-generate-parameter-library>`.


The ``avt_341_param_lib`` library package is used to help manage large sets of ROS parameters.
Parameters are managed in a hierarchical format allowing selective parameter overrides at each stage.
It consists of two major processing steps:

#. **Code-Generation:** Invoked at build time. Converts input template yaml files into C++ or python
   code which is built into referencing the node executable.
   The generated code automatically manages declaring and loading ROS parameters corresponding to the template yaml.

#. **Runtime:** Invoked when nodes are ran through ROS launch. Intercepts and manipulates input runtime parameter files
   and command line arguments. Provides final merged parameter values to node.

The manipulation of parameters through these stages is summarized in :numref:`fig-param-lib-overview`.

.. figure:: images/conf_param_lib_overview.svg
    :name: fig-param-lib-overview
    :alt: Parameter library data flow
    :width: 100%
    :align: center

    Overview of parameter manipulation pipeline.

:numref:`fig-param-lib-priority` further illustrates the priority of the different parameter sources, with command line arguments having highest priority.
Note that parameter overriding is done on a per-parameter basis. Parameters which are not specified in higher priority sources will retain their lower priority values.

.. figure:: images/conf_param_lib_priority.svg
    :name: fig-param-lib-priority
    :alt: Parameter override priority
    :width: 100%
    :align: center

    Hierarchical parameter override priority.


.. note::

    Template file parameters apply to **node classes** while runtime parameters may apply further
    to named **node instances** (such as specified in a launch file) only known at run-time.

Code-Generation Configuration
-------------------------------------------

This sections describes the major configuration elements in the code-generation step. These are:

#. :ref:`Template parameter files <template-parameter-files>`
#. :ref:`CMake and in-code usage <cmake-and-in-code-usage>`

.. _template-parameter-files:

Template Parameter Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Template parameter files define the set of all available ROS parameters including their names, default values and type.
Refer to the ``avt_341_param_lib_example/parameters`` sample ROS2 package for an example.

:numref:`lst-param-template-basic` shows a basic template parameter file.

.. code-block:: yaml
    :name: lst-param-template-basic
    :caption: Basic template parameter file.

    code_namespace: nav_demo
    ros__parameters:
      cruise_speed:
        type: double
        default_value: 1.0
        description: "Cruising speed of the vehicle in m/s"
      planner_mode:
        type: string
        default_value: "grid"
        description: "Planning algorithm selection"
        validation:
          one_of<>: [ [ "grid", "graph" ] ]

.. _template-root-keys:

**Root YAML Keys**

The keys accepted at the root of a template parameter file are listed in :numref:`tbl-param-root-keys`.
Any other root key will result in a code-generation error.

.. list-table:: Template parameter file root yaml keys.
    :name: tbl-param-root-keys
    :header-rows: 1
    :widths: 22 53 25

    * - Key
      - Description
      - Default value
    * - ``class_name``
      - C++ or Python class name of the generated parameter structure. In a
        :ref:`mixin file <template-mixins>` it also names the class that
        ``__inherit_mixins`` derives from.
      - ``Params``
    * - ``code_namespace``
      - C++ namespace for the generated code. Slash-separated tokens become nested namespaces.
      - *Required*
    * - ``ros__parameters``
      - Non-empty mapping holding the parameter definitions.
      - *Required*

.. _template-mixins:

**Mixins**

A mixin is a reusable fragment of parameter definitions which can be reused by other template files.
Currently a mixin may not itself reference other mixins.
There are two ways to reference mixins, summarised in :numref:`tbl-param-mixin-keys`.

.. list-table:: Mixin reuse keys.
    :name: tbl-param-mixin-keys
    :header-rows: 1
    :widths: 18 20 26 18 18

    * - Key
      - Allowed at
      - Generated Code
    * - ``__include_mixins``
      - any level of ``ros__parameters``
      - members spliced inline
    * - ``__inherit_mixins``
      - directly under ``ros__parameters`` only
      - the generated class derives from the mixin's class

**Composing mixins**

Use ``__include_mixins: <mixin-list>`` at any level of the ``ros__parameters`` tree.
Each mixin's parameters are spliced into the mapping holding the key.

:numref:`lst-param-mixin-file` defines a mixin, which is then included by the template file in
:numref:`lst-param-template-mixin`.

.. code-block:: yaml
    :name: lst-param-mixin-file
    :caption: Mixin file ``mixins/geometry_mixin.yaml``.

    code_namespace: params/core
    ros__parameters:
      geometry:
        width:
          type: double
          default_value: 200.0
          description: "Total grid width in meters."

.. code-block:: yaml
    :name: lst-param-template-mixin
    :caption: Template parameter file including the mixin.

    code_namespace: params/costmap
    ros__parameters:
      costmap:

        __include_mixins: geometry_mixin

        thresh:
          type: double
          default_value: 0.5
          description: "Minimum cell slope that is considered occupied."

.. note::

    Mixin parameters are addressed by their position in the including tree, giving a final parameter
    path of ``<parent_tree>.<mixin param>``. In the example above the mixin defines ``geometry.width``,
    which is included under ``costmap`` and is therefore addressed as ``costmap.geometry.width``.

**Inheriting mixins**

Use ``__inherit_mixins: <mixin-list>`` directly under ``ros__parameters``.
The mixin's parameters splice in at the template root, and the generated class derives from the class
the mixin file itself generates. Several mixins may be inherited at once.

:numref:`lst-param-inherit-mixin-file` defines a mixin, inherited by the template file in
:numref:`lst-param-template-inherit`, producing the C++ in :numref:`lst-param-inherit-generated`.

.. code-block:: yaml
    :name: lst-param-inherit-mixin-file
    :caption: Mixin file ``mixins/timing_mixin.yaml``.

    class_name: TimingParams
    code_namespace: params/core
    ros__parameters:
      rate_hz:
        type: double
        default_value: 20.0
        description: "Node update rate in Hz."

.. code-block:: yaml
    :name: lst-param-template-inherit
    :caption: Template parameter file inheriting the mixin.

    code_namespace: params/costmap
    ros__parameters:

      __inherit_mixins: timing_mixin

      thresh:
        type: double
        default_value: 0.5
        description: "Minimum cell slope that is considered occupied."

.. code-block:: cpp
    :name: lst-param-inherit-generated
    :caption: Generated parameter structure.

    namespace params::costmap {
        struct Params : public params::core::TimingParams {
            double thresh = 0.5;
            ParamsStamp __stamp;
        };
    }

The inherited parameter is declared as ``rate_hz``, read as ``params.rate_hz``, and the whole structure
can be passed to code that only knows ``params::core::TimingParams``.

.. note::

    ``__inherit_mixins`` is only allowed directly under ``ros__parameters``. Inherited parameters become
    members of the generated class itself, so they carry no name prefix; mounting them under a group
    would make the C++ member path and the ROS parameter name disagree. Use ``__include_mixins`` to nest
    a mixin.

.. _cmake-and-in-code-usage:

CMake and In-Code Usage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Code generation is invoked from the package ``CMakeLists.txt`` as shown in :numref:`lst-param-cmake`.
``avt_341_generate_cpp_parameters()`` takes a folder (or glob) of template files and generates a data
transfer object (dto) library and a parameter service library per template, both named after the
template file stem. Node targets then link the service library.

.. code-block:: cmake
    :name: lst-param-cmake
    :caption: Invoking code generation from ``CMakeLists.txt``.

    find_package(avt_341_param_lib REQUIRED)

    # parameters/costmap.yaml -> costmap_params_dto and costmap_params_service
    avt_341_generate_cpp_parameters(parameters)

    add_executable(costmap_node src/costmap_node.cpp)
    target_link_libraries(costmap_node PUBLIC costmap_params_service)

The python equivalent is ``avt_341_generate_python_parameters()``.

The generated service header is included as ``<package_name>/<stem>_params_service.hpp`` and declares a
``<class_name>Listener`` class in the template's ``code_namespace``. Constructing the listener declares
and loads all parameters, and ``get_params()`` returns the populated parameter structure.
:numref:`lst-param-node` reads the parameters of the template file shown in
:numref:`lst-param-template-mixin`.

.. code-block:: cpp
    :name: lst-param-node
    :caption: Reading parameter values in a C++ ROS node.

    #include <rclcpp/rclcpp.hpp>

    #include <my_package/costmap_params_service.hpp>

    class CostmapNode : public rclcpp::Node {
     public:
      CostmapNode() : Node("costmap") {
        param_listener_ = std::make_shared<params::costmap::ParamsListener>(
            get_node_parameters_interface(), get_logger());
        params_ = param_listener_->get_params();

        RCLCPP_INFO(get_logger(), "grid width: %f", params_.costmap.geometry.width);
      }

     private:
      std::shared_ptr<params::costmap::ParamsListener> param_listener_;
      params::costmap::Params params_;
    };

Runtime Configuration
-------------------------------------------

Runtime parameter configuration can be used to override the template defaults provided during code generation.
The major configuration elements in the runtime step are:

#. :ref:`Runtime parameter files <runtime-parameter-files>`
#. :ref:`Node configuration file <node-configuration-file>`
#. :ref:`Command line arguments <command-line-arguments>`
#. :ref:`Regular expression node selectors <regex-node-selectors>`
#. :ref:`Parameter expressions <parameter-expressions>`

.. _runtime-parameter-files:

Runtime Parameter Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Runtime parameter files are an existing concept in ROS2 parameter but this library includes several augmentations.
Nonetheless, the basic parameter file format and node selector syntax is briefly reviewed in this section.

Each top-level node in a runtime parameter file consists of a node selector with possible wildcards ``*`` or ``**``
followed by the ``ros__parameters`` key-value map.
Unlike, template parameter files, which are linked to node targets in the ``CMakeLists.txt``, runtime parameters apply
to named node instances to support possibly having several instances of the same node class.

:numref:`lst-param-runtime-file` overrides two of the parameters declared by the template file in :numref:`lst-param-template-mixin`.

.. code-block:: yaml
    :name: lst-param-runtime-file
    :caption: Runtime parameter file overriding template defaults.

    /**:
      ros__parameters:
        costmap:
          geometry:
            width: 400.0

    /**/costmap_node:
      ros__parameters:
        costmap:
          thresh: 0.75

**Selector Syntax**

Node selectors use the standard ROS2 parameter file convention: a slash-delimited path of namespace
tokens ending in a node name, where ``**`` matches any number of tokens, ``*`` matches exactly one
token, and any other token matches literally. As a library extension, selector tokens may also be
:ref:`regular expressions <regex-node-selectors>`, described in their own section. The possible
forms are given in :numref:`tbl-param-selectors`.

.. list-table:: Node selector forms and the nodes they match.
    :name: tbl-param-selectors
    :header-rows: 1
    :widths: 26 37 37

    * - Selector
      - Matches
      - Does not match
    * - ``/**``
      - Every node, in any namespace and at any depth.
      - --
    * - ``/**/costmap_node``
      - ``/costmap_node``, ``/veh1/costmap_node``, ``/veh1/nav/costmap_node``
      - ``/veh1/planner_node``
    * - ``/*/costmap_node``
      - ``/veh1/costmap_node``, ``/veh2/costmap_node``
      - ``/costmap_node``, ``/veh1/nav/costmap_node``
    * - ``/veh1/**``
      - Every node under ``/veh1``, at any depth.
      - ``/veh2/costmap_node``
    * - ``/veh1/costmap_node``
      - Only ``/veh1/costmap_node``.
      - ``/veh2/costmap_node``, ``/veh1/nav/costmap_node``

**Selector Precedence**

ROS2 applies selector precedence based on two axes of selector specificity and parameter file ordering.
File ordering only takes precedent for the exact same node selector (before wildcard matching). Beyond this, more
specific node selectors take precedence. Examples of the two axes are summarized in :numref:`tbl-param-precedence`.

.. list-table:: Resolution of sections competing for one parameter of one node.
    :name: tbl-param-precedence
    :header-rows: 1
    :widths: 28 28 44

    * - Axis
      - Example
      - Decided by
    * - Different selectors matching the node
      - ``/**`` against ``/veh1/costmap_node``
      - Specificity: the broadest section is applied first, so the narrowest one wins. File order is
        irrelevant.
    * - One selector repeated
      - ``/**`` against ``/**``
      - Document order and then file order, with the later entry winning.

.. _runtime-file-overrides:

**Override Declaration**

In order to avoid ambiguity in parameter file ordering, a file may declare override precedence to another file
using the top-level ``__overrides`` key of :numref:`lst-param-runtime-overrides`. Files specified under the ``__overrides``
key will also be automatically loaded and do not need to be passed to the launch file.
This is a library extension with no ROS2 counterpart.

.. code-block:: yaml
    :name: lst-param-runtime-overrides
    :caption: Runtime parameter file declaring the file it overrides.

    __overrides: global_params.yaml

    /**/costmap_node:
      ros__parameters:
        costmap:
          thresh: 0.9

.. _node-configuration-file:

Node Configuration File
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The node configuration file includes additional settings per node.
Currently topic remappings (``remappings`` key) and environment variables (``additional_env`` key) are supported.
This is a novel configuration file type, not native to the ROS2 ecosystem.
It uses the same selector syntax as runtime parameter files (:numref:`tbl-param-selectors`),
including :ref:`regular expression selector tokens <regex-node-selectors>`.
:numref:`lst-param-node-config` shows an example file.

.. code-block:: yaml
    :name: lst-param-node-config
    :caption: Node configuration file.

    /**/perception_local_node:
      remappings:
        avt_341/terrain_slope: avt_341/terrain_slope_local

    /**/uab_perception_node:
      remappings:
        avt_341/points: /ouster/points
      additional_env:
        LOG_LEVEL: debug

.. _command-line-arguments:

Command Line Arguments
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Command line arguments have the highest priority and override the values of any runtime parameter file,
again per parameter. Where several command line entries apply to the same parameter, the last one wins.

ROS2 does not natively support the node selector syntax as is used in parameter files (:ref:`described earlier <runtime-parameter-files>`).
This library adds that capability: launch arguments which look like a selector
are intercepted before the nodes are launched and expanded using the same rules as parameter files.

.. _cli-selector-syntax:

**Selector Syntax and Automatic Expansions**

Command line selectors may be abbreviated. The library completes them against the list of vehicle ids
being launched, as summarized in :numref:`tbl-param-cli-expansions`. A first segment naming a vehicle
scopes the override to that vehicle; any other first segment applies across all vehicles. The
command-line specifics of :ref:`regular expression selector tokens <regex-node-selectors>` are
described in their own section.

.. list-table:: Command line selector expansions, for vehicle ids ``veh1`` and ``veh2``.
    :name: tbl-param-cli-expansions
    :header-rows: 1
    :widths: 36 28 36

    * - Command line argument
      - Expanded selector
      - Applies to
    * - ``width:=400.0``
      - ``/**``
      - Every node declaring ``width``. Only recognized for a parameter name declared by a template.
    * - ``costmap.geometry:="{width: 400.0, height: 400.0}"``
      - ``/**``
      - Every node declaring ``costmap.geometry.width/height``. A mapping attached to a declared parameter-group prefix is flattened below that prefix.
    * - ``**/width:=400.0``
      - ``/**``
      - Every node declaring ``width``.
    * - ``costmap_node/width:=400.0``
      - ``/*/costmap_node``
      - ``costmap_node`` of every vehicle.
    * - ``nav/costmap_node/width:=400.0``
      - ``/*/nav/costmap_node``
      - Nested ``nav/costmap_node`` of every vehicle.
    * - ``veh1/costmap_node/width:=400.0``
      - ``/veh1/costmap_node``
      - ``costmap_node`` of ``veh1`` only.
    * - ``veh1:="{width: 400.0}"``
      - ``/veh1/**``
      - Every node of ``veh1``.
    * - ``/veh1/costmap_node/width:=400.0``
      - ``/veh1/costmap_node``
      - No expansion; vehicle, namespace, node and parameter are all given.
    * - ``'(veh[12])/costmap_node/width:=400.0'``
      - ``/(veh[12])/costmap_node``
      - ``costmap_node`` of ``veh1`` and ``veh2``; the regex matches vehicle ids, so it anchors at
        the vehicle position.
    * - ``'(veh[12]):="{width: 400.0}"'``
      - ``/(veh[12])/**``
      - Every node of ``veh1`` and ``veh2``.

.. _parameter-sub-maps:

**Parameter Sub-Maps**

This library also augments command line arguments with the ability to specify sub-maps to reduce repetition of stem elements. For example:

.. code-block:: bash

    ros2 launch avt_341_bringup krc.launch.py \
        veh1/costmap_node:="{costmap: {geometry: {width: 400.0}, thresh: 0.75}}"

is equivalent to the two scalar entries below.

.. code-block:: bash

    ros2 launch avt_341_bringup krc.launch.py \
        veh1/costmap_node/costmap.geometry.width:=400.0 \
        veh1/costmap_node/costmap.thresh:=0.75

A bare parameter-group prefix applies the sub-map to every matching node, just
like the bare scalar parameter shorthand applies a complete parameter name to
``/**``. For example:

.. code-block:: bash

    ros2 launch avt_341_bringup krc.launch.py \
        costmap.geometry:="{width: 400.0, height: 300.0}"

is equivalent to:

.. code-block:: bash

    ros2 launch avt_341_bringup krc.launch.py \
        costmap.geometry.width:=400.0 \
        costmap.geometry.height:=300.0

If a bare mapping key is also a vehicle id or node name, its selector meaning
takes precedence. Use an explicit ``**`` selector mapping when that distinction
would otherwise be ambiguous.


.. _regex-node-selectors:

Regular Expression Node Selectors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The glob wildcards can only express "exactly one" (``*``) or "any number of" (``**``) tokens. As a
library extension, a selector token wrapped in parentheses is a Python regular expression, covering
the selections in between -- for example two vehicles of a three vehicle formation. The parentheses
are part of the pattern (they form an ordinary regex group), and the expression must fully match
exactly one slash-delimited token, so a regex can never contain ``/``. Regex tokens compose freely
with literal tokens and the glob wildcards. :numref:`tbl-param-regex-selectors` shows examples.

.. list-table:: Regular expression node selector examples.
    :name: tbl-param-regex-selectors
    :header-rows: 1
    :widths: 26 37 37

    * - Selector
      - Matches
      - Does not match
    * - ``/(veh[12])/costmap_node``
      - ``/veh1/costmap_node``, ``/veh2/costmap_node``
      - ``/veh3/costmap_node``
    * - ``/**/(planner_[0-9]+)``
      - ``/veh1/planner_1``, ``/veh1/nav/planner_2``
      - ``/veh1/planner_x``

.. _parameter-expressions:

Parameter Expressions
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Parameter values may hold expressions which are resolved before the values are handed to the nodes.
The available expressions are given in :numref:`tbl-param-expressions`.

.. list-table:: Parameter expressions.
    :name: tbl-param-expressions
    :header-rows: 1
    :widths: 26 44 30

    * - Expression
      - Description
      - Example
    * - ``$ref{<selector>/<param>}``
      - Resolved value of another parameter in the same set of files. Must be the entire value. Where
        wildcards match several entries, the first match wins.
      - ``$ref{/**/max_speed}``
    * - ``$python{<expression>}``
      - Result of evaluating the python expression, with ``os``, ``math`` and
        ``get_package_share_directory`` available. Must be the entire value.
      - ``$python{2.0 * 3.0}``
    * - ``$pkg_path{<package>}``
      - Share directory of the package. May appear anywhere, and several times, within a string value.
      - ``$pkg_path{avt_341_bringup}/env_data``

``$ref{}`` and ``$python{}`` are resolved in runtime parameter files. ``$pkg_path{}`` is additionally
expanded in string valued command line overrides and in
:ref:`file level override target paths <runtime-file-overrides>`, where the other two are not
accepted. Template parameter files are never scanned for expressions.

The first match ``$ref{}`` resolves to follows the effective file order, so a
:ref:`file level override declaration <runtime-file-overrides>` which moves a file ahead of another
also moves it ahead for reference lookups.

.. _comparison-to-generate-parameter-library:

Comparison to generate_parameter_library
-------------------------------------------

The existing `generate_parameter_library <https://github.com/pickNikRobotics/generate_parameter_library>`_ only implements the baseline
code-generation step. **It does not contain any of the runtime parameter management features.**
This library also implements additional features in the code-generation step. The major augmentations are summarized below, by section.

**Code-Generation:**

* Additional configuration provided by :ref:`root yaml keys <template-root-keys>`.
* Support for :ref:`mixins <template-mixins>` composition or inheritance.
* Added float32 parameter type support.
* Separated data transfer object (dto) and parameter listener service classes into separate files to reduce dependencies in referencing code.

**Run-Time:**

* Hierarchical override system merging :ref:`1) code-gen template parameter files <template-parameter-files>`,
  :ref:`2) runtime parameter files <runtime-parameter-files>`, and :ref:`3) command line arguments <command-line-arguments>`. Listed in increasing priority.
* :ref:`Parameter file override declarations <runtime-file-overrides>`.
* Added concept of :ref:`node configuration file <node-configuration-file>` for topic remappings and environment variables.
* Command line support for :ref:`node selector syntax <cli-selector-syntax>`.
* Command line support for :ref:`sub-map parameters <parameter-sub-maps>`.
* :ref:`Regular expression support <regex-node-selectors>` for the node selector syntax.
* :ref:`Parameter expressions <parameter-expressions>`.
