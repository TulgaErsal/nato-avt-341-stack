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

TODO listing shows a basic template parameter file.100

.. listing::

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

**Root YAML Keys**

Before the main ``ros__parameters`` yaml key, there are a number of possible additional configuration keys. `All keys are optional.`

**code_namespace**: Defines the C++ namespace for the generated code. If not specified, the package name is used.
**class_name**: Defines the C++ class name for the generated code. If not specified, the package name is used.

**Mixins**

TODO

.. _cmake-and-in-code-usage:

CMake and In-Code Usage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^



Runtime Configuration
-------------------------------------------

This sections describes the major configuration elements in the code-generation step. These are:

#. TODO
#. TODO
#. TODO


Runtime Parameter Files
^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO: basic file definition

Node Configuration File
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Command Line Arguments
^^^^^^^^^^^^^^^^^^^^^^^^^^^

**Selector Syntax and Automatic Expansions**

TODO

Parameter Expressions
^^^^^^^^^^^^^^^^^^^^^^^^^^^

TODO

.. _comparison-to-generate-parameter-library:

Comparison to generate_parameter_library
-------------------------------------------

The existing `generate_parameter_library <https://github.com/pickNikRobotics/generate_parameter_library>`_ only implements the baseline
code-generation step. **It does not contain any of the runtime parameter management features.**
This library also implements additional features in the code-generation step. The major augmentations are summarized below, by section.

**Code-Generation:**

* Additional configuration provided by root yaml keys (described in table TODO).
* Support for mixins (ref TODO).
* Added float32 parameter type support.
* Separated data transfer object (dto) and parameter listener service classes into separate files to reduce dependencies in referencing code.

**Run-Time:**

* Hierarchical override system for runtime parameter files and command line arguments.
* Parameter expressions (ref TODO).
* Added concept of node configuration file for topic remappings and environment variables (ref TODO).
* Command line support for namespace and node selector syntax (ref TODO)