Introduction
============

ROS package with autonomy algorithms for the NATO AVT-341.

.. todo::

    This section requires more information.

Repository packages
-------------------

The repository is a ROS workspace source folder holding the following packages:

.. list-table::
   :header-rows: 1
   :widths: 28 72

   * - Package
     - Description
   * - ``avt_341``
     - Metapackage for the stack. Can be used to conveniently build and install all major packages in repository.
   * - ``avt_341_nav``
     - Set of core autonomy algorithms implementing major navigation tasks: slam,
       perception mission, global, local planning, and control.
   * - ``avt_341_msgs``
     - Custom ROS message and service definitions.
   * - ``avt_341_bringup``
     - Launch files, parameter overrides and deployment data used to the launch autonomy stack.
   * - ``avt_341_param_lib``
     - | Parameter management library.
       | Refer to :doc:`/pages/configuration/parameter_library` for more details.
   * - ``avt_341_rviz_plugins``
     - Custom rviz plugins associated with the autonomy stack. Monitor stack status, issue commands, and visualize custom messages.
   * - ``avt_341_system_tests``
     - | System level tests typically requiring multiple nodes to run.
       | Refer to :doc:`/pages/testing/index` for more details.

Every package builds with ``ament_cmake``. Each one lives in the
similarly named folder at the root of the repository, except
``avt_341_param_lib`` and ``avt_341_param_lib_example``, which are both nested
under the ``avt_341_param_lib/`` folder.

The repository additionally references the ``avt_341_depth`` and
``avt_341_ganav`` git submodules, which supply optional perception modules.
They are empty until the submodules are initialized and are not part of the
default build.

Outside the ROS packages, the repository also carries the ``docs/`` sources for
this documentation, ``docker/`` build and compose files for the containerized
setups, and ``tools/`` development scripts.
