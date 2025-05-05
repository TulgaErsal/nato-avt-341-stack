ROS 1 Linux - Build From Source
==================================

Prerequisites
--------------------------
- Installation of `ROS1 Noetic <https://wiki.ros.org/noetic/Installation/Ubuntu>`_.
- Working ROS1 catkin workspace.

.. note::

    Make sure that ``setup.bash`` has been sourced in either the workspace's ``devel`` or
    ``install`` folder depending on if ``catkin_make`` or ``catkin_make
    install`` has been used respectively. Typically this command is added to
    ``~/.bashrc`` to automatically run on shell prompt startup.

    .. code-block:: shell

        source ~/<path_to_catkin_workspace>/[install|devel]/setup.bash

Dependency Installation
--------------------------

Install the following ROS packages via the command line:

.. code-block:: shell

    sudo apt-get install ros-noetic-pcl-ros ros-noetic-jsk-recognition-msgs ros-noetic-jsk-rviz-plugins ros-noetic-tf2-sensor-msgs ros-noetic-ackermann-msgs


MPC Local Planner Dependencies (Optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If using the stack's MPC local planner, additional dependencies must be installed. Refer to documentation at `TulgaErsal/AVT-341-MPC <https://github.com/TulgaErsal/AVT-341-MPC>`_ for more details.

A functioning `catkin workspace
<http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_ is required to build
and run this code.

Build Source Code
-------------------------------

Clone the repo into your ``<catkin_ws>/src`` directory:

.. code-block:: shell

    cd <catkin_ws>/src
    git clone https://github.com/TulgaErsal/nato-avt-341-stack.git avt_341_stack

From the top level catkin workspace directory, execute:

.. code-block:: shell

    catkin_make --only-pkg-with-deps avt_341

Test Installation
--------------------------

To test the installation, execute the following command. Rviz should appear with a mock scenario.

.. code-block:: shell

    roslaunch avt_341 example.launch

.. image:: images/rviz_mock_test_scenario.png
    :alt: RVIZ Mock Scenario