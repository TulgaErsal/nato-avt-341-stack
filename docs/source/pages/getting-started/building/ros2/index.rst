Building on ROS 2
=================

The stack has been built and tested on Ubuntu 20.04 and 22.04.


Preparing the workspace
^^^^^^^^^^^^^^^^^^^^^^^

A functioning `colcon workspace
<https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html>`_
is required to build and run this code.

Clone the repo into your catkin_ws/src directory with the following command:

.. code-block:: shell

    git clone https://github.com/TulgaErsal/nato-avt-341-stack.git

Next, since you are building on ROS 2, copy the appropriate package file:

.. code-block:: shell
    
    cd ./nato-avt-341-stack
    cp ./avt_341/package_ros2.xml ./avt_341/package.xml
    cp ./avt_341_msgs/package_ros2.xml ./avt_341_msgs/package.xml

Building the packages
^^^^^^^^^^^^^^^^^^^^^

From the top level ROS workspace directory, type:

.. code-block:: shell

    colcon build

Currently, colcon will echo some warning messages on the first build about
deprecated point cloud libraries. Execute the build a second time:

.. code-block:: shell

    colcon build

and these will be fixed.

__If user-defined workspace with default install spaces:__ Make sure that
```ros2_ws/install/setup.bash``` has been sourced after the build. Typically
this command is added to ```~/.bashrc``` so that it is called on opening a
command prompt instead of being issued manually.

This may also be placed in ~/.bashrc also so that it does not need to be issued
manually:

.. code-block:: shell

    source ~/<path_to_colcon_workspace>/[install]/setup.bash

Testing the installation
^^^^^^^^^^^^^^^^^^^^^^^^

To test the installation, type
.. code-block:: shell

    ros2 launch avt_341 example.launch.py