Building on ROS 1
=================

The MPC plugin is available at `TulgaErsal/AVT-341-MPC
<https://github.com/TulgaErsal/AVT-341-MPC>`_.

The stack has been built and tested on Ubuntu 16.04, 18.04 and 20.04.

Installing dependencies
^^^^^^^^^^^^^^^^^^^^^^^

The package requires the ROS PointCloud Library (PCL) interface. If you get
errors related to missing pcl header files, then you may need to install pcl_ros
on your system.

.. code-block:: shell

    sudo apt-get install ros-noetic-pcl-ros


Preparing the workspace
^^^^^^^^^^^^^^^^^^^^^^^

A functioning `catkin workspace
<http://wiki.ros.org/catkin/Tutorials/create_a_workspace>`_ is required to build
and run this code.

Install dependencies:

.. code-block:: shell

    sudo apt install ros-<version>-jsk-recognition-msgs
    sudo apt install ros-<version>-jsk-rviz-plugins
    sudo apt install ros-<version>-tf2-sensor-msgs

Clone the repo into your `catkin_ws/src` directory with the following command.

.. code-block:: shell

    git clone https://github.com/TulgaErsal/nato-avt-341-stack.git

Next, since you are building on ROS-1, copy the appropriate package file.

.. code-block:: shell

    cd ./nato-avt-341-stack
    cp ./avt_341/package_ros1.xml ./avt_341/package.xml
    cp ./avt_341_msgs/package_ros1.xml ./avt_341_msgs/package.xml

Building the packages
^^^^^^^^^^^^^^^^^^^^^

From the top level catkin workspace directory, type

.. code-block:: shell

    catkin_make install

Or, if you want to build only this package:

.. code-block:: shell

    catkin_make --only-pkg-with-deps avt_341

__If user-defined workspace with default install spaces:__ Make sure that
```setup.bash``` has been sourced in either the workspace's ```devel``` or
```install``` folder depending on if ```catkin_make``` or ```catkin_make
install``` has been used respectively. Typically this command is added to
```~/.bashrc``` so that it is called on opening a command prompt instead of
being issued manually.

Can be placed in ~/.bashrc also so does not need to be issued manually

.. code-block:: shell

    source ~/<path_to_catkin_workspace>/[install|devel]/setup.bash

Example (when built with catkin_make):

.. code-block:: shell

    source ~/catkin_ws/devel/setup.bash

Testing the installation
^^^^^^^^^^^^^^^^^^^^^^^^

To test the installation, type

.. code-block:: shell

    roslaunch avt_341 example.launch
