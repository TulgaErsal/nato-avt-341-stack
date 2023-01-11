# AVT-341
ROS package with autonomy algorithms for the NATO AVT-341.

The MPC plugin is available at [https://github.com/TulgaErsal/AVT-341-MPC](https://github.com/TulgaErsal/AVT-341-MPC)

## Requirements
The stack will work with either ROS1 or ROS2. It has been built and tested on Ubuntu 16, 18, 20, and 22. 

## ROS-1 Installation
A functioning [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) is required to build and run this code.

Clone the repo into your catkin_ws/src directory with the following command.
```bash
git clone https://github.com/TulgaErsal/nato-avt-341-stack.git
```

Next, since you are building on ROS-1, copy the appropriate package file.
```bash
cp package_ros1.xml package.xml
```

From the top level catkin_ws directory, type
```bash
catkin_make install
```

Or, if you want to build only this package:
```bash
catkin_make --only-pkg-with-deps avt_341
```

__If user-defined workspace with default install spaces:__ Make sure that ```setup.bash``` has been sourced in either the workspace's ```devel``` or ```install``` folder depending on if ```catkin_make``` or ```catkin_make install``` has been used respectively. Typically this command is added to ```~/.bashrc``` so that it is called on opening a command prompt instead of being issued manually.  

Can be placed in ~/.bashrc also so does not need to be issued manually
```bash 
source ~/<path_to_catkin_workspace>/[install|devel]/setup.bash
```

Example (when built with catkin_make): 
```bash
source ~/catkin_ws/devel/setup.bash
```

To test the installation, type
```bash
roslaunch avt_341 example.launch
```

## ROS-2 Installation
A functioning [colcon workspace](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) is required to build and run this code.

Clone the repo into your catkin_ws/src directory with the following command.
```bash
git clone https://github.com/TulgaErsal/nato-avt-341-stack.git
```

Next, since you are building on ROS-2, copy the appropriate package file.
```bash
cp package_ros2.xml package.xml
```

From the top level ros2_ws directory, type
```bash
colcon build
```
Currently, colcon will echo some warning messages on the first build about deprecated point cloud libraries. Execute the build a second time:
```bash
colcon build
```
and these will be fised.

__If user-defined workspace with default install spaces:__ Make sure that ```ros2_ws/install/setup.bash``` has been sourced after the build. Typically this command is added to ```~/.bashrc``` so that it is called on opening a command prompt instead of being issued manually.  

Can be placed in ~/.bashrc also so does not need to be issued manually
```bash 
source ~/<path_to_colcon_workspace>/[install]/setup.bash
```

To test the installation, type
```bash
ros2 launch avt_341 example.launch.py
```

##  Troubleshooting
In ROS1, the package requires the ROS PointCloud Library (PCL) interface. If you get errors related to missing pcl header files, then you may need to install pcl_ros on your system.
```bash
sudo apt-get install ros-kinetic-pcl-ros
```

## Running with MAVS
To run an example simulation with MAVS, first [install and build MAVS](https://mavs-documentation.readthedocs.io/en/latest/MavsBuildInstructions/).

Next, install and build the [MAVS-ROS package](https://github.com/CGoodin/mavs_ros) (for ROS1) or the [MAVS-ROS2](https://github.com/CGoodin/mavs-ros2) package (for ROS2).

Next, clone the example MAVS simulation package.
```bash
git clone https://github.com/CGoodin/mavs_avt_example.git
```

To test in ROS-1:
```bash
roslaunch avt_341 mavs_example.launch
```

To test in ROS-2:
```bash
ros2 launch avt_341 mavs_example.launch.py
```

## Funding Acknowledgement
This project is made possible by technical and financial support of the Mississippi State University Center for Advanced Vehicular Systems as well as the Automotive Research Center (ARC) in accordance with Cooperative Agreement W56HZV 14 2 0001 U.S. Army CCDC Ground Vehicle Systems Center (GVSC) Warren, MI.
