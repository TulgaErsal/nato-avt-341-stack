#!/bin/bash

# Disable hardware acceleration to fix glitches and disable warnings in RViz.
export LIBGL_ALWAYS_SOFTWARE=1

# Source the ROS Noetic distribution.
source /opt/ros/noetic/setup.bash

# Build and source the catkin workspace.
catkin_make install && \
source install/setup.bash

# Run the launch file (from command line, from mounted file, or a fallback
# example)
if [[ ! -z "${1}" ]]; then
    roslaunch avt_341 ${1}.launch
elif [ -f "src/avt_341/launch/devcontainer.launch" ]; then
    roslaunch avt_341 devcontainer.launch
else
    roslaunch avt_341 example.launch
fi