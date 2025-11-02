#!/usr/bin/env bash

set -e

source ${HOME}/.bashrc

printf "Running entrypoint script ...\n"

printf "Sourcing ROS distribution ...\n"
source /opt/ros/humble/setup.bash

if [ -f ${HOME}/ROS/install/local_setup.bash ]; then
    printf "Pre-sourcing local workspace ...\n"
    source ${HOME}/ROS/install/local_setup.bash
else
    printf "Local workspace not available, skipping source operation ...\n"
fi

if [ -n "${DOCKER_ROS_BUILD}" ]
then
    printf "Running build ...\n"
    if [ -n "${DOCKER_ROS_CMAKE_ARGS}" ]
    then
        printf "Passing additional CMake arguments \"${DOCKER_ROS_CMAKE_ARGS}\"...\n"
        DOCKER_ROS_COLCON_CMAKE_ARGS="--cmake-args ${DOCKER_ROS_CMAKE_ARGS} --no-warn-unused-cli"
    fi
    colcon build ${DOCKER_ROS_COLCON_CMAKE_ARGS}
    source ${HOME}/ROS/install/local_setup.bash
fi

if [ "${DOCKER_ROS_EXEC}" == "node" ]
then
    if [ -n "${DOCKER_ROS_PARAMS}" ]
    then
        printf "Using custom parameters \"${DOCKER_ROS_PARAMS}\" ..."
        DOCKER_ROS_NODE_ARGS="--ros-args --params-file ${HOME}/ROS/config/params/${DOCKER_ROS_PARAMS}"
    fi

    if [ -n "${DOCKER_ROS_REMAPS}" ]
    then
        printf "Using remappings \"${DOCKER_ROS_REMAPS}\" ...\n"
        remappings=(`echo ${DOCKER_ROS_REMAPS}`)
        for remapping in "${remappings[@]}"
        do
            echo "Current remap: ${remapping}"
            DOCKER_ROS_REMAP_ARGS+="--remap ${remapping} "
        done
    fi

    ros2 run ${DOCKER_ROS_PACKAGE} ${DOCKER_ROS_NODE} ${DOCKER_ROS_NODE_ARGS} ${DOCKER_ROS_REMAP_ARGS}
elif [ "${DOCKER_ROS_EXEC}" == "launch" ]
then
    ros2 launch ${DOCKER_ROS_PACKAGE} ${DOCKER_ROS_LAUNCH_FILE}
elif [ "${DOCKER_ROS_EXEC}" == "play" ]
then
    if [ -n "${DOCKER_ROS_REMAPS}" ]
    then
        printf "Using remappings \"${DOCKER_ROS_REMAPS}\" ..."
        DOCKER_ROS_REMAP_ARGS="--remap ${DOCKER_ROS_REMAPS}"
    fi
    ros2 bag play --clock 1000 --loop ${DOCKER_ROS_BAG_PATH} ${DOCKER_ROS_REMAP_ARGS}
elif [ "${DOCKER_ROS_EXEC}" == "record" ]
then
    if [ -n "${DOCKER_ROS_BAG_NAME}" ]
    then
        DOCKER_ROS_BAG_NAME=bag
    fi
    printf "Starting recorder ...\n"
    mkdir -p ${DOCKER_ROS_BAG_PATH}
    ros2 bag record -s mcap -a -o ${DOCKER_ROS_BAG_PATH}/$(date +%s)-${DOCKER_ROS_BAG_NAME}
elif [ "${DOCKER_ROS_EXEC}" == "cli" ]
then
    /bin/bash -l
fi

printf "Done!\n"