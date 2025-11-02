ARG TARGET=cpu
ARG USER=avt

FROM ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel AS cpu

FROM ghcr.io/tulgaersal/nato-avt-341-stack:ros2-devel-cuda AS cuda

FROM ${TARGET} AS common

ARG USER

###############################################################################
# Build and install the local version of the autonomy stack                   #
###############################################################################

COPY avt_341 \
     /home/${USER}/ROS/src/avt_341
COPY avt_341_msgs \
     /home/${USER}/ROS/src/avt_341_msgs

WORKDIR /home/${USER}/ROS

RUN source /opt/ros/humble/setup.bash && \
    colcon build


###############################################################################
# Set up the container entrypoint script                                      #
###############################################################################

COPY --chown=${USER}:${USER} \
     --chmod=+x \
     docker/build/ros2/runtime/entrypoint.sh \
     /home/${USER}/entrypoint.sh

ENTRYPOINT /home/avt/entrypoint.sh


###############################################################################
# Define container labels                                                     #
###############################################################################

LABEL org.opencontainers.image.source=https://github.com/TulgaErsal/nato-avt-341-stack.git
LABEL org.opencontainers.image.revision=main
LABEL org.opencontainers.image.name=nato-avt-341-stack

