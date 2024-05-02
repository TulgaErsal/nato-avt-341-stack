FROM ubuntu:focal

# Avoid interactive prompts during the build.
ARG DEBIAN_FRONTEND=noninteractive

# Set the default shell to a Bash login shell.
SHELL ["/bin/bash", "-l", "-c"]

# Update the distribution and install the system dependencies.
RUN apt-get update && \
    apt-get install \
        --yes \
        curl \
        git \
        gpg \
        lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Add the ROS Noetic APT repository and install the desktop distribution.
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install \
        --yes \
        ros-noetic-desktop && \
    echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    rm -rf /var/lib/apt/lists/*

# Install and configure rosdep.
RUN apt-get update && \
    apt-get install \
        --yes \
        python3-rosdep && \
    rosdep init && \
    rosdep update && \
    rm -rf /var/lib/apt/lists/*

# Copy the "avt_341" and "avt_341_msgs" packages into the catkin workspace
COPY avt_341 /opt/ros/ws/src/TulgaErsal/nato-avt-341-stack/avt_341
COPY avt_341_msgs /opt/ros/ws/src/TulgaErsal/nato-avt-341-stack/avt_341_msgs

# Set the working directory to the root of the catkin workspace.
WORKDIR /opt/ros/ws

# Install all dependencies via rosdep and build the "avt_341" and "avt_341_msgs" ROS packages.
RUN sudo apt-get update && \
    source /opt/ros/noetic/setup.bash && \
    ln -sf \
        /opt/ros/ws/src/TulgaErsal/nato-avt-341-stack/avt_341/package_ros1.xml \
        /opt/ros/ws/src/TulgaErsal/nato-avt-341-stack/avt_341/package.xml && \
    ln -sf \
        /opt/ros/ws/src/TulgaErsal/nato-avt-341-stack/avt_341_msgs/package_ros1.xml \
        /opt/ros/ws/src/TulgaErsal/nato-avt-341-stack/avt_341_msgs/package.xml && \
    rosdep install \
        -y \
        --from-paths src \
        --ignore-src && \
    catkin_make install

# Copy the entrypoint script.
COPY docker/build/devcontainers/ros/entrypoint.sh /usr/local/share/docker/entrypoint.sh

# Register the entrypoint script.
ENTRYPOINT ["/bin/bash", "/usr/local/share/docker/entrypoint.sh"]