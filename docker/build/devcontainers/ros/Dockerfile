FROM ubuntu:focal

# Avoid interactive prompts during the build.
ARG DEBIAN_FRONTEND=noninteractive

# Update the distribution and install the system dependencies.
RUN apt-get update && \
    apt-get install \
        --yes \
        curl \
        git \
        gpg \
        lsb-release && \
    rm -rf /var/lib/apt/lists/*

# Add a lower
RUN groupadd --gid 1000 avt341 && \
    useradd --uid 1000 --gid 1000 -m avt341

# Add the ROS Noetic APT repository and install the desktop distribution.
RUN echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && \
    apt-get install \
        --yes \
        ros-noetic-desktop && \
    echo "source /opt/ros/noetic/setup.bash" >> /home/avt341/.bashrc && \
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

# Update the ROS workspace ownership and nstall all dependencies via rosdep. To
# source the ROS distribution, we temporarily switch the container build user to
# a Bash shell.
SHELL ["/bin/bash", "-c"]
RUN chown -R avt341:avt341 /opt/ros/ws && \
    apt-get update && \
    source /opt/ros/noetic/setup.bash && \
    rosdep install \
        -y \
        --from-paths src \
        --ignore-src

# Set the default container user and the default shell to a Bash login shell
USER avt341
SHELL ["/bin/bash", "-l", "-c"]

# Build the "avt_341" and "avt_341_msgs" ROS packages.
RUN source /opt/ros/noetic/setup.bash && \
    catkin_make install

# Copy and register the entrypoint script.
COPY docker/build/devcontainers/ros/entrypoint.sh /usr/local/share/docker/entrypoint.sh
ENTRYPOINT ["/bin/bash", "/usr/local/share/docker/entrypoint.sh"]