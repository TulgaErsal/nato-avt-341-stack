###############################################################################
# Build arguments                                                             #
###############################################################################

ARG DEBIAN_FRONTEND=noninteractive
ARG LOCALE=en_US
ARG ROS_DISTRIBUTION=humble
ARG TARGET=cpu
ARG TORCH_URL
ARG USER=avt

FROM ubuntu:22.04 AS cpu

ARG TORCH_URL=https://download.pytorch.org/libtorch/cpu/libtorch-shared-with-deps-2.8.0%2Bcpu.zip

FROM nvidia/cuda:12.4.0-devel-ubuntu22.04 AS cuda

ARG TORCH_URL=https://download.pytorch.org/libtorch/cu124/libtorch-cxx11-abi-shared-with-deps-2.6.0%2Bcu124.zip

FROM ${TARGET} AS common

ARG APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE
ARG DEBIAN_FRONTEND
ARG LOCALE
ARG ROS_DISTRIBUTION
ARG TORCH_URL
ARG USER

# Set up locales
RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        locales && \
    locale-gen ${LOCALE} ${LOCALE}.UTF-8 && \
    update-locale LC_ALL=${LOCALE}.UTF-8 LANG=${LOCALE}.UTF-8 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install ROS dependencies
RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        curl \
        software-properties-common && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install the ROS distribution
RUN ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${VERSION_CODENAME})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get install --no-install-recommends --yes \
        ros-dev-tools \
        ros-${ROS_DISTRIBUTION}-desktop \
        ros-${ROS_DISTRIBUTION}-rosbag2-storage-mcap && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


###############################################################################
# Download and install the LibTorch runtime and libraries                     #
###############################################################################

RUN apt-get update && \
    apt-get install --no-install-recommends --yes \
        ros-${ROS_DISTRIBUTION}-ackermann-msgs \
        ros-${ROS_DISTRIBUTION}-pcl-ros \
        ros-${ROS_DISTRIBUTION}-vision-msgs && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
    apt-get install --no-install-recommends \
        unzip && \
    curl -L ${TORCH_URL} -o torch.zip && \
    unzip torch.zip && \
    cp -r libtorch/* /usr/local/ && \
    ldconfig && \
    rm -R libtorch/ torch.zip && \
    apt-get clean && rm -rf /var/lib/apt/lists/*


###############################################################################
# Download and install the Julia language runtime and libraries               #
###############################################################################

ADD https://julialang-s3.julialang.org/bin/linux/x64/1.5/julia-1.5.4-linux-x86_64.tar.gz \
    /tmp/julia-1.5.4-linux-x86_64.tar.gz

RUN cd /tmp && \
    tar -xvf julia-1.5.4-linux-x86_64.tar.gz && \
    rm julia-1.5.4/LICENSE.md && \
    cp -R julia-1.5.4/** /usr/local && \
    rm -rf julia*


###############################################################################
# Add unprivileged user                                                       #
###############################################################################

RUN useradd --create-home --shell /bin/bash ${USER} && \
    mkdir -p /home/${USER}/ROS && \
    chown -R ${USER}:${USER} /home/${USER}/ROS && \
    echo "export LANG=${LOCALE}.UTF-8" >> /home/${USER}/.profile && \
    echo "source /opt/ros/${ROS_DISTRIBUTION}/setup.bash" >> /home/${USER}/.profile

# Set the default container user
USER ${USER}

# Set the working directory to the home folder of the container user
WORKDIR /home/${USER}/ROS

# Set the default shell to a Bash login shell
SHELL ["/bin/bash", "-l", "-c"]


###############################################################################
# Set up the ROS workspace                                                    #
###############################################################################

RUN mkdir -p ${HOME}/ROS/{build,install,log,src}

VOLUME ["/home/${USER}/ROS/build", "/home/${USER}/ROS/install"]


###############################################################################
# Define container labels                                                     #
###############################################################################

LABEL org.opencontainers.image.source=https://github.com/TulgaErsal/nato-avt-341-stack.git
LABEL org.opencontainers.image.revision=main
LABEL org.opencontainers.image.name=nato-avt-341-stack

