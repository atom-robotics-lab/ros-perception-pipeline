# Use cuda_version arg to take CUDA version as input from user
ARG cuda_version=12.2.0

# Use NVIDA-CUDA's base image
FROM nvcr.io/nvidia/cuda:${cuda_version}-devel-ubuntu22.04 

# Prevent console from interacting with the user
ARG DEBIAN_FRONTEND=noninteractive

# Prevent hash mismatch error for apt-get update, qqq makes the terminal quiet while downloading pkgs
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR. Only to prevent warnings when running RViz2 and Gz
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

RUN apt-get update

RUN apt-get install --no-install-recommends -yqqq \
    apt-utils \
    nano \
    git

# Using shell to use bash commands like 'source'
SHELL ["/bin/bash", "-c"]

# Python Dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

# Install ROS 2 Humble

# Add locale
RUN locale  && \
    apt update && apt install locales  -y && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale 

# Setup the sources
RUN apt-get update && apt-get install -y software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/nul

RUN apt update && apt install -y && \
    apt install ros-humble-desktop -y && \
    apt install ros-dev-tools -y

# Install cv-bridge
RUN apt install -y ros-humble-cv-bridge

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/percep_ws

# Add target workspace in environment
ENV WORKSPACE=$WORKSPACE

# Creating the models folder
RUN mkdir -p $WORKSPACE/models && \
    mkdir -p $WORKSPACE/src

# Create folders and setting up the project
COPY object_detection/requirements.txt /requirements.txt
RUN  pip3 install -r requirements.txt

# ROS Dependencies
RUN apt-get install --no-install-recommends -y \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# Update .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

