ARG ROS_DISTRO=humble

# Use base image, https://hub.docker.com/_/ros/
FROM ros:$ROS_DISTRO-ros-base

# Prevent console from interacting with the user
# Read more here - https://bobcares.com/blog/debian_frontendnoninteractive-docker
ARG DEBIAN_FRONTEND=noninteractive

# Prevent hash mismatch error for apt-get update, qqq makes the terminal quiet while downloading pkgs
RUN apt-get clean && rm -rf /var/lib/apt/lists/* && apt-get update -yqqq

# Set folder for RUNTIME_DIR. Only to prevent warnings when running RViz2 and Gz
RUN mkdir tmp/runtime-root && chmod 0700 tmp/runtime-root
ENV XDG_RUNTIME_DIR='/tmp/runtime-root'

# https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils

# Non Python/ROS Dependencies
# apt-utils: https://stackoverflow.com/questions/51023312/docker-having-issues-installing-apt-utils

RUN apt-get update

RUN apt-get install --no-install-recommends -yqqq \
    apt-utils \
    vim \
    git

# Python Dependencies
RUN apt-get install --no-install-recommends -yqqq \
    python3-pip

# Addtional DDS dependencies
RUN apt-get install --no-install-recommends -yqqq \
    ros-$ROS_DISTRO-cyclonedds \
    ros-$ROS_DISTRO-rmw-cyclonedds-cpp

# Using shell to use bash commands like 'source'
SHELL ["/bin/bash", "-c"]

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/percep_ws

# Add target workspace in environment
ENV WORKSPACE=$WORKSPACE

# Create folders
RUN mkdir -p $WORKSPACE/src && \
cd $WORKSPACE/src && \
git clone git@github.com:atom-robotics-lab/ros-perception-pipeline.git && \
cd ros-perception-pipeline && \
rm -rf perception_bringup && \
cd ~

RUN mkdir -p /build_scripts
COPY $WORKSPACE/src/ros-perception-pipeline/docker_scripts build_scripts

# Another possiblity is to create a metapackage and run rosdep, this saves time in next step
# Since dependencies are preinstalled and only build is missing
#COPY object_detection $WORKSPACE/src/

# Pip installing requirements
RUN pip install -r $WORKSPACE/src/ros-perception-pipeline/object_detection/requirements.txt

# One time rosdep installs for the meta package
# @TODO: This can be optimized bby creating metapackage for faster builds
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $WORKSPACE && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build

# Copy over bash scripts to root directory
#COPY docker_scripts/bash_scripts/ /

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# For .bashrc
ENV ROS_DISTRO=$ROS_DISTRO

# Gazebo Fortress
#RUN apt-get install --no-install-recommends -yqqq \
    #ros-$ROS_DISTRO-ros-gz-sim \
    #ros-$ROS_DISTRO-ros-gz-interfaces \
    #ros-$ROS_DISTRO-ros-gz-bridge


# Install ros_gz
#RUN apt install -y ros-humble-ros-ign

# Install Rviz2
#RUN apt install -y ros-humble-rviz2

# Install xacro 
#RUN apt-get install ros-humble-xacro


# Update .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> /root/.bashrc
