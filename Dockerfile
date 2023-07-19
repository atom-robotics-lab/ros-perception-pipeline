ARG ROS_DISTRO=humble

# Use base image, https://hub.docker.com/_/ros/
FROM ros:$ROS_DISTRO-ros-base

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

# Create folders and setting up the project
RUN mkdir -p $WORKSPACE/src && \
cd $WORKSPACE/src && \
git clone -b topguns/dockerfile https://github.com/atom-robotics-lab/ros-perception-pipeline.git && \ 
cd ros-perception-pipeline && \
rm -rf perception_bringup && \
cd object_detection && \
pip install -r requirements.txt  

# Creating the models folder
RUN mkdir -p $WORKSPACE/models


RUN mkdir -p /build_scripts/ 
RUN cp -r $WORKSPACE/src/ros-perception-pipeline/docker_scripts/bash_scripts/launch_controller.sh /


# One time rosdep installs for the meta package
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    cd $WORKSPACE && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# For .bashrc
ENV ROS_DISTRO=$ROS_DISTRO

# Install cv-bridge
RUN apt install -y ros-humble-cv-bridge


# Update .bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc && \
    echo "source $WORKSPACE/install/setup.bash" >> /root/.bashrc 

