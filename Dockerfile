# Use cuda_version arg to take CUDA version as input from user
ARG cuda_version=11.8.0

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

# Install build dependencies for building OpenCV from source
RUN apt-get clean && \
    apt-get update && \
    apt-get install -y --no-install-recommends --fix-missing \
        build-essential binutils \
        ca-certificates cmake cmake-qt-gui curl \
        dbus-x11 \
        ffmpeg \
        gdb gcc g++ gfortran git \
        tar \
        lsb-release \
        procps \
        manpages-dev \
        unzip \
        zip \
        wget \
        xauth \
        swig \
        python3-pip python3-dev python3-numpy python3-distutils \
        python3-setuptools python3-pyqt5 python3-opencv \
        libboost-python-dev libboost-thread-dev libatlas-base-dev libavcodec-dev \
        libavformat-dev libavutil-dev libcanberra-gtk3-module libeigen3-dev \
        libglew-dev libgl1-mesa-dev libgl1-mesa-glx libglib2.0-0 libgtk2.0-dev \
        libgtk-3-dev libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev liblapack-dev \
        liblapacke-dev libopenblas-dev libopencv-dev libpng-dev libpostproc-dev \
        libpq-dev libsm6 libswscale-dev libtbb-dev libtbb2 libtesseract-dev \
        libtiff-dev libtiff5-dev libv4l-dev libx11-dev libxext6 libxine2-dev \
        libxrender-dev libxvidcore-dev libx264-dev libgtkglext1 libgtkglext1-dev \
        libvtk9-dev libdc1394-dev libgstreamer-plugins-base1.0-dev \
        libgstreamer1.0-dev libopenexr-dev \
        openexr \
        pkg-config \
        qv4l2 \
        v4l-utils \
        zlib1g-dev \
        && apt-get clean

# Build OpenCV from source
ARG OPENCV_VERSION="4.8.0"

WORKDIR /opencv
RUN wget -O opencv.zip https://github.com/opencv/opencv/archive/${OPENCV_VERSION}.zip \
    && wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/${OPENCV_VERSION}.zip \
    && unzip opencv.zip \
    && unzip opencv_contrib.zip \
    && mv opencv-${OPENCV_VERSION} opencv \
    && mv opencv_contrib-${OPENCV_VERSION} opencv_contrib

RUN mkdir /opencv/opencv/build
WORKDIR /opencv/opencv/build

RUN cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_PYTHON_EXAMPLES=ON \
    -D INSTALL_C_EXAMPLES=ON \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D WITH_CUDA=ON \
    -D WITH_CUDNN=ON \
    -D OPENCV_DNN_CUDA=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D CUDA_ARCH_BIN=8.6 \
    -D WITH_CUBLAS=1 \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_EXTRA_MODULES_PATH=/opencv/opencv_contrib/modules \
    -D PYTHON_EXECUTABLE=/usr/local/bin/python \
    -D BUILD_EXAMPLES=ON \
    -D WITH_LAPACK=OFF ..

RUN make -j8
RUN make install 
RUN ldconfig

# Add locale
RUN locale  && \
    apt update && apt install --no-install-recommends -yqqq locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8 && \
    locale  

# Setup the sources
RUN apt-get update && apt-get install --no-install-recommends -yqqq software-properties-common curl && \
    add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/nul

# Install ROS 2 Humble
RUN apt update && apt install --no-install-recommends -yqqq \
    ros-humble-desktop \
    ros-dev-tools

# Install cv-bridge
RUN apt update && apt install --no-install-recommends -yqqq \
    ros-humble-cv-bridge

# Target workspace for ROS2 packages
ARG WORKSPACE=/root/percep_ws

# Add target workspace in environment
ENV WORKSPACE=$WORKSPACE

# Creating the models folder
RUN mkdir -p $WORKSPACE/models && \
    mkdir -p $WORKSPACE/src

# Installing Python dependencies
COPY object_detection/requirements.txt .
RUN  pip3 install -r requirements.txt

# ROS Dependencies
RUN apt update && apt install --no-install-recommends -yqqq \
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp

# Use cyclone DDS by default
ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

WORKDIR /root/percep_ws

# Update .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

