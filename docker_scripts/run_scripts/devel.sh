#!/bin/bash

# http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:root

./stop.sh

docker run -t -d --privileged --net=host --ipc=host \
--name object_detection \
-v $PWD/../../../../../percep_ws/src:/root/percep_ws/src \
-v $PWD/../../../models/:/root/percep_ws/models/ \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
object_detection:latest
