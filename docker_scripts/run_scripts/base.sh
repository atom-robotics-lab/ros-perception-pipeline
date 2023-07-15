#!/bin/bash

# http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:root

./stop.sh

docker run -t -d --privileged --net=host \
--name object_detection \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
-v $PWD/publisher.yaml:/root/percep_ws/install/object_detection/share/object_detection/config/publisher.yaml \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env ROS_DOMAIN_ID=11 \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
object_detection:latest
