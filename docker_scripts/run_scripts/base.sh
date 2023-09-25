#!/bin/bash

# http://wiki.ros.org/docker/Tutorials/GUI
xhost +local:root

./stop.sh

docker run -t -d --privileged=true --net=host \
--name object_detection \
-v $PWD/../../../models:/root/percep_ws/models \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
-v $PWD/publisher.yaml:/root/percep_ws/install/object_detection/share/object_detection/config/publisher.yaml \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
object_detection:latest
