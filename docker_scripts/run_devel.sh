#!/bin/bash

# Stop the container if it is already running
docker stop object_detection && docker rm object_detection

# Run the docker container
docker run -it --privileged --net=host --ipc=host \
--name object_detection \
-v $PWD/../../../src:/root/percep_ws/src \
-v $PWD/../../../models/:/root/percep_ws/models/ \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
object_detection:latest
