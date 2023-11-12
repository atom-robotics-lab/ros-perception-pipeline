#!/bin/bash

CONTAINER_NAME="object_detection"

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo "Attaching to running container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Run the docker container
docker run --gpus all --shm-size=1g --ulimit memlock=-1 --ulimit stack=67108864 \
-it --rm --privileged --net=host --ipc=host \
--name $CONTAINER_NAME \
-v $PWD/../../../src:/root/percep_ws/src \
-v $PWD/../../../models/:/root/percep_ws/models/ \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
object_detection:latest

