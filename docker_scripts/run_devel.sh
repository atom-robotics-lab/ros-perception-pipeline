#!/bin/bash

xhost +local:root

IMAGE_NAME="object_detection"
IMAGE_TAG="latest"
CONTAINER_NAME="object_detection"

# Build the image if it doesn't exist
if docker images "$IMAGE_NAME:$IMAGE_TAG" | grep -q "$IMAGE_NAME[[:space:]]+$IMAGE_TAG"; then
    echo "The image $IMAGE_NAME:$IMAGE_TAG exists."

else
    echo "The image $IMAGE_NAME:$IMAGE_TAG does not exist. Building the image...."
    echo "Enter your preferred CUDA Version (default set to 11.8.0) : "
    read cuda_version

    # If the user input is blank, use 11.8.0 as the cuda_version
    if [ -z "$cuda_version" ]; then
        cuda_version="11.8.0"
    fi
    cd ..
    docker build --build-arg cuda_version=$cuda_version -t $IMAGE_NAME:$IMAGE_TAG .
    cd docker_scripts
fi

# Enter into the container if it is already running
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

