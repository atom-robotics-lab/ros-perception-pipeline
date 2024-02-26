#!/bin/bash

xhost +local:root

IMAGE_NAME="perception_pipeline"
IMAGE_TAG="latest"
CONTAINER_NAME="perception_pipeline"

# Build the image if it doesn't exist
if docker inspect "$IMAGE_NAME:$IMAGE_TAG" &> /dev/null; then
    echo "The image $IMAGE_NAME:$IMAGE_TAG exists."

else
    echo "The image $IMAGE_NAME:$IMAGE_TAG does not exist. Building the image...."

    echo "Choose the base image:"
    echo "1. NVIDIA CUDA image"
    echo "2. Ubuntu 22.04 image"
    read -p "Enter your choice (1 or 2): " base_image_choice

    # If the user input is blank or not 1 or 2, default to NVIDIA CUDA image
    if [ -z "$base_image_choice" ] || [ "$base_image_choice" != "1" ] && [ "$base_image_choice" != "2" ]; then
        base_image_choice="1"
    fi

    # Choose the appropriate Dockerfile based on user input
    if [ "$base_image_choice" == "1" ]; then
        DOCKERFILE="Dockerfile.cuda"

        echo "Enter your preferred CUDA Version (default set to 11.8.0) : "
        read cuda_version

        # If the user input is blank, use 11.8.0 as the cuda_version
        if [ -z "$cuda_version" ]; then
            cuda_version="11.8.0"
        fi

        cd ..
        docker build --build-arg cuda_version="$cuda_version" -f "$DOCKERFILE" -t "$IMAGE_NAME:$IMAGE_TAG" .
        echo "Completed building the docker image"
    else
        DOCKERFILE="Dockerfile.ubuntu"

        cd ..
        docker build -f "$DOCKERFILE" -t "$IMAGE_NAME:$IMAGE_TAG" .
    fi
fi

# Enter into the container if it is already running
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo -e "\nAttaching to running container: $CONTAINER_NAME"
    docker exec -it $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Check if the PERCEP_WS_PATH environment variable is empty
if [ -z "$PERCEP_WS_PATH" ]; then
    echo -e "\nThe environment variable : PERCEP_WS_PATH is empty. Point it to the path of the ROS 2 workspace in which the ros-perception-pipeline project is kept !!"
    exit 1
fi

# Run the docker container
docker run --gpus all --shm-size=1g --ulimit memlock=-1 --ulimit stack=67108864 \
-it --rm --privileged --net=host --ipc=host \
--name $CONTAINER_NAME \
-v $PERCEP_WS_PATH/src/:/root/percep_ws/src \
-v $PERCEP_WS_PATH/models/:/root/percep_ws/models/ \
-v ddsconfig.xml:/ddsconfig.xml \
--env ROS_LOG_DIR=$PERCEP_WS_PATH/logs \
--env CYCLONEDDS_URI=/ddsconfig.xml \
--env="QT_X11_NO_MITSHM=1"  \
--env="DISPLAY"  \
$IMAGE_NAME:$IMAGE_TAG
