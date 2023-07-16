#!/bin/bash

./base.sh

docker exec -it object_detection /home/mediapipe/Desktop/ros_workspaces/percep_ws/src/ros-perception-pipeline/docker_scripts/bash_scripts/launch_controller.sh

xhost -local:root
