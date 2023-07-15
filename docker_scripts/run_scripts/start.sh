#!/bin/bash

./base.sh

docker exec -it object_detection /launch_gazebo.sh

xhost -local:root
