#!/bin/bash

./base.sh

docker exec -it object_detection /launch_controller.sh

xhost -local:root
