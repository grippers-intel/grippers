#!/bin/bash

export DISPLAY=:0.0

xhost +

touch "${HOME}/docker/.Xauthority"
chmod 644 "${HOME}/docker/.Xauthority"

docker run -dit \
    --name IntelPi \
    --privileged \
    --restart always \
    --network=host \
    -e DISPLAY=${DISPLAY} \
    -e ROS_DOMAIN_ID=21 \
    -e XAUTHORITY=/home/ubuntu/.Xauthority \
    -v ${HOME}/docker/.Xauthority:/home/ubuntu/.Xauthority:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ${HOME}/docker/shared/grippers/ros2_ws:/ros2_ws \
    -v ${HOME}/docker/shared:/shared \
    -v ${HOME}/docker/shared/grippers:/grippers \
    -v ${HOME}/docker/shared/grippers/third_party:/third_party \
    -v /dev:/dev \
    intelpi:latest \
    tail -f /dev/null
