#!/bin/bash

export DISPLAY=:0.0

xhost +

cat ${HOME}/.Xauthority > ${HOME}/docker/.Xauthority
chmod 644 "${HOME}/docker/.Xauthority"

docker exec -it \
        -u ubuntu \
        -e DISPLAY=${DISPLAY} \
        -e XAUTHORITY=/home/ubuntu/.Xauthority \
        IntelPi \
        /bin/zsh
