#!/usr/bin/env bash
#
# Typical usage: ./join.bash subt
#

IMG=erickiemvp/ros:melodic_essential

xhost +
containerid=$(docker ps -aqf "ancestor=${IMG}") && echo $containerid
docker exec -it \
    --privileged \
    -e DISPLAY=${DISPLAY} \
    -e LINES="$(tput lines)" \
    ${containerid} \
    bash
xhost -