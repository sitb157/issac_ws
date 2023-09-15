#!/bin/bash
xhost +local:

sudo docker run -it \
     --privileged \
     --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
     --env="DISPLAY=unix$DISPLAY" \
     --volume="/home/$USER:/home/$USER" \
     --volume="/dev:/dev" \
     --workdir="/home/$USER" \
     --net=host \
     --name ros2_container \
     spot_ros:foxy_noetic

xhost -local:docker
