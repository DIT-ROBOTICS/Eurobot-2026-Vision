#!/usr/bin/env bash
set -e 

USER=$1
ROS_WS=/home/${USER}/vision_ws

. "/opt/ros/humble/setup.sh"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    echo "[rosdep] initializing..."
    sudo rosdep init
else
    echo "[rosdep] already initialized, skip"
fi

apt-get update
rosdep update
rosdep install \
    --from-paths ${ROS_WS}/src \
    --rosdistro humble -y \
    --ignore-src \
    --skip-keys="librealsense2 librealsense2-dev"

