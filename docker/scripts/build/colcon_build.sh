#!/usr/bin/env bash
set -e 

USER=$1
ROS_WS=/home/${USER}/vision_ws

source /opt/ros/humble/setup.bash
cd ${ROS_WS}
colcon build
chown -R ${USER}:${USER} ${ROS_WS}
echo "source ${ROS_WS}/install/setup.bash" >> /home/${USER}/.bashrc