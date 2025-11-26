#!/bin/bash
set -e

echo "Installing ROS2 related tools for ROS distro: ${ROS_DISTRO:-humble}"

sudo apt-get update && sudo apt-get install -y \
        ros-humble-ros-base \
        ros-humble-image-transport-plugins \
        ros-humble-cv-bridge \
        ros-humble-vision-msgs \
        ros-humble-pcl-ros \
        ros-humble-octomap-ros \
        ros-humble-octomap-msgs \
        ros-humble-rviz2 \
        ros-humble-tf2-ros \
        ros-humble-tf2-geometry-msgs \
        ros-humble-tf2-eigen \
    && sudo rm -rf /var/lib/apt/lists/*

echo "Ros2 related tool installation completed successfully!"