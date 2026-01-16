#!/bin/bash
set -e

if [ -z "$REALSENSE" ]; then
    echo "Skipping RealSense installation as REALSENSE is not set"
    exit 0
fi

if [ "$REALSENSE" = "YES" ]; then
    echo "Installing RealSense ros packages for ROS distro: ${ROS_DISTRO:-humble}"

    sudo apt-get update && sudo apt-get install -y \
        ros-humble-point-cloud-transport \
        ros-humble-librealsense2* \
        && sudo rm -rf /var/lib/apt/lists/*
fi

echo "RealSense installation completed successfully!"
