#!/usr/bin/env bash
set -e 

source /opt/ros/humble/setup.bash
apt-get update
apt-get install -y \
    git \
    vim \
    sudo \
    curl \
    wget \
    ccache \
    usbutils \
    net-tools \
    iputils-ping \
    python3-pip \
    apt-transport-https \
    software-properties-common \
    ros-humble-launch-pytest \
    ros-humble-rmw-cyclonedds-cpp