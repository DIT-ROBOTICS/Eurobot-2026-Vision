#!bin/sh
. "/opt/ros/humble/setup.sh"
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
    ros-humble-rmw-cyclonedds-cpp \
    
apt-get clean -y 
rm -rf /var/lib/apt/lists/*

