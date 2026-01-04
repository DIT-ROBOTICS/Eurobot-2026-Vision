FROM ros:humble AS base

# Keep apt cache to speed up subsequent builds
RUN rm -f /etc/apt/apt.conf.d/docker-clean; echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

# Install common tools
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    curl \
    git \
    sudo \
    usbutils \
    v4l-utils \
    htop \
    iputils-ping \
    wget \
    net-tools \
    tmux \
    vim \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Install Python pip
RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-image-transport-plugins \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-pcl-ros \
    ros-humble-octomap-ros \
    ros-humble-octomap-msgs \
    ros-humble-rviz2 \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen \
    ros-humble-rosbridge-suite \
    ros-humble-foxglove-bridge \
    ros-humble-pcl-conversions \
    cmake \
    build-essential \
    libceres-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libopencv-dev \
    libsuitesparse-dev \
    && rm -rf /var/lib/apt/lists/*

ARG USERNAME=vision
ARG USER_UID=1000

RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash -u $USER_UID -G sudo $USERNAME \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

USER $USERNAME

COPY --chown=$USERNAME:$USERNAME \
    modules/install_realsense.sh /tmp/install_realsense.sh

RUN --mount=type=cache,target=/var/cache/apt,sharing=private \
    sudo apt-get update && sudo apt-get install -y dos2unix && \
    dos2unix /tmp/install_realsense.sh && \
    bash /tmp/install_realsense.sh && \
    sudo rm /tmp/install_realsense.sh

ENTRYPOINT []
CMD ["/bin/bash"]