#!/bin/sh
set -e

USER=$1
USER_UID=$2
USER_GID=$3

addgroup --gid "$USER_GID" "$USER"
adduser --uid "$USER_UID" --gid "$USER_GID" --disabled-password --gecos "" "$USER"

mkdir -p /etc/sudoers.d
echo "$USER ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/$USER"
chmod 0440 "/etc/sudoers.d/$USER"

mkdir -p "/home/$USER"
chown "$USER:$USER" "/home/$USER"

echo "source /opt/ros/humble/setup.bash" >> "/home/$USER/.bashrc"
