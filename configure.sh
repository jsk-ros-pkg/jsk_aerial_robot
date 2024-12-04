#!/bin/bash

set -e

# Get VERSION_CODENAME from /etc/os-release
DISTRO=$(grep '^VERSION_CODENAME=' /etc/os-release | cut -d'=' -f2)

# Conditional branching based on distro_codename
if [ "$DISTRO" = "jammy" ]; then
    echo "This is Ubuntu 22.04 (jammy). Install ROS-O"

    sudo apt install -y python3-pip
    pip3 install catkin-tools
    echo "deb [trusted=yes] https://raw.githubusercontent.com/sugikazu75/ros-o-builder/$DISTRO-one-unstable/repository ./" | sudo tee /etc/apt/sources.list.d/sugikazu75_ros-o-builder.list
    sudo apt update
    sudo apt install -y python3-rosdep2
    echo "yaml https://raw.githubusercontent.com/sugikazu75/ros-o-builder/$DISTRO-one-unstable/repository/local.yaml debian" | sudo tee /etc/ros/rosdep/sources.list.d/1-sugikazu75_ros-o-builder.list
    rosdep update
    sudo apt install -y ros-one-desktop-full
    sudo apt install -y python3-wstool
    sudo pip3 install -U catkin_tools
fi
