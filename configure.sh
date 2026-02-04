#!/bin/bash

set -e

# Get VERSION_CODENAME from /etc/os-release
DISTRO=$(grep '^VERSION_CODENAME=' /etc/os-release | cut -d'=' -f2)

# Conditional branching based on distro_codename
if [ "$DISTRO" = "jammy" ] || [ "$DISTRO" == "noble" ]; then
    echo "This is Ubuntu ($DISTRO). Install ROS-O"

    # Configure ROS One apt repository
    sudo apt install -y curl
    sudo curl -sSL https://ros.packages.techfak.net/gpg.key -o /etc/apt/keyrings/ros-one-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs)-testing main" | sudo tee /etc/apt/sources.list.d/ros1.list
    echo "# deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-one-keyring.gpg] https://ros.packages.techfak.net $(lsb_release -cs)-testing main-dbg" | sudo tee -a /etc/apt/sources.list.d/ros1.list

    # Install and setup rosdep
    # Do not install python3-rosdep2, which is an outdated version of rosdep shipped via the Ubuntu repositories (instead of ROS)!
    sudo apt update
    sudo apt install -y python3-rosdep
    sudo rosdep init

    # Define custom rosdep package mapping
    echo "yaml https://ros.packages.techfak.net/ros-one.yaml one" | sudo tee /etc/ros/rosdep/sources.list.d/1-ros-one.list
    rosdep update

    # Install packages, e.g. ROS desktop
    sudo apt install -y ros-one-desktop


    # Install other packages
    sudo apt install -y python3-catkin-tools python3-vcstool python-is-python3
    sudo apt install -y xvfb
fi
