#!/bin/bash

set -ex

apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg git sed # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

sudo sh -c "echo \"deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
sudo apt-get update -qq

## workaround for following git problem:
## "fatal: unknown value for config 'protocol.version': 2"
## https://github.com/juju/charm-tools/issues/532
git config --global --unset protocol.version

# Install ROS
if [[ "$ROS_DISTRO" ==  "noetic" ]]; then
    sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon
else
    sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
fi

sudo apt-get install -q -y  ros-${ROS_DISTRO}-catkin

source /opt/ros/${ROS_DISTRO}/setup.bash
sudo rosdep init
rosdep update
# script:
(cd ${CI_SOURCE_PATH}; git log --oneline | head -10)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
ln -sf ${CI_SOURCE_PATH} src/${REPOSITORY_NAME}
wstool init src
wstool merge -t src src/aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible

# Build
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build -p1 -j1 --no-status
catkin run_tests -p1 -j1 --no-status aerial_robot --no-deps
catkin_test_results --verbose build || catkin_test_results --all build
