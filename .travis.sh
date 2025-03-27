#!/bin/bash

set -ex

apt-get update -qq && apt-get install -y -q wget sudo lsb-release gnupg git sed build-essential ca-certificates # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"

# Install ROS
if [[ "$ROS_DISTRO" ==  "one" ]]; then
    ${CI_SOURCE_PATH}/configure.sh
else
    sudo sh -c "echo \"deb ${REPOSITORY} `lsb_release -cs` main\" > /etc/apt/sources.list.d/ros-latest.list"
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update -qq

    if [[ "$ROS_DISTRO" ==  "noetic" ]]; then
        sudo apt-get install -y -q python3-catkin-pkg python3-catkin-tools python3-rosdep python3-wstool python3-rosinstall-generator python3-osrf-pycommon python-is-python3
    else
        sudo apt-get install -y -q python-catkin-pkg python-catkin-tools python-rosdep python-wstool python-rosinstall-generator
    fi
    sudo apt-get install -y -q ros-$ROS_DISTRO-catkin

    # Setup for rosdep
    sudo rosdep init
    rosdep update --include-eol-distros
fi


source /opt/ros/${ROS_DISTRO}/setup.bash

# Install source code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
cp -r ${CI_SOURCE_PATH} src/${REPOSITORY_NAME} # copy the whole contents instead of create symbolic link
wstool init src
wstool merge -t src src/${REPOSITORY_NAME}/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible

# Build
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build --no-status
catkin build --catkin-make-args run_tests -- -i --no-deps --no-status -p 1 -j 1 aerial_robot
catkin_test_results --verbose build || catkin_test_results --all build
