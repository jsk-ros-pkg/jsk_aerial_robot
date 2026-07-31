#!/bin/bash

set -ex

apt-get update -qq && apt-get install -y -q curl wget sudo lsb-release gnupg git sed build-essential ca-certificates # for docker
echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections

echo "Testing branch $TRAVIS_BRANCH of $REPOSITORY_NAME"

# Install ROS
${CI_SOURCE_PATH}/configure.sh

source /opt/ros/${ROS_DISTRO}/setup.bash

# Install source code
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
cp -r ${CI_SOURCE_PATH} src/${REPOSITORY_NAME} # copy the whole contents instead of create symbolic link

if [[ "$ROS_DISTRO" ==  "one" ]]; then
    vcs import src < src/${REPOSITORY_NAME}/aerial_robot_${ROS_DISTRO}.rosinstall --recursive
else
    wstool init src
    wstool merge -t src src/${REPOSITORY_NAME}/aerial_robot_${ROS_DISTRO}.rosinstall
    wstool update -t src
fi
rosdep install --from-paths src -y -q -r --ignore-src --rosdistro ${ROS_DISTRO} # -r is indisapensible

# Build
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
catkin build --no-status
catkin build --catkin-make-args run_tests -- -i --no-deps --no-status -p 1 -j 1 aerial_robot
catkin_test_results --verbose build || catkin_test_results --all build
