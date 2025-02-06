[![Build Status](https://travis-ci.com/jsk-ros-pkg/jsk_aerial_robot.svg?branch=devel)](https://travis-ci.com/jsk-ros-pkg/jsk_aerial_robot)

# This is for aerial robot, especially for transformable aerial robot as shown in following figure.

![uav_intro](images/multilink-all.jpg)

## Setup

### Ubuntu 18.04, 20.04

#### Install ROS1 from official site
- Melodic: https://wiki.ros.org/melodic/Installation/Ubuntu
- Noetic: https://wiki.ros.org/noetic/Installation/Ubuntu

#### Build

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash # please replace ${ROS_DISTRO} with your specific env variable, e.g., melodic
sudo apt install -y python3-wstool python3-catkin-tools # for melodic, please replace `python3` with `python`
mkdir -p ~/ros/jsk_aerial_robot_ws/src
cd ~/ros/jsk_aerial_robot_ws
sudo rosdep init
rosdep update
wstool init src
wstool set -u -t src jsk_aerial_robot http://github.com/jsk-ros-pkg/jsk_aerial_robot --git
./src/jsk_aerial_robot/configure.sh # for configuration especially for ros-o in jammy
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

### Ubuntu 22.04 (ROS-O)

```bash
sudo apt install -y python3-wstool
mkdir -p ~/ros/jsk_aerial_robot_ws/src
cd ~/ros/jsk_aerial_robot_ws
wstool init src
wstool set -u -t src jsk_aerial_robot http://github.com/jsk-ros-pkg/jsk_aerial_robot --git
./src/jsk_aerial_robot/configure.sh # for configuration especially for ros-o in jammy
source /opt/ros/one/setup.bash
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## Demo
Please check instruction in [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki).
