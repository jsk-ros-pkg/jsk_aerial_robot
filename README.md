[![Build Status](https://travis-ci.com/jsk-ros-pkg/jsk_aerial_robot.svg?branch=devel)](https://travis-ci.com/jsk-ros-pkg/jsk_aerial_robot)

# Our repository handles the control of aerial robots - especially for transformable aerial robots as DRAGON, HALO, and HYDRUS

![uav_intro](images/multilink-all.jpg)

## Setup

### Ubuntu 18.04, 20.04

#### Install ROS from official site depending on your Ubuntu version
- Melodic (for Ubuntu 18.04): https://wiki.ros.org/melodic/Installation/Ubuntu
- Noetic (for Ubuntu 20.04): https://wiki.ros.org/noetic/Installation/Ubuntu

#### Build your workspace

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
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

#### Install ROS from thirdparty website
- ROS-O: https://ros.packages.techfak.net/

#### Build your workspace

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
Please check instructions in [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki).
