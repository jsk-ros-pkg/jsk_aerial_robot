[![.github/workflows/ros_test.yml](https://github.com/jsk-ros-pkg/jsk_aerial_robot/actions/workflows/ros_test.yml/badge.svg)](https://github.com/jsk-ros-pkg/jsk_aerial_robot/actions/workflows/ros_test.yml)

# This is for aerial robot, especially for transformable aerial robot as shown in following figure.

![uav_intro](images/multilink-all.jpg)

## Setup

### Ubuntu 20.04

#### Install ROS1 from official site
- Noetic: https://wiki.ros.org/noetic/Installation/Ubuntu

#### Build

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash # please replace ${ROS_DISTRO} with your specific env variable, e.g., noetic
sudo apt install -y python3-wstool python3-catkin-tools
mkdir -p ~/ros/jsk_aerial_robot_ws/src
cd ~/ros/jsk_aerial_robot_ws
sudo rosdep init
rosdep update --include-eol-distros
wstool init src
wstool set -u -t src jsk_aerial_robot http://github.com/jsk-ros-pkg/jsk_aerial_robot --git
wstool merge -t src src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

### Ubuntu 22.04, 24.04 (ROS-O)

```bash
sudo apt install python3-vcstool python3-catkin-tools
mkdir -p ~/ros/jsk_aerial_robot_ws/src
cd ~/ros/jsk_aerial_robot_ws/src
git clone https://github.com/jsk-ros-pkg/jsk_aerial_robot.git
./jsk_aerial_robot/configure.sh # for configuration especially for ros-o
source /opt/ros/one/setup.bash
cd ~/ros/jsk_aerial_robot_ws
vcs import src < src/jsk_aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall --recursive
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## Demo
Please check instruction in [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki).
