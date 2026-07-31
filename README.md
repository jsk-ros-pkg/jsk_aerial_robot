[![.github/workflows/ros_test.yml](https://github.com/jsk-ros-pkg/jsk_aerial_robot/actions/workflows/ros_test.yml/badge.svg)](https://github.com/jsk-ros-pkg/jsk_aerial_robot/actions/workflows/ros_test.yml)

# This is for aerial robot, especially for transformable aerial robot as shown in following figure.

![uav_intro](images/multilink-all.jpg)

## Setup

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
