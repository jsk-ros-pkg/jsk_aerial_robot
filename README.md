[![Build Status](https://travis-ci.com/JSKAerialRobot/aerial_robot.svg?branch=devel)](https://travis-ci.com/JSKAerialRobot/aerial_robot)

# This is for aerial robot, especially for transformable aerial robot as shown in following figure.

![uav_intro](images/multilink-all.jpg)

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -u -t src aerial_robot http://github.com/JSKAerialRobot/aerial_robot --git
wstool merge -t src src/aerial_robot/aerial_robot_${ROS_DISTRO}.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

#### Special Cases
- Jetson TX2: please check https://github.com/JSKAerialRobot/aerial_robot/wiki/Nvidia-Jetson


## how to use
Please check instruction in [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki).


