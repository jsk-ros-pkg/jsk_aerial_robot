[![Build Status](https://travis-ci.com/tongtybj/aerial_robot.svg?branch=devel)](https://travis-ci.com/tongtybj/aerial_robot)

# This is for aerial robot, especially for transformable aerial robot as shwon in following figure.

![uav_intro](images/multilink-all.jpg)

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -u -t src aerial_robot http://github.com/tongtybj/aerial_robot --git
wstool merge -t src src/aerial_robot/aerial_robot.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```

## how to use
Please check instruction in [wiki](https://github.com/tongtybj/aerial_robot/wiki).


