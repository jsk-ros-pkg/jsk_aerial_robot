[![Build Status](https://travis-ci.com/tongtybj/aerial_robot.svg?branch=devel)](https://travis-ci.com/tongtybj/aerial_robot)

# This is for aerial robot.

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -t src aerial_robot http://github.com/tongtybj/aerial_robot --git
wstool merge -t src https://raw.githubusercontent.com/tongtybj/aerial_robot/devel/aerial_robot.rosinstall?token=AGwEzdzKIUPJe3NHheO_xyAnnJkMPKvZks5avHp9wA%3D%3D
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```



