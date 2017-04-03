[![Build Status](https://travis-ci.org/tongtybj/aerial_robot.svg?branch=devel)](https://travis-ci.org/tongtybj/aerial_robot)

# This is for aerial robot.

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -t src aerial_robot http://github.com/tongtybj/aerial_robot --git
wstool merge -t src https://raw.githubusercontent.com/tongtybj/aerial_robot/new_structure/aerial_robot.rosinstall
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```



