# This is for aerial robot.

## how to compile

```
cd <catkin_ws>
wstool init src
wstool set -t src jsk_mbzirc http://github.com/tongtybj/aerial_robot --git
wstool merge -t src https://raw.githubusercontent.com/tongtybj/aerial_robot/new_structure/aerial_robot.rosinstall?token=ADfwr6xu8nvS--HSJjRgkFD2UgwRbyBbks5Y66iOwA%3D%3D
wstool update -t src
rosdep install -y -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO
catkin build
```



