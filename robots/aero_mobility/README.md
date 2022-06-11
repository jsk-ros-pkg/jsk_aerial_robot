# HYDRUS: 

**Definiation**: Transformable multirotor with two-dimensional multilinks

<p align="center">
<a href="https://www.youtube.com/embed/o_jsDUk0oFo" target="_blank"><img src="http://img.youtube.com/vi/o_jsDUk0oFo/0.jpg" alt="euroc" width="560 height="315" border="10" /></a>
</p>

**Related Papers**: 
- Moju Zhao, Koji Kawasaki, Kei Okada, Masayuki Inaba:
Transformable multirotor with two-dimensional multilinks: modeling, control, and motion planning for aerial transformation,
Advanced Robotics, Vol.30, No.13, pp.825--845, 2016.
- Tomoki Anzai, Moju Zhao, Xiangyu Chen, Fan Shi, Koji Kawasaki, Kei Okada, Masayuki Inaba:
Multilinked Multirotor with Internal Communication System for Multiple Objects Transportation based on Form Optimization Method,
in Proceedings of The 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems, pp.5977-5984, 2017.
- Tomoki Anzai, Moju Zhao, Shunichi Nozawa, Fan Shi, Kei Okada, Masayuki Inaba:
Aerial Grasping Based on Shape Adaptive Transformation by HALO: Horizontal Plane Transformable Aerial Robot with Closed-Loop Multilinks Structure,
in Proceedings of The 2018 IEEE International Conference on Robotics and Automation, pp.6990-6996, 2018.

# how to use:

**note**: the following instructions are based on hex type robot, we also have quad type robot (chage the var ```type```)

## 0. calibration: 
    Calibration is via rosserial between on-board processor and spinal. Please check the wiki.
## 1. robot bringup command:

### real machine
**note**: `${model}` can be `quad` or `hex`. 
-  egomotion estimation only with on-board sensors:
```
$ roslaunch hydrus bringup.launch control_mode:=0 estimate_mode:=0 type:=${model}
```
-  egomotion estimation with mocap:
```
$ roslaunch hydrus bringup.launch control_mode:=0 estimate_mode:=1 type:=${model}
```

### simulation
-  flight control in gazebo:
```
$ roslaunch hydrus bringup.launch real_machine:=false simulation:=true headless:=false control_mode:=0 type:=${model}
```
- only check kinematics
```
$ roslaunch hydrus bringup.launch real_machine:=false simulation:=false headless:=false type:=${model}
```
- egomotion estimation debug
```
$ roslaunch hydrus bringup.launch real_machine:=true simulation:=true headless:=false type:=${model}
```

## 2. tele-operation
   **note**: after the robot completely transforms to normal form

   - **keyboard**: please check instruction [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki/keyboard_operation)
   - **joystick**: please check instruction [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki/joystick_operation)
   
## 3. transformation demostration
   **note**: after the robot completely hovering with the message `Hovering!`
   
   - quad:  ``` $  rosrun hydrus hydrus_demo.py _link_num:=4 _duration:=6 _joint_control_topic_name:=/hydrus/joints_ctrl ```
