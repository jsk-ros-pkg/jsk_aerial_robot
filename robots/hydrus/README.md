# HYDRUS: 

definiation: Transformable multirotor with two-dimensional multilinks

paper: https://www.tandfonline.com/doi/full/10.1080/01691864.2016.1181006

# how to use:

**note**: the following instructions are based on hex type robot, we also have quad type robot (chage the var ```type```)

## 1. robot bringup command:

### real machine
-  egomotion estimation only with on-board sensors :
```
$ roslaunch hydrus hydrusx_bringup.launch control_mode:=0 estimate_mode:=0 type:=hex
```
-  egomotion estimation with mocap:
```
$ roslaunch hydrus hydrusx_bringup.launch control_mode:=0 estimate_mode:=1 type:=hex
```

### simulation
```
$ roslaunch hydrus hydrusx_bringup.launch real_machine:=false simulation:=true headless:=false control_mode:=0 type:=hex
```

## 2. teleop 
   **note**: after the robot completely transforms to normal form

   - **keyboard**: please check [instruction](https://github.com/tongtybj/aerial_robot/wiki/keyboard_operation)
   - **joystick**: please check [instruction](https://github.com/tongtybj/aerial_robot/wiki/joystick_operation)
   
## 3. transformation demostration
   **note**: after the robot completely hovering with the message `Hovering!`
   
   ``` $  rosrun hydrus hydrusx_demo.py _link_num:=6 _duration:=8 _joint_control_topic_name:=/hydrusx/joints_ctrl ```
