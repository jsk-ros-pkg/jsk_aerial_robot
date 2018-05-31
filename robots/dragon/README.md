# DRAGON: 

definiation: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation 

paper: https://ieeexplore.ieee.org/document/8258850/

# how to use:

## 1. robot bringup command:

   - **real machine**: ``` $ roslaunch dragon bringup.launch ```

   - **simulation**: ``` $ roslaunch dragon bringup.launch real_machine:=false simulation:=true headless:=false ```

## 2. teleop 
   **note**: after the robot completely transforms to normal form

   - **keyboard**: please check [instruction](https://github.com/tongtybj/aerial_robot/wiki/keyboard_operation)
   - **joystick**: please check [instruction](https://github.com/tongtybj/aerial_robot/wiki/joystick_operation)
   
## 3. transformation demostration
   **note**: after the robot completely hovering with the message `Hovering!`
   
   ``` $ rosrun dragon transformation_demo.py  ```
