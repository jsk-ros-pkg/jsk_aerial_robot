# DRAGON: 

**Definiation**: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation 

<img src="images/dragon_clip.jpg" width="410" height="300"> <a href="https://www.youtube.com/embed/ZDYU22qNI_Q" target="_blank"><img src="http://img.youtube.com/vi/ZDYU22qNI_Q/0.jpg"  alt="euroc" width="400" height="300" border="10" /></a>

**Related Papers**:
-  M. Zhao, T. Anzai, F. Shi, X. Chen, K. Okada and M. Inaba, "Design, Modeling, and Control of an Aerial Robot DRAGON: A Dual-Rotor-Embedded Multilink Robot With the Ability of Multi-Degree-of-Freedom Aerial Transformation," in IEEE Robotics and Automation Letters, vol. 3, no. 2, pp. 1176-1183, April 2018.
# how to use:

## 0. calibration: 
    Calibration is via rosserial between on-board processor and spinal. Please check the wiki.
## 1. robot bringup command:

   - **real machine**: ``` $ roslaunch dragon bringup.launch ```
         
     **note**: please check the servo angles before takeoff. use  `$ rostopic echo -c /servo/states` to check whether any angle exceeds the normal value (e.g. 5 -> 4095).

   - **simulation**: ``` $ roslaunch dragon bringup.launch real_machine:=false simulation:=true headless:=false ```

## 2. tele-operation
   **note**: after the robot completely transforms to normal form

   - **keyboard**: please check instruction [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki/keyboard_operation)
   - **joystick**: please check instruction [wiki](https://github.com/JSKAerialRobot/aerial_robot/wiki/joystick_operation)
   
## 3. transformation demostration
   **note**: after the robot completely hovering with the message `Hovering!`
   
   - defualt dragon pose: ``` $ rosrun dragon transformation_demo.py  _mode:=0```
   - spiral pose: ``` $ rosrun dragon transformation_demo.py  _mode:=1```
   - m-like pose: ``` $ rosrun dragon transformation_demo.py  _mode:=2```
   - normal pose: ``` $ rosrun dragon transformation_demo.py  _reset:=1```
   - reverse normal pose: ``` $ rosrun dragon transformation_demo.py  _reverse_reset:=1```
