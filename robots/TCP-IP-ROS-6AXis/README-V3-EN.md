---

typora-root-url: ./image
---

# <center>ROS-Robot</center>

Chinese version of the README -> please [click here](./README-V3.md)

Dobot   TCP-IP-ROS-6AXis   secondary development API interface ([TCP-IP-Python-V3 Public  README](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git))

# 1\. Introduction

**TCP-IP-ROS-6AXis** is a software development kit designed by Dobot based on the ROS of TCP/IP protocol. It is developed based on ROS/C++ language, follows the Dobot-TCP-IP control communication protocol, connects to the device terminal via socket, and provides users with an easy-to-use API interface. Through TCP-IP-ROS-6AXis, users can quickly connect to the Dobot device and carry out secondary development to control and use the device.

## Pre-dependency

* You can connect your computer to the network interface of the controller with a network cable, and then set the fixed IP to be in the same network segment as the controller IP. You can also connect your computer to the controller via wireless connection.
  
  When connected to LAN1 via wired connection: IP: 192.168.5.1; When connected to LAN2 via wired connection: IP: 192.168.100.1; Wireless connection: IP: 192.168.1.6

* Try pinging the controller IP to make sure it is under the same network segment.

* This API interface and Demo are suitable for Ubuntu-ROS1 environment. Here is the ROS1 version corresponding to the Ubuntu version:
  
  ** Install ROS1 on Ubuntu 16.04
  
  ​     [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
  
  ** Install ROS1 on Ubuntu 18.04
  
  ​     [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
  
  ** Install ROS1 on Ubuntu 20.04
  
  ​     [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

* This API interface and Demo are applicable to V3 series controller V3.5.5.0 and above.

## Version and Release

### Current version V1.0.0.0

| Version| Date|
|:----------:|:----------:|
| V1.0.0.0| 2024-03-06|

# 2\. Technical support


# 3\. CR_ROS control protocol

As the communication based on TCP/IP has high reliability, strong practicability and high performance with low cost, many industrial automation projects have a wide demand for controlling robots based on TCP/IP protocol. Therefore, the CR-V3 robot is designed to provide rich interfaces for interaction with external devices based on the TCP/IP protocol. For more details, see [Dobot TCP_IP Remote Control Interface Guide](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis.git).

# 4\. Obtaining CR_ROS

1. Obtain the secondary development API program of Dobot  TCP-IP-ROS-6AXis from GitHub.
  
   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. Refer to the corresponding README-V3.m file for use.

# 5\. File and class descriptions

| File| Description|
|----------|----------|
| dobot_bringup| Communication node of V3 robot, responsible for communication with the robot.|
| dobot_v4_bringup| Communication node of V4 robot, responsible for communication with the robot.|
| dobot_gazebo| Robot-simulation related|
| rviz_dobot_control| Rviz related|
| dobot_description| URDF model|
| dobot_moveit| Active moveit for different models|
| cr3_moveit| CR3 moveit|
| cr5_moveit| CR5 moveit|
| cr10_moveit| CR10 moveit|
| cr16_moveit| CR10 moveit|
| me6_moveit| E6 moveit|
| nova2_moveit| Nova2 moveit|
| nova5_moveit| Nova5 moveit|
| rosdemo_v3| V3 robot running demo|
| README-V3-EN| V3 Readme |
| README-V4-EN| V4 Readme |
| | |

# 6\. Source Code Compilation

### Download source code

```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git

cd $HOME/catkin_ws
```

### Compile

```
catkin_make
```

### Set environment variable

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

### If it is a CR3 robot, please type the following commands

```
echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
source ~/.bashrc
```

### If it is a CR5 robot, please type the following commands

```
echo "export DOBOT_TYPE=cr5" >> ~/.bashrc
source ~/.bashrc
```

### If it is a CR10 robot, please type the following commands

```
echo "export DOBOT_TYPE=cr10" >> ~/.bashrc
source ~/.bashrc
```

### If it is a CR16 robot, please type the following commands

```
echo "export DOBOT_TYPE=cr16" >> ~/.bashrc
source ~/.bashrc
```

# 7\. Demonstration

## Use in a simulation environment

## rviz display

```
roslaunch dobot_description display.launch
```

You can adjust the angle of each joint via joint_state_publisher_gui, and see the result on rviz.

![rviz display](/rviz.jpg)

## moveit control

* Start moveit with the following command

```
roslaunch dobot_moveit demo.launch
```

* Drag the joint to any angle and click "Plan and Execute" to see the result.

![moveit display](/moveit.gif)

**If the operating environment is Ubuntu16-ros-Kinetic or below, replace the demo.launch file**

```
<?xml version="1.0"?>
<!--
  Usage:
    roslaunch cr5_bringup cr5_bringup.launch robot_ip:=<value>
-->
<launch>
  <arg name="DOBOT_TYPE" default="$(env DOBOT_TYPE)" />

  <!-- export DOBOT_MOVEIT_PARENT_PATH=$(dirname $(rospack find dobot_moveit)) -->
  <arg name="Dobot_parent_path" value="$(env DOBOT_MOVEIT_PARENT_PATH)" />
  <arg name="Dobot_path" default="$(env DOBOT_TYPE)_moveit" />
  <arg name="Target_path" value="$(arg Dobot_parent_path)/$(arg Dobot_path)/launch" />
  <include file="$(arg Target_path)/demo.launch"/>
</launch>

```

* Then execute the following commands

```
source  ~/.bashrc 

source $HOME/catkin_ws/devel/setup.bash

export DOBOT_MOVEIT_PARENT_PATH=$(dirname $(rospack find dobot_moveit))

roslaunch dobot_moveit demo.launch
```

* If error occurs, please check the above commands.

## gazebo simulation

* Start gazebo with the following command

```
roslaunch dobot_gazebo gazebo.launch 
```

* You can also use “MoveIt!” to control the gazebo robots
* Set “MoveIt!” node that allows motion planning to run

```
roslaunch dobot_moveit moveit.launch
```

* Drag the joint to any angle and click "Plan and Execute" to see the result.

Exception handling methods:

![joint_controller](/joint_controller.png)

1. Install the ros-control  ros-controller

```
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
```

2. Node start order:    Start **gazebo** first, then **moveit**.

## Controlling the real robot

* **Use the following command to connect to the robot, robot_ip is the IP address of the actual robot.**
  
  ```
  roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1
  ```

* **Start Moveit with the following command**
  
  ```
  roslaunch dobot_moveit moveit.launch
  ```

**If the operating environment is Ubuntu16-ros-Kinetic or below, replace the moveit.launch file**

```
<?xml version="1.0"?>
<!--
  Usage:
    roslaunch cr5_bringup cr5_bringup.launch robot_ip:=<value>
-->
<launch>
  <arg name="DOBOT_TYPE" default="$(env DOBOT_TYPE)" />

  <!-- export DOBOT_MOVEIT_PARENT_PATH=$(dirname $(rospack find dobot_moveit)) -->
  <arg name="Dobot_parent_path" value="$(env DOBOT_MOVEIT_PARENT_PATH)" />
  <arg name="Dobot_path" default="$(env DOBOT_TYPE)_moveit" />
  <arg name="Target_path" value="$(arg Dobot_parent_path)/$(arg Dobot_path)/launch" />
  <include file="$(arg Target_path)/$(arg Dobot_path).launch"/>
</launch>

```

* Then execute the following commands

```
source  ~/.bashrc 

source $HOME/catkin_ws/devel/setup.bash

export DOBOT_MOVEIT_PARENT_PATH=$(dirname $(rospack find dobot_moveit))

roslaunch dobot_moveit moveit.launch
```

* If error occurs, please check the above commands.

* **Modify the contents of the robot_type.json file in the rviz_dobot_control directory, and modify the robotType field to V3 (V3 by default)**

```
{
    "robotType": "v3"
}
```

![](/robotType.png)

* **Add the DobotControl plugin to rviz to enable the robot**
  
  1. Click **Panels --> Add New Panel**.
  2. Select **DobotControl** and click **OK**.
  3. Click **EnableRobot** to enable the robot.
  4. When **Enable** and **Connected** are displayed on the status area, it means the robot is connected and enabled, and you can control the robot via Moveit.
  
  ![DobotControl](/cr5control.jpg)

# Custom function development

    The dobot_bringup defines msgs and srvs, and you can use these underlying msgs and srvs to control the robot.

# 8\. Common problem

**Problem 1:**  TCP connection. Port 29999/30003 cannot be connected or cannot deliver commands after connecting.

**Solution:**  If the controller version is below V3.5.6.0, you can try to upgrade the controller to V3.5.6.1 or above. If the controller version is already V3.5.6.1 or above, you can feedback the problem to technical support.

**Problem 2:** During TCP connection, the 29999 control port can send commands, but the 30003 motion port cannot send commands.

**Solution:**  If motion queue is blocked, you can try to reopen the queue by delivering **clearerror()** and **continue()** commands via port 29999.  The continue() command is supported by V3.5.7.0 and above version.

**Problem 3:** How to judge whether the Robot motion command is in place or not

**Solution:**  It can be judged by delivering a sync command.

​                   It can be judged by comparing the Cartesian coordinates of the target point with the actual Cartesian coordinates of the robot.

​                   It can be judged by comparing the joint coordinates of the target point with the actual joint coordinates of the robot.

# 9\. Example

* Dobot-Demo realizes TCP control of the robot and other interactions. It connects to the control port, motion port, and feedback port of the robot respectively. It delivers motion commands to robot, and handles the abnormal status of the robot, etc.

1. Main thread: Connect to the control port, motion port, and feedback port of the robot respectively. Enable the robot.

![](/main_en.png)

2. Feedback status thread: Real-time feedback of robot status information.

![](/feed_en.png)

3. Robot motion thread: Deliver motion commands to robot

![](/move_en.png)

4. Exception handling thread: Judge and handle the abnormal status of the robot

![](/excetion_en.png)

**Steps to run the Demo:**

1. Obtain the secondary development API program of Dobot TCP-IP-ROS-6AXis from GitHub.
  
   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. Connect to the robot via LAN1 interface, and set the local IP address to 192.168.5.X
  
   Control Panel >> Network and Internet >> Network Connections
   
   ![](/netConnect_en.png)
   
   Select the connected Ethernet >> Right click >> Properties >> Internet Protocol Version (TCP/IPV4)
   
   Modify the IP address to 192.168.5.X
   
   <img src="/updateIP_en.png" style="zoom:80%;" />

3. Open the DobotStudio Pro, connect to the robot, and set the robot mode to **TCP/IP secondary development**.
  
   ![](/checkTcpMode_en.png)
   
   ```
   Download source code
   cd $HOME/catkin_ws/src
   git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   cd $HOME/catkin_ws
   
   Compile
   catkin_make
   
   Set environment variable
   source $HOME/catkin_ws/devel/setup.bash
   
   Import model
   echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
   source ~/.bashrc
   
   ```
   
   ![](/bringup.png)

```
Start node
roslaunch dobot_bringup bringup.launch
 
New terminal
cd $HOME/catkin_ws/src
source ~/.bashrc
source $HOME/catkin_ws/devel/setup.bash

Start demo
roslaunch rosdemo_v3  democr_v3.launch
```

![](/demov3.png)

**Make sure the robot is in a safe position before running the Demo to prevent unnecessary collisions.**
