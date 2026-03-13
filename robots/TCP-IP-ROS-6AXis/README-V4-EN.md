---

typora-root-url: ./image
---

# <center>ROS-Robot</center>

Chinese version of the README -> please [click here](./README-V4.md)

Dobot   TCP-IP-ROS-6AXis   secondary development API interface ([TCP-IP-ROS-6AXis Public README](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git))

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

* This API interface and Demo are applicable to V4 series controller V440 and above.

## Version and Release

### Current version V1.0.0.0

| Version| Date|
|:----------:|:----------:|
| V1.0.0.0 | 2024-03-06|

# 2\. Technical support



# 3\. CR_ROS control protocol

As the communication based on TCP/IP has high reliability, strong practicability and high performance with low cost, many industrial automation projects have a wide demand for controlling robots based on TCP/IP protocol. Dobot robots, designed on the basis of TCP/IP protocol, provide rich interfaces for interaction with external devices.

Robot status:

    1. Error status is the first priority. If the robot reports an error and is not enabled, it returns the error status.
    
    2. Power-off status is the second priority. If the robot is not powered on and not enabled, it returns the power-off status.
    
    3. Collision status is the third priority. If the robot is in a collision and the script is paused, it returns the collision status.
    
    4. Brake-ON status is the fourth priority. If the brake of the robot is switched on but the robot is not enabled, it returns the brake-ON status.
    
    The other statuses are fed back according to the actual situation.

Port feedback:

    Port 29999 is responsible for receiving the commands related to settings and motion control by sending and receiving.
    
    Port 30004 (hereinafter referred to as the real-time feedback port) feeds back robot information every 8ms.
    
    Port 30005 feeds back robot information every 200ms.
    
    Port 30006 is a configurable port to feed back robot information (feed back every 50ms by default).
    
    Cancel port 30003.

For more details, see [**Dobot TCP_IP Remote Control Interface Guide**](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4.git).

# 4\. Obtaining TCP-IP-ROS-6AXis

1. Obtain the secondary development API program of Dobot  TCP-IP-ROS-6AXis from GitHub.
  
   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. Refer to the corresponding README-V4.md file for use.

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
| README-V3-EN | V3 Readme|
| README-V4-EN | V4 Readme|
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
source $HOME/catkin_ws/devel/setup.bash
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
  roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1
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

* **Modify the contents of the robot_type.json file in the rviz_dobot_control directory, and modify the robotType field to V4 (V3 by default)**

```
{
    "robotType": "v4"
}
```

![](/robotType.png)

* **Add the DobotControl plugin to rviz to enable the robot**
  
  1. Click **Panels --> Add New Panel**.
  2. Select **DobotControl** and click **OK**.
  3. Click **EnableRobot** to enable the robot.
  4. When **Enable** and **Connected** are displayed on the status area, it means the robot is connected and enabled, and you can control the robot via Moveit.
  
  ![DobotControl](/cr5control.jpg)

## Controlling multiple real robots

* **Modify the bringup_v4.launch file in the dobot_v4_bringup/launch directory, find the field <param name="num_nodes" type="int" value="1" />, and modify the value of num_nodes to greater than 1.**
  
  ```
  eg:
  <arg name="robotIp" doc="IP of the controller" default="192.168.5.1"/>
    <arg name="robotName" doc="Name of the controller" default=""/>
  
    <node name="$(arg robotName)$(env DOBOT_TYPE)_robot" pkg="dobot_v4_bringup" type="dobot_v4_bringup" output="screen" >
      <param name="num_nodes" type="int" value="2" />
      <param name="robot_node_name" type="str" value="$(arg robotName)"/>
      <param name="joint_publish_rate" type="double" value="10" />
      <param name="num_nodes" type="double" value="0.1" />
      <param name="robot_ip_address" type="str" value="$(arg robotIp)"/>
    </node>
  ```

* **Use the following command to connect to the robot A, robot_ip is the IP address of the actual robot.**
  
  ```
  roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.100.10 robotName:=robotA
  ```

* **Use the following command to connect to the robot B, robot_ip is the IP address of the actual robot.**
  
  ```
  roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.100.20 robotName:=robotB
  ```

# Custom function development

    The dobot_bringup defines msgs and srvs, and you can use these underlying msgs and srvs to control the robot.

# 7\. Common problem

* TCP connection:
  
  Port 29999 can be connected by one client at a time;
  
  Ports 30004 30005 30006 can be connected by multiple clients at the same time;
  
  Port 29999 has mode restriction. Before opening, you need to set the robot to TCP mode, otherwise it cannot respond and return “Control Mode Is Not Tcp” after the command is delivered. There is no mode restriction for real-time feedback ports 30004, 30005, 30006.

* To obtain the robot status, you can monitor the return value of RobotMode().
  
  1. When the controller is in initializing status, it returns 1;
  
  2. When the robot status is not 0 or 1, the robot initialization is completed;
  
  3. When the robot is in power-off status, it returns 3;
  
  4. When the robot is initialized but not enabled, it returns 4;
  
  5. When the robot is enabled but idle, it returns 5, indicating that it can receive motion commands normally;
  
  6. movj, script, and other queue commands are running status and return 7;
  
  7. movJog is a single motion status, it return 8; no power-on status.
    
     | Mode| Description| Note|
     |----------|----------|----------|
     | 1| ROBOT_MODE_INIT| Initialized status|
     | 2| ROBOT_MODE_BRAKE_OPEN| Brake switched on|
     | 3| ROBOT_MODE_POWEROFF| Power-off status|
     | 4| ROBOT_MODE_DISABLED| Disabled (no brake switched on)|
     | 5| ROBOT_MODE_ENABLE| Enabled (idle)|
     | 6| ROBOT_MODE_BACKDRIVE| Drag|
     | 7| ROBOT_MODE_RUNNING| Running status (script, TCP queue)|
     | 8| ROBOT_MODE_SINGLE_MOVE| Single motion status (jog)|
     | 9| ROBOT_MODE_ERROR| Error status|
     | 10| ROBOT_MODE_PAUSE| Pause status|
     | 11| ROBOT_MODE_COLLISION| Collision status|


* Commands for different status: 
  
  *Commands can be executed for error status: ClearError(), GetErrorID(), EmergeStop(), RobotMode(), the rest are rejected commands and return -2;
  
  *Commands can be executed for emergency stop status: ClearError(), GetErrorID(), EmergeStop(), RobotMode(), the rest are rejected commands and return -3;
  
  *Commands can be executed for power-off status: ClearError(), GetErrorID(), EmergeStop(), RobotMode(), PowerOn(), the rest are rejected commands and return -4;
  
  *TCP commands are all immediate return commands except EnableRobot() and PowerOn(). The return of the command only represents successful delivering, not completion of execution.

* Coordinate system:
  
  After entering TCP/IP mode, the system sets the user/tool coordinate system to 0 by default. It automatically restores the value of the user/tool coordinate system after exiting TCP/IP mode.
  
  ​      Global coordinates: the coordinates set by User() and Tool() commands are the global coordinates, which takes effect for all commands after setting;
  
  ​      Local coordinates: the optional parameters (user/tool) in the motion command are only effective in the current motion command, and will be restored to the global coordinates after the execution of the current command.

* The algorithm allows a queue depth of 64, 64 queued commands can be processed continuously at the same time. If the queued commands sent to the port exceed the queue depth, the port will not be disconnected, and the command interface will return -1 and the command will not be executed.

* The queue command is an immediate return command. The successful return of the interface only means that it was delivered successfully, it does not mean that the execution is completed. If it is judged that the execution is completed, it needs to be combined with CommandID and RobotMode to make a comprehensive judgment.

* The servo run time represents the accumulated time after the robot is enabled, and the controller power-on time represents the time after the controller is powered on, so the controller power-on time is always greater than the servo run time.

* The collision status is not an error, there is no collision error returned in GetErrorID(). You can query the collision status by RobotMode(), which returns 11. You can clear the collision status by ClearError().

* After a collision, you can deliver an error clearing command to clear the collision status, and the robot can be run again without re-enabling.

* After switching to TCP/IP mode, the speed/acceleration/user/tool coordinate system parameters will be set to the default values, so there is no need to call the EnableRobot() command to set the default parameters.

* The return value of the TCP queue command only indicates whether the parameters and syntax of the command conform to the specification, it does not indicate whether the command is successfully executed or not. The TCP queue command returns when it is delivered, and the return of 0 only means that it is delivered successfully, not that it can be executed successfully.

* If an error occurs in parameter type/number of the TCP command , the system will alarm directly and will not give a 0 to handle it, and the command will not be delivered to the algorithm.

* After triggering the emergency stop signal, the robot stops running by default. If it does not stop after 500ms, it will be powered off. If it stops, it will not be powered off. In general, the robot will not be powered off after an emergency stop.

* After setting the default gateway, the default gateway data will be saved and the value will not be changed after a restart.

* The Enable command only executes the enable action without other parameter settings. It will not clear the kinematic parameters and coordinate system parameters.

* The Pause() and Continue() commands are effective for script, and the motion commands (queue-related) are also effective. The robot enters the pause status and the algorithmic queue pauses after calling the Pause() command. You can continue to run the queue commands using the Continue() command. The MovJog command cannot be paused or continued.

* The current TCP does not support the insertion of extraneous characters between commands. You can write in the following formats 
  
  ① MovJ()MovJ()MovJ()
  
  ② MovJ() MovJ() MovJ()

# 8\. Example

* Dobot-Demo realizes TCP control of the robot and other interactions. It connects to the control port and feedback port of the robot respectively. It delivers motion commands to robot, and handles the abnormal status of the robot, etc.

1. Main thread: Connect to the control port, motion port, and feedback port of the robot respectively. Enable the robot.

![](/main_en.png)

2. Feedback status thread: Real-time feedback of robot status information.

![](/feed_en.png)

3. Robot motion thread: Deliver motion commands to robot

![](/move_en.png)

Arrival signal

The queue command is an immediate return command. The successful return of the interface only means that it was delivered successfully, it does not mean that the execution is completed. If it is judged that the execution is completed, it needs to be combined with

* If the current CommandID is greater than the CommandID that delivered, it means the delivered queue command is completed.

* If the current CommandID is equal to the CommandID that delivered, and RobotMode command returns a value of 5, it means the delivered queue command is completed.

4. Exception handling thread: Judge and handle the abnormal status of the robot

![](/excetion_en.png)

**Steps to run the Demo:**

1. Obtain the secondary development API program of Dobot TCP-IP-CR-Python-V4 from GitHub.
  
   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. Connect to the robot via LAN1 interface, and set the local IP address to 192.168.5.X
  
   Control Panel >> Network and Internet >> Network Connections
   
   ![](/netConnect_en.png)
   
   Select the connected Ethernet >> Right click >> Properties >> Internet Protocol Version (TCP/IPV4)
   
   Modify the IP address to 192.168.5.X
   
   ![](/updateIP_en.png)

3. Open the DobotStudio Pro, connect to the robot, and set the robot mode to **TCP/IP secondary development**.
  
   ![](/checkTcpMode_en.png)

#### rosdemo_v4

```python
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

Start node
roslaunch dobot_v4_bringup bringup_v4.launch
 
New terminal
cd $HOME/catkin_ws/src
source ~/.bashrc
source $HOME/catkin_ws/devel/setup.bash

Start demo
roslaunch rosdemo_v4  democr_v4.launch

```

Control robot movement: Deliver motion commands to control the robot to move

```python
 launch
    
    <launch>
   
    <!-- Start one node -->
    <node name="rosdemo_v4" pkg="rosdemo_v4" type="rosdemo_v4" output="screen">
        <!-- Add parameters for the node, if needed -->
        <!-- <param name="pointA" type="double" value="10"/>
        <param name="pointB" type="double" value="10" /> -->
       <rosparam param="pointA">[-90, 20, 0, 0, 0, 0]</rosparam>
       <rosparam param="pointB">[90, 20, 0, 0, 0, 0]</rosparam>
    </node>
</launch>

    
 Deliver motion commands via sersice  
 void RosDemoCRV4::movePoint(std::vector<double>& point, int& currentCommandID)
{
    if (point.size() < 6U) {
        ROS_ERROR("MovJ params is  ERROR");
        return;
    }

    rosdemo_v4::MovJ srvMovJ;
    srvMovJ.request.mode = 1;
    srvMovJ.request.a = point[0];
    srvMovJ.request.b = point[1];
    srvMovJ.request.c = point[2];
    srvMovJ.request.d = point[3];
    srvMovJ.request.e = point[4];
    srvMovJ.request.f = point[5];
    if (!SendService(m_movj, srvMovJ)) {
        ROS_ERROR("MovJ service fail");
    } else {
        currentCommandID = int(srvMovJ.response.res);
        ROS_INFO("CurrentCommandID  %d", currentCommandID);
    }
}
```

A sign that the command is completed: Wait for the robot to complete the motion command, which is similar to the Sync.

```python

Feedback of the currentCommandID: srvMovJ.response.res

void RosDemoCRV4::movePoint(std::vector<double>& point, int& currentCommandID)
{
    if (point.size() < 6U) {
        ROS_ERROR("MovJ params is  ERROR");
        return;
    }

    rosdemo_v4::MovJ srvMovJ;
    srvMovJ.request.mode = 1;
    srvMovJ.request.a = point[0];
    srvMovJ.request.b = point[1];
    srvMovJ.request.c = point[2];
    srvMovJ.request.d = point[3];
    srvMovJ.request.e = point[4];
    srvMovJ.request.f = point[5];
    if (!SendService(m_movj, srvMovJ)) {
        ROS_ERROR("MovJ service fail");
    } else {
        currentCommandID = int(srvMovJ.response.res);
        ROS_INFO("CurrentCommandID  %d", currentCommandID);
    }
}
```

Obtain information of the robot: Obtain robot status from the feedback port

```python
Obtain robot feedback via Topic

subFeedInfo = nh->subscribe("/dobot_v4_bringup/msg/FeedInfo", 10, &RosDemoCRV4::getFeedBackInfo, this);


Callback function, you can add the required robot information
void RosDemoCRV4::getFeedBackInfo(const std_msgs::String::ConstPtr& msg)
{
    std::string feedIndo = msg->data;
    nlohmann::json parsedJson = nlohmann::json::parse(feedIndo);
    std::unique_lock<std::mutex> lockInfo(m_mutex);
    if (parsedJson.count("EnableStatus") && parsedJson["EnableStatus"].is_number()) {
        feedbackData.EnableStatus = parsedJson["EnableStatus"];
    }
    if (parsedJson.count("ErrorStatus") && parsedJson["ErrorStatus"].is_number()) {
        feedbackData.ErrorStatus = parsedJson["ErrorStatus"];
    }
    if (parsedJson.count("RobotMode") && parsedJson["RobotMode"].is_number()) {
        feedbackData.RobotMode = parsedJson["RobotMode"];
    }
    if (parsedJson.count("CurrentCommandID") && parsedJson["CurrentCommandID"].is_number()) {
        feedbackData.CurrentCommandID = parsedJson["CurrentCommandID"];
    }
}
     ...  ...
```

Monitor the robot status: Monitor the abnormal status of the robot and error-clearing function

```python
Monitor the robot status thread:
void RosDemoCRV4::warmRobotError()
{
    while (true) {
        {
              ... ...
                // Request for service
                if (SendService(m_getErrorID, srvGetError)) {
              ... ...
                    for (int i = 0; i < errorId.size(); i++) {
                        bool alarmState{ false };
                        for (const auto& alarmControllerJson : m_JsonDataController) {
                            if (static_cast<int>(srvGetError.response.error_id[i]) ==
                                static_cast<int>(alarmControllerJson["id"])) {
                                ROS_ERROR("Control ErrorID : %d,  %s, %s", srvGetError.response.error_id[i],
                                          static_cast<std::string>(alarmControllerJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmControllerJson["en"]["description"]).c_str());
                                alarmState = true;
                                break;
                  ......
                        for (const auto& alarmServoJson : m_JsonDataServo) {
                            if (static_cast<int>(srvGetError.response.error_id[i]) ==
                                static_cast<int>(alarmServoJson["id"])) {
                                ROS_ERROR("Servo ErrorID : %d,  %s, %s", srvGetError.response.error_id[i],
                                          static_cast<std::string>(alarmServoJson["zh_CN"]["description"]).c_str(),
                                          static_cast<std::string>(alarmServoJson["en"]["description"]).c_str());
         ... ...
    }
}

                                
 Error-clearing command:
rosservice  call  /dobot_v4_bringup/srv/ClearError
```

**Make sure the robot is in a safe position before running the Demo to prevent unnecessary collisions.**
