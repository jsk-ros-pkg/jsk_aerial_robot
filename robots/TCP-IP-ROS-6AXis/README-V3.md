---
typora-root-url: ./image
---

# <center>ROS-Robot</center>



English version of the README -> please [click here](./README-V3-EN.md)

Dobot  TCP-IP-ROS-6AXis   二次开发api接口 （ [TCP-IP-Python-V3 Public  README](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git) ）



# 1. 简介

TCP-IP-ROS-6AXis 是为 Dobot 公司旗下基于TCP/IP协议的ROS的封装设计的软件开发套件。它基于 ROS/C++ 语言开发，遵循Dobot-TCP-IP控制通信协议，通过socket与机器终端进行Tcp连接，  并为用户提供了易用的api接口。通过 TCP-IP-ROS-6AXis，用户可以快速地连接Dobot机器并进行二次开发对机器的控制与使用。



## 前置依赖

* 电脑可用网线连接控制器的网口，然后设置固定 IP，与控制器 IP 在同一网段下。也可无线连接控制器。

  有线连接时连接LAN1：ip为192.168.5.1, 有线连接时连接LAN2：ip为192.168.100.1,  无线连接：ip为192.168.1.6

* 尝试 ping 通控制器 IP，确保在同一网段下。

* 此API接口与Demo适用于Ubuntu-ROS1环境，以下是Ubuntu版本对应ROS1版本：

  ​    *在 Ubuntu 16.04 上安装 ROS1

  ​    [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

     *在 Ubuntu 18.04 上安装 ROS1

  ​    [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

     *Ubuntu 20.04 上安装 ROS1

  ​     [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

* 此API接口与Demo适用于V3系列的3.5.5.0及以上控制器版本

  


##  版本和发布记录

###  当前版本 v1.0.0.0

|   版本   |  修改日期  |
| :------: | :--------: |
| v1.0.0.0 | 2023-11-20 |


# 2.技术支持


# 3. CR_ROS 控制协议

由于基于TCP/IP的通讯具有成本低、可靠性高、实用性强、性能高等特点；许多工业自动化项目对支持TCP/IP协议控制机器人需求广泛，因此 CR-V3 机器人将设计在TCP/IP协议的基础上，提供了丰富的接口用于与外部设备的交互；有关协议更详细的信息请查阅**《[越疆TCPIP控制协议文档6AXis](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis.git)》**



# 4. 获取 CR_ROS

1. 从GitHub 下载或者克隆Dobot  TCP-IP-ROS-6AXis 二次开发api程序

   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. 参考对应的 README-V3.m 文档使用；

   

# 5. 文件类的说明以及使用方法



| 文件               | 功能说明                             |
| ------------------ | ------------------------------------ |
| dobot_bringup      | V3版本机器通信节点，负责与机器通信。 |
| dobot_v4_bringup   | V4版本机器通信节点，负责与机器通信。 |
| dobot_gazebo       | 机器仿真相关                         |
| rviz_dobot_control | Rviz相关                             |
| dobot_description  | URDF模型                             |
| dobot_moveit       | 统一启动不同机型的moveit             |
| cr3_moveit         | CR3机型moveit                        |
| cr5_moveit         | CR5机型moveit                        |
| cr10_moveit        | CR10机型moveit                       |
| cr16_moveit        | CR10机型moveit                       |
| me6_moveit         | E6机型moveit                         |
| nova2_moveit       | Nova2机型moveit                      |
| nova5_moveit       | Nova5机型moveit                      |
| rosdemo_v3         | V3版本机器运行示例-demo              |
| README-V3          | V3自述文件                           |
| README-V4          | V4自述文件                           |
| README-V3-EN       | V3自述文件-英文版                    |
| README-V4-EN       | V4自述文件-英文版                    |
|                    |                                      |



# 6. 源码编译

### 下载源码

```
cd $HOME/catkin_ws/src

git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git

cd $HOME/catkin_ws
```

### 编译

```
catkin_make
```

### 设置环境变量

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```



### 若为 CR3 机械臂，则使用如下命令设置机械臂类型

```
echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
source ~/.bashrc
```

### 若为 CR5 机械臂，则使用如下命令设置机械臂类型

```
echo "export DOBOT_TYPE=cr5" >> ~/.bashrc
source ~/.bashrc
```

### 若为 CR10 机械臂，则使用如下命令设置机械臂类型

```
echo "export DOBOT_TYPE=cr10" >> ~/.bashrc
source ~/.bashrc
```

### 若为 CR16 机械臂，则使用如下命令设置机械臂类型

```
echo "export DOBOT_TYPE=cr16" >> ~/.bashrc
source ~/.bashrc
```



# 7. 使用演示

## 在仿真环境下使用

## rviz 显示

```
roslaunch dobot_description display.launch
```

可通过 joint_state_publisher_gui 调节各关节的角度，在 rviz 上看到其显示效果

![rviz显示](/rviz.jpg)

## moveit 控制

* 使用如下命令启动 moveit

```
roslaunch dobot_moveit demo.launch
```

* 鼠标将关节拖到任意的角度，点击 "Plan and Execute" 即可看到运行效果

![moveit显示](/moveit.gif)



**如运行环境是Ubuntu16- ros-Kinetic及以下版本，替换demo.launch文件**

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

* 然后执行以下指令

```
source  ~/.bashrc 

source $HOME/catkin_ws/devel/setup.bash

export DOBOT_MOVEIT_PARENT_PATH=$(dirname $(rospack find dobot_moveit))

roslaunch dobot_moveit demo.launch
```

* 如遇报错，请检查上述指令是否执行漏执行或者执行错误。



## gazebo 仿真

* 使用如下命令启动 gazebo

```
roslaunch dobot_gazebo gazebo.launch 
```

* 同样，您可以使用MoveIt!控制gazebo里的仿真机器人
* 设置MoveIt!允许运动规划运行的节点,dobot类型需要对应 

```
roslaunch dobot_moveit moveit.launch
```

* 鼠标将关节拖到任意的角度，点击 "Plan and Execute" 即可看到运行效果

  

仿真异常处理方法：

![joint_controller](/joint_controller.png)

1.  安装 ros-control  ros-controller依赖

```
sudo apt-get install ros-$ROS_DISTRO-ros-control ros-$ROS_DISTRO-ros-controllers
```

2.   节点启动顺序：    先启动**gazebo** ，后启动**moveit**。



##  控制真实机械臂

* **使用如下命令连接机械臂, robot_ip 为实际机械臂所对应的IP地址**

  ```
  roslaunch dobot_bringup bringup.launch robot_ip:=192.168.5.1
  ```

* **使用如下命令启动 Moveit**

  ```
  roslaunch dobot_moveit moveit.launch
  ```



​     **如运行环境是Ubuntu16   Ros-Kinetic及以下版本，替换moveit.launch文件内容**

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

* 然后执行以下指令

```
source  ~/.bashrc 

source $HOME/catkin_ws/devel/setup.bash

export DOBOT_MOVEIT_PARENT_PATH=$(dirname $(rospack find dobot_moveit))

roslaunch dobot_moveit moveit.launch
```

* 如遇报错，请检查上述指令是否执行漏执行或者执行错误。



* **修改rviz_dobot_control文件目录下 robot_type.json文件内容，把robotType字段改为V3（默认为V3）**  

```
{
    "robotType": "v3"
}
```

![](/robotType.png)



* **在 rviz 中添加 DobotControl 插件控制面板，用来 使能机械臂**

  1. 点击 rviz 工具栏上的 Panels --> "Add New Panel"
  2. 选中 DobotControl, 再点击 “OK”
  3. 点击 “EnableRobot” 使机械臂
  4. 当状态样上显示 “Enable” “Connected” 表示机械臂已连接和使能，即可通过 Moveit 控制机械臂

  ![DobotControl](/cr5control.jpg)



# 自定义功能开发

    dobot_bringup 中定义了 msg 和 srv，用户可通过这些底层 msg 和 srv 实现对机械臂的控制



# 8. 常见问题

问题一：  Tcp连接  29999/30003端口无法连接或者连接后无法下发指令

 解决方法：  如控制器版本是3.5.6.0版本以下，可尝试升级控制器为3.5.6.1版本及以上版本。 如机器已经是3.5.6.1版本及以上，可将问题现象和操作反馈给技术支持



问题二： Tcp连接过程中  29999控制端口能发送指令，30003运动端口发送不了指令

 解决方法：  运动队列被堵塞，尝试用29999端口下发clearerror()和 continue()指令来重新开启队列，3.5.7.0以上版本支持continue指令。



问题三：怎么判断机器运动指令是否到位

解决方法：  可通过下发sync指令来判断机器运动指令是否到位

​                     可通过对比目标点位笛卡尔坐标值和机器实际笛卡尔坐标值来判断是否到位

​                     可通过对比目标关节坐标值和机器实际关节坐标值来判断是否到位



# 9. 示例

* Dobot-Demo 实现Tcp对机器的控制等交互，分别对控制端口，运动端口，反馈端口进行tcp连接，通过机器运动指令完成状态来进行下发指令，且对机器异常状态进行处理等功能。

  

1.  主线程：分别对机器控制端口，运动端口，反馈端口进行连接。给机器使能，MovL移动指令等动作

![](/main.png)

2.  反馈状态线程：实时反馈机器的状态信息

![](/feed.png)

3. 机器运动线程： 给机器下发运动指令

![](/move.png)

4.  异常处理线程：对机器异常状态进行判断和处理动作

![](/excetion.png)

**Demo运行的操作步骤时序如下图所示 ：**

1. 从GitHub 获取越疆Dobot  TCP-IP-ROS-6AXis  二次开发Api程序

   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. 通过LAN1网口-连接机器端，设置本机机器IP地址为192.168.5.X  网段

   控制面板>>网络>> Internet>>网络连接  

   ![](/netConnect.png)

   

   选择连接的以太网  >>  点击右键  >> 属性  >>   Internet协议版本(TCP/IPV4)

   修改ip地址为192.168.5.X网段IP

   <img src="/updateIP.png" style="zoom:80%;" />

   

3. 连接上位机DobotStudio Pro，连接机器，把机器模式切换至TCP/IP模式

   ![](/checkTcpMode.png)

   

   ```
   下载源码
   cd $HOME/catkin_ws/src
   git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   cd $HOME/catkin_ws
   
   编译
   catkin_make
   
   设置环境变量
   source $HOME/catkin_ws/devel/setup.bash
   
   导入机型
   echo "export DOBOT_TYPE=cr3" >> ~/.bashrc
   source ~/.bashrc
   
   ```

   ![](/bringup.png)



```
启动节点
roslaunch dobot_bringup bringup.launch
 
新建终端
cd $HOME/catkin_ws/src
source ~/.bashrc
source $HOME/catkin_ws/devel/setup.bash

启动demo
roslaunch rosdemo_v3  democr_v3.launch
```



![](/demov3.png)



**运行示例前请确保机器处于安全位置，防止机器发生不必要的碰撞**
