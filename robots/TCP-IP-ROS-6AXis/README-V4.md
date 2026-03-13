---
typora-root-url: ./image
---

# <center>ROS-Robot</center>



English version of the README -> please [click here](./README-V4-EN.md)

Dobot  TCP-IP-ROS-6AXis   二次开发api接口 （ [TCP-IP-ROS-6AXis Public README](https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git) ）



# 1. 简介

TCP-IP-ROS-6AXis是为 dobot 公司旗下基于TCP/IP协议的ROS的封装设计的软件开发套件。它基于 ROS/C++ 语言开发，遵循Dobot-TCP-IP控制通信协议，通过socket与机器终端进行Tcp连接，  并为用户提供了易用的api接口。通过 TCP-IP-ROS-6AXis，用户可以快速地连接Dobot机器并进行二次开发对机器的控制与使用。



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

* 此API接口与Demo适用于V4系列的440及以上控制器版本

  


##  版本和发布记录

###  当前版本 v1.0.0.0

|   版本   |  修改日期  |
| :------: | :--------: |
| v1.0.0.0 | 2023-11-20 |



# 2. 技术支持




# 3. CR_ROS 控制协议

由于基于TCP/IP的通讯具有成本低、可靠性高、实用性强、性能高等特点；许多工业自动化项目对支持TCP/IP协议控制机器人需求广泛，因此Dobot机器人将设计在TCP/IP协议的基础上，提供了丰富的接口用于与外部设备的交互；

机器状态等级：

    1.报错状态为第一优先级，如机器人报错且未上使能，则状态返回为报错状态
    
    2.下电状态为第二优先级，如机器人未上电，未上使能，则状态返回未上电状态
    
    3.碰撞状态为第三优先级，如机器人处于碰撞，脚本暂停，则状态返回碰撞状态
    
    4.开抱闸状态为第四优先级，如机器人开抱闸，未使能状态，则状态返回开抱闸状态
    
    ​ 其余状态根据实际情况反馈。


端口反馈信息：

    29999服务器端口通过一发一收的方式负责接收一些设置以及运动控制相关的指令
    
    30004服务器端口(以下简称实时反馈端口)每8ms反馈机器人的信息
    
    30005服务器端口每200ms反馈机器人的信息
    
    30006端口为可配置的反馈机器人信息端口(默认为每50ms反馈)
    
    取消30003端口

有关协议更详细的信息请查阅**《[越疆TCPIP控制协议文档6AXis-V4](https://github.com/Dobot-Arm/TCP-IP-Protocol-6AXis-V4.git)》**



# 4. 获取 TCP-IP-ROS-6AXis

1. 从GitHub 下载或者克隆Dobot  TCP-IP-ROS-6AXis 二次开发api程序

   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. 参考对应的 README-V4.md 文档使用；

   

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
source $HOME/catkin_ws/devel/setup.bash
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
  roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.5.1
  ```

* **使用如下命令启动 Moveit**

  ```
  roslaunch dobot_moveit moveit.launch
  ```



**如运行环境是Ubuntu16   Ros-Kinetic及以下版本，替换moveit.launch文件内容**

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



* **修改rviz_dobot_control文件目录下 robot_type.json文件内容，把robotType字段改为V4（默认为V3）**  

```
{
    "robotType": "v4"
}
```

![](/robotType.png)



* **在 rviz 中添加 DobotControl 插件控制面板，用来 使能机械臂**

  1. 点击 rviz 工具栏上的 Panels --> "Add New Panel"
  2. 选中 DobotControl, 再点击 “OK”
  3. 点击 “EnableRobot” 使机械臂
  4. 当状态样上显示 “Enable” “Connected” 表示机械臂已连接和使能，即可通过 Moveit 控制机械臂

  ![DobotControl](/cr5control.jpg)



## 控制多台真实机械臂

* **修改dobot_v4_bringup/launch目录下bringup_v4.launch文件, 找到  <param name="num_nodes" type="int" value="1" />字段， num_nodes对应的value值改为大于1  **

  ```
   eg：
   <arg name="robotIp" doc="IP of the controller" default="192.168.5.1"/>
    <arg name="robotName" doc="Name of the controller" default=""/>
  
    <node name="$(arg robotName)$(env DOBOT_TYPE)_robot" pkg="dobot_v4_bringup" type="dobot_v4_bringup" output="screen" >
      <param name="num_nodes" type="int" value="2" />
      <param name="robot_node_name" type="str" value="$(arg robotName)"/>
      <param name="joint_publish_rate" type="double" value="10" />
      <param name="trajectory_duration" type="double" value="0.1" />
      <param name="robot_ip_address" type="str" value="$(arg robotIp)"/>
    </node>
  ```

* **使用命令连接机械臂A, robot_ip 为实际机械臂所对应的IP地址**

  ```
  roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.100.10 robotName:=robotA
  ```

* **使用命令连接机械臂B, robot_ip 为实际机械臂所对应的IP地址**

  ```
  roslaunch dobot_v4_bringup bringup_v4.launch robotIp:=192.168.100.20 robotName:=robotB
  ```



# 自定义功能开发

    dobot_bringup 中定义了 msg 和 srv，用户可通过这些底层 msg 和 srv 实现对机械臂的控制



# 7. 常见问题

* Tcp连接问题：

  29999端口为单客户端连接  ，只允许一个客户端连接；

  30004  30005  30006端口可以多客户端同时连接；

  29999端口有机器人模式限制，开放前需要先将机器人设置为TCP模式，否则指令发送后无法响应并返回"Control Mode Is Not Tcp"，30004、30005、30006实时反馈端口无模式限制；

* 要获取机器人的状态， 可监测RobotMode()返回值。

  1. 控制器启动阶段为初始化状态，返回1；

  2. 机器人状态不为0或1时，机器人初始化完成；

  3. 机器下电状态，返回3；

  4. 初始化完成但未上使能为未使能状态，返回4；

  5. 机器使能空闲状态，返回5，代表可正常接收运动指令；

  6. movj(运行相关指令)、脚本运行及其他队列指令统一为运行状态，返回7；

  7. movJog（点动）为单次运动状态，返回8；无上电状态；

     | 模式 | 描述                   | 备注                            |
     | ---- | ---------------------- | ------------------------------- |
     | 1    | ROBOT_MODE_INIT        | 初始化状态                      |
     | 2    | ROBOT_MODE_BRAKE_OPEN  | 抱闸松开                        |
     | 3    | ROBOT_MODE_POWEROFF    | 本体下电状态                    |
     | 4    | ROBOT_MODE_DISABLED    | 未使能(抱闸未松开)              |
     | 5    | ROBOT_MODE_ENABLE      | 使能(空闲)                      |
     | 6    | ROBOT_MODE_BACKDRIVE   | 拖拽                            |
     | 7    | ROBOT_MODE_RUNNING     | 运行状态（含脚本和TCP队列运行） |
     | 8    | ROBOT_MODE_SINGLE_MOVE | 单次运动状态(点动)              |
     | 9    | ROBOT_MODE_ERROR       | 错误状态                        |
     | 10   | ROBOT_MODE_PAUSE       | 暂停状态                        |
     | 11   | ROBOT_MODE_COLLISION   | 碰撞状态                        |

* 不同机器状态响应指令

  ​    *错误状态可执行指令：ClearError()、GetErrorID()、EmergeStop()、RobotMode()，其余指令均拒绝指令，返回-2；

  ​	*急停状态可执行指令：ClearError()、GetErrorID()、EmergeStop()、RobotMode()，其余指令均拒绝指令，返回-3；

  ​	*下电状态可执行指令：ClearError()、GetErrorID()、EmergeStop()、RobotMode()，PowerOn()，其余指令均拒绝指令，返回-4；

  ​	*TCP指令除必要的影响到机器人状态的指令外（EnableRobot()、PowerOn()），其余均为立即返回指令，指令返回仅代表成功发送，不代表执行完成；

* 坐标系问题

  进入TCP/IP模式默认会将用户/工具坐标系设置为0，0，退出TCP/IP模式自动恢复至上位机设置的用户/工具坐标系索引值；

  ​      全局坐标：User()、Tool()指令设置的是全局坐标系，设置后对所有指令均生效；

  ​      局部坐标：运动指令中带的user/tool可选参仅在当前运动指令生效，执行完当前指令后恢复至全局坐标系；

* 算法允许的队列深度为64，可同时连续处理64条队列指令，若一直向端口发送队列指令而超出队列深度，端口不会断连，指令接口会返回-1，指令不会执行

* 队列指令为立即返回指令，接口返回成功仅代表发送成功，不代表执行完毕。若判断执行完毕，则需要结合CommandID和RobotMode来综合判断

* 伺服运行时间代表上使能后的累计时间，控制器开机时间代表控制器开机后的时间（正常上电），因此控制器开机时间总要大于伺服运行时间的

* 碰撞状态不属于错误，GetErrorID()中没有碰撞的错误返回，可通过RobotMode()查询碰撞状态，返回11；ClearError()可清除碰撞状态

* 发生碰撞后正常发送清错指令，清除碰撞状态即可重新运行，不需要重新上使能

* 切换TCP/IP模式后会默认将速度/加速度/用户、工具坐标系参数设置为默认值，无需调用EnableRobot()指令进行默认参数设置

* Tcp队列指令返回值仅表示指令参数和语法是否符合规范，不表示指令是否成功执行。TCP队列指令发送后即返回，返回0仅代表正常发送，不表示可成功执行。

* Tcp指令参数类型/数量错误会直接报警，不会补0处理，指令不会下发至算法。

* 触发急停信号后，机器人默认进行运动停止，若停止过程超过500ms还未停止运动，则执行下电动作；否则不进行下电；一般情况下机器人急停后不下电

* 设置默认网关后，数值默认网关数据会保存，重启后数值不会更改

* 上使能指令仅执行使能动作，无其余参数设置，不会清除运动学参数和坐标系参数。

* Pause()、Continue()指令对脚本运行生效，运动指令（队列相关）也生效，调用Pause()指令后机器人进入暂停状态，算法队列暂停；可使用Continue()指令继续运行队列指令。MovJog(点动)指令属于单次运行状态，不可暂停和继续

* 目前的TCP的指令之间不支持插入无关的字符  可以使用如下两种格式来写
  ① MovJ()MovJ()MovJ()

  ②
  MovJ()
  MovJ()
  MovJ()





# 8. 示例

* Dobot-Demo 实现Tcp对机器的控制等交互，分别对控制端口，反馈端口进行tcp连接，通过机器运动指令完成状态来进行下发指令，且对机器异常状态进行处理等功能。

  

1.  主线程：分别对机器控制端口，运动端口，反馈端口进行连接。给机器使能，MovL移动指令等动作

![](/main.png)

2.  反馈状态线程：实时反馈机器的状态信息

![](/feed.png)

3. 机器运动线程： 给机器下发运动指令

![](/move.png)

运动指令到位信号：

队列指令为立即返回指令，接口返回成功仅代表发送成功，不代表执行完毕。若判断执行完毕，则需要结合

* 当前CommandID大于下发运动队列指令的CommandID，则表示下发队列指令已完成。

* 当前CommandID等于下发运动指令的CommandID，且机器状态RobotMode指令返回值为5，则表示下发队列指令已完成。

  

4.  异常处理线程：对机器异常状态进行判断和处理动作

![](/excetion.png)



**Demo运行的操作步骤时序如下图所示 ：**

1. 从GitHub 获取越疆dobot  TCP-IP-CR-Python-V4 二次开发Api程序

   ```bash
   `git clone https://github.com/Dobot-Arm/TCP-IP-ROS-6AXis.git
   ```

2. 通过LAN1网口-连接机器端，设置本机机器IP地址为192.168.5.X  网段

   控制面板>>网络>> Internet>>网络连接  

   ![](/netConnect.png)

   

   选择连接的以太网  >>  点击右键  >> 属性  >>   Internet协议版本(TCP/IPV4)

   修改ip地址为192.168.5.X网段IP

   ![](/updateIP.png)

   

3. 连接上位机DobotStudio Pro，连接机器，把机器模式切换至TCP/IP模式

   ![](/checkTcpMode.png)





#### rosdemo_v4

```python
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

启动节点
roslaunch dobot_v4_bringup bringup_v4.launch
 
新建终端
cd $HOME/catkin_ws/src
source ~/.bashrc
source $HOME/catkin_ws/devel/setup.bash

启动demo
roslaunch rosdemo_v4  democr_v4.launch

```



控制机械臂运动： 下发运行指令，控制机械臂运动

```python
 launch文件设置运动指令点位
    
    <launch>
   
    <!-- 启动一个节点 -->
    <node name="rosdemo_v4" pkg="rosdemo_v4" type="rosdemo_v4" output="screen">
        <!-- 添加节点的参数，如果需要的话 -->
        <!-- <param name="pointA" type="double" value="10"/>
        <param name="pointB" type="double" value="10" /> -->
       <rosparam param="pointA">[-90, 20, 0, 0, 0, 0]</rosparam>
       <rosparam param="pointB">[90, 20, 0, 0, 0, 0]</rosparam>
    </node>
</launch>

    
 通过sersice下发运动指令  
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



运行指令完成标志： 等待机械臂运动指令完成 ，类似与 Sync功能。

```python

运动指令反馈currentCommandID： 服务的返回值 srvMovJ.response.res

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



获取机械臂信息： 获取反馈端口机械臂状态信息

```python
通过Topic话题获取机械臂反馈信息

subFeedInfo = nh->subscribe("/dobot_v4_bringup/msg/FeedInfo", 10, &RosDemoCRV4::getFeedBackInfo, this);


回调函数，自定义添加所需的机械臂信息
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



监控机器状态： 监控机械臂异常状态和机械臂清错功能

```python
监控机械臂状态线程
void RosDemoCRV4::warmRobotError()
{
    while (true) {
        {
              ... ...
                // 请求服务
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

                                
 清错指令：
rosservice  call  /dobot_v4_bringup/srv/ClearError
```



**运行示例前请确保机器处于安全位置，防止机器发生不必要的碰撞**

