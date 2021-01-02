---
layout: default
title: Motion-Capture
---


#### Product:  Optitrack [Prime13](http://www.mocap.jp/optitrack/products/prime-13/) and [Prime13W](http://www.mocap.jp/optitrack/products/prime-13w/)。

#### Motive Version: 1.7.5。

#### Calibration:
1. set the THR for prim13W from 255 to 100, since the wanding ball is too small.
![](https://github.com/tongtybj/aerial_robot/blob/documentation/images/mocap_calib1.png)

2. set ground plane: 50mm
![](https://github.com/tongtybj/aerial_robot/blob/documentation/images/mocap_calib2.png)

#### How to use in ROS:
https://github.com/tuw-cpsg/tuw-cpsg.github.io/tree/master/tutorials/optitrack-and-ros

#### パケットが来ているか確認する方法
`roslaunch aerial_robot_base mocap.launch`

をしてから，

`sudo tcpdump host [mocap ip] -i [device]`

ex. `sudo tcpdump host 192.168.1.100 -i wlan0`