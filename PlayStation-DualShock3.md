---
layout: default
title: PSjoy3
---


## PlayStation DualShock3 
![PS3joy](http://wiki.ros.org/ps3joy?action=AttachFile&do=get&target=ps3_buttons.jpg)
### Install for wireless connection
Please refer to https://retropie.org.uk/forum/topic/2913/guide-use-qtsixa-on-ubuntu-16-04-and-derivatives
```
$ sudo apt-get install  pyqt4-dev-tools   libusb-dev libjack-dev libbluetooth-dev python-dbus -y
$ git clone https://github.com/falkTX/qtsixa.git
$ cd qtsixa
$ make
$ sudo make install
```
### Pairing 
Connect the controller with host PC via USB cable, and run following command:
```
$ sudo sixpair
```

### Wireless Connection
Unplug the joystick from the computer, and run following command:
```
$ sixad --start
```
Press the PS button in the middle of the joystick and the connection will be activated. 

### Start sixad on boots
edit `/etc/rc.local`, and add following description before `exit 0`
``` 
sixad --start 
```
Note: if you can not connect with pc even pressing the PS button, please restart the sixad by following command:
```
$ sixad -r
```

### Common commands:
| command  | action |
|:-:|:-:|
| `start`| motor arming (please do this before takeoff)|
| `cross-left + circle` | takeoff (this can be received by robot only after motor arming)|
| `cross-right + square`| landing|
| `select` (short push) | force landing (without xy position control, robot will descend slowly)|
| `select` (long push, > 2.0s) | halt (i.e. stop motor immediately )|
| `triangle` | xy(horizontal) velocity control mode|
| `cross(X)` | xy(horizontal) position control mode|
| `cross-down`| xy(horizontal) attitude control mode|
| `left stick vertical`| movement in world x axis (only available in velocity/attitude control mode)|
| `left stick horizontal`| movement in world y axis (only available in velocity/attitude control mode)|
| `left stick vertical` + `L2`| movement in baselink (local) x axis (only available in velocity/attitude control mode)|
| `left stick horizontal` + `L2`| movement in baselink (local) y axis (only available in velocity/attitude control mode) |
| `right stick vertical`| movement in z axis |
| `right stick horizontal`| movement in yaw axis |

For more information about ps3joy in ros, please check [here](http://wiki.ros.org/ps3joy)