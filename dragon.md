---
layout: default
title: Dragon
---


## servo ID in each link:

joint_yaw: 1 \
joint_pitch: 2 \
gimbal_pitch: 3 \
gimbal_roll: 4 

**note**: servo ID in `spinal/SetBoardConfig` : minus 1 (-1) from servo ID in each link (e.g. joint_yaw: 0)

## inital phase:
- check all neuron state by following command:
`$ rosservice call /get_board_info "{}" `

**Following is the result on 2018.12**
```
boards: 
  - 
    slave_id: 1
    imu_send_data_flag: 0
    servos: 
      - 
        id: 1
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 10
        current_limit: 648
        send_data_flag: 1
      - 
        id: 2
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 10
        current_limit: 648
        send_data_flag: 1
      - 
        id: 3
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 0
        current_limit: 0
        send_data_flag: 0
      - 
        id: 4
        p_gain: 2500
        i_gain: 100
        d_gain: 16000
        profile_velocity: 110
        current_limit: 0
        send_data_flag: 0
  - 
    slave_id: 2
    imu_send_data_flag: 0
    servos: 
      - 
        id: 1
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 10
        current_limit: 648
        send_data_flag: 1
      - 
        id: 2
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 10
        current_limit: 648
        send_data_flag: 1
      - 
        id: 3
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 0
        current_limit: 0
        send_data_flag: 0
      - 
        id: 4
        p_gain: 2500
        i_gain: 100
        d_gain: 16000
        profile_velocity: 110
        current_limit: 0
        send_data_flag: 0
  - 
    slave_id: 3
    imu_send_data_flag: 0
    servos: 
      - 
        id: 1
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 10
        current_limit: 648
        send_data_flag: 1
      - 
        id: 2
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 10
        current_limit: 648
        send_data_flag: 1
      - 
        id: 3
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 0
        current_limit: 0
        send_data_flag: 0
      - 
        id: 4
        p_gain: 2500
        i_gain: 100
        d_gain: 16000
        profile_velocity: 110
        current_limit: 0
        send_data_flag: 0
  - 
    slave_id: 4
    imu_send_data_flag: 0
    servos: 
      - 
        id: 3
        p_gain: 2500
        i_gain: 100
        d_gain: 3000
        profile_velocity: 0
        current_limit: 0
        send_data_flag: 0
      - 
        id: 4
        p_gain: 2500
        i_gain: 100
        d_gain: 16000
        profile_velocity: 110
        current_limit: 0
        send_data_flag: 0
```


 - check servo state by following command:
` $ rostopic echo /servo/states -c`

```
stamp: 
  secs: 1491045517
  nsecs:  35114374
servos: 
  - 
    index: 0
    angle: 4069
    temp: 30
    load: -3
    error: 0
  - 
    index: 1
    angle: 2010
    temp: 30
    load: 9
    error: 0
  - 
    index: 4
    angle: 4085
    temp: 28
    load: -8
    error: 0
  - 
    index: 5
    angle: 2038
    temp: 32
    load: -9
    error: 0
  - 
    index: 8
    angle: 4065
    temp: 29
    load: 8
    error: 0
  - 
    index: 9
    angle: 2039
    temp: 30
    load: -7
    error: 0
---
```
**Note**: the angle should be around 5, if the true joint angle is -1.57[rad], or should be around 4095 if the true joint angle is 1.57[rad].

- set the local index 1 servo of No.2 neuron to 5 (neutral point).

## calibration 
### joint 
#### guarantee the enough tension for each joint, using the clamp as shown in following figure.
![](https://github.com/tongtybj/aerial_robot/blob/documentation/images/dragon_clamp.jpg)

#### yaw: using following command (for neuron id 2) at the joint angles is ideally -1.57[rad]
`$ rosserive call /set_board_info spinal/SetBoardInfo “command: 2 data: [2, 0, 5]”  ` \
**Note**: set a small offset (e.g. 5) from the true value 0, because of the different gear ratio.

### pitch: using following command (for neuron id 2)  at the joint angles is ideally 0[rad]
`$ rosserive call /set_board_info spinal/SetBoardInfo “command: 2 data: [2, 1, 2047]”  `

### gimbal
#### use following tool to make sure the gimbal module is level against the ground, then use following command.
![](https://github.com/tongtybj/aerial_robot/blob/documentation/images/gimbal_tool.jpg)
`$ rosserive call /set_board_info spinal/SetBoardInfo “command: 2 data: [2, 2, 2047]”  ` \
`$ rosserive call /set_board_info spinal/SetBoardInfo “command: 2 data: [2, 3, 2047]”  `

## Servo PID gain and velocity
### joint
#### XH430-W350R
- PID gains: [2500, 100, 3000], date: 2017.08
- velocity: 25, date: 2019.03

### gimbal pitch
#### MX28-AR
- PID gains: [2500,100,3000], date: 2017.08
- velocity: 0, date: 2017.08
### gimbal roll
#### MX28-AR (link2, link3)
- PID gains: [2500,100,16000], date: 2017.08
- velocity: 110, date: 2017.08

#### XH430-W350R
- PID gains: [1000,100,2500], date: 2019.02
- velocity: 0, date: 2019.02 (**note**: this is the max speed, but still slower than other type servo)

#### XM430-W210R (link1), no reduction
- PID gains: [1500,100,**10000**], date: 2019.05.26
- velocity: 110, date: 2019.05.26

#### XM430-W210R (link4), no reduction
- PID gains: [1500,100,**6000**], date: 2019.05.26
- velocity: 110, date: 2019.05.26

#### XM430-W210R (link1/link4), with reduction (34-60)
- PID gains: [1000,100,4000], date: 2019.05.27
- velocity: 220, date: 2019.05.27
- with rigid, no any dead zone