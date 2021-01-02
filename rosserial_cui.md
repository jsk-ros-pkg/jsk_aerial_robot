---
layout: default
title: rosserial_cui
---


### get general configuration use rosservice:
` $ rosserive call /get_board_info spinal/GetBoardInfo {} `

**note1**: neuron board ID is from 1 \
**note2**: servo ID is from 0 

### set general configuration use rosservice:

`spinal/SetBoardInfo` is as follows:
```
#commands                                                                                                                                                                                                   
## slave_id is from 1                                                                                                                                                                                       
## servo_index is from 0                                                                                                                                                                                    
uint8 SET_SLAVE_ID = 0
# [slave_id, new_slave_id]                                                                                                                                                                                  
uint8 SET_IMU_SEND_FLAG = 1
# [slave_id, flag]                                                                                                                                                                                          
uint8 SET_SERVO_HOMING_OFFSET = 2
# [slave_id, servo_index, joint_offset] !torque disable!                                                                                                                                                    
uint8 SET_SERVO_PID_GAIN = 3
# [slave_id, servo_index, p_gain, i_gain, d_gain]                                                                                                                                                           
uint8 SET_SERVO_PROFILE_VEL = 4
# [slave_id, servo_index, profile_vel]                                                                                                                                                                      
uint8 SET_SERVO_SEND_DATA_FLAG = 5
# [slave_id, servo_index, flag]                                                                                                                                                                             
uint8 SET_SERVO_CURRENT_LIMIT = 6
# [slave_id, servo_index, current_limit] !torque disable!                                                                                                                                                   

uint8 command
int32[] data
---
bool success
```
- change the servo speed (commad ID: 4) on local index 0 servo of No.4 neuron.\
` $ rosserive call /set_board_info spinal/SetBoardInfo “command: 4 data: [4, 0, 10]” `

-  modify the servo gains (P: 2500, I: 100, D:3000) on local index 3 (4th) servo of No.3 neuron.\
` $ rosserive call /set_board_info spinal/SetBoardInfo  “command: 3 data: [3, 3, 2500, 100, 3000]” ` 

- set the local index 3 servo of No.2 neuron to 2047 (neutral point).\
` $ rosserive call /set_board_info spinal/SetBoardInfo “command: 2 data: [2, 3, 2047]” `

### servo torque enable/disable

`$ rostopic pub -1 /servo/torque_enable xxxxx/ServoTorqueCmd “index: [0, 1, 4, 5, 8, 9] torque_enable: [0, 0, 0, 0, 0, 0]” `
**meaning**: disable servo (global index: 0, 1, 4, 5, 8, 9), `xxxxx` is the name of robot. \
**note**: use the global index of servos.