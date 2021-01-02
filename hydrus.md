---
layout: default
title: Hydrus
---

## old model:

#### joint servo: 

dynamixel MX28AR

#### neuron configuration (includeing servo)

```
boards: 
  - 
    slave_id: 1
    imu_send_data_flag: 1
    dynamixel_ttl_rs485_mixed: 0
    servos: 
      - 
        id: 1
        p_gain: 5000
        i_gain: 0
        d_gain: 1000
        profile_velocity: 30
        current_limit: 0
        send_data_flag: 1
  - 
    slave_id: 2
    imu_send_data_flag: 1
    dynamixel_ttl_rs485_mixed: 0
    servos: 
      - 
        id: 1
        p_gain: 5000
        i_gain: 0
        d_gain: 1000
        profile_velocity: 30
        current_limit: 0
        send_data_flag: 1
  - 
    slave_id: 3
    imu_send_data_flag: 1
    dynamixel_ttl_rs485_mixed: 0
    servos: 
      - 
        id: 1
        p_gain: 5000
        i_gain: 0
        d_gain: 1000
        profile_velocity: 30
        current_limit: 0
        send_data_flag: 1
  - 
    slave_id: 4
    imu_send_data_flag: 1
    dynamixel_ttl_rs485_mixed: 1
    servos: []
```