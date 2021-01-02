---
layout: default
---

# Requirement:
```
$ roslaunch spinal_ros_bridge serial.launch
```

# 1. IMU calibration

check https://github.com/tongtybj/aerial_robot/pull/357

# 2. Neuron calibration

check https://github.com/tongtybj/aerial_robot/pull/302

# 3. Battery voltage calibration
```
$ rostopic pub -1 /set_adc_scale std_msgs/Float32 "data: 0.01265"
```
**note**: `0.01265` is the nominal value. You have to tune this value using power supply.