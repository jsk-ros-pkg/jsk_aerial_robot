#!/bin/bash

rosbag record $1/mocap/pose $1/mocap/ground_pose $1/baro $1/battery_voltage_status $1/leddar_one/range $1/gps $1/imu $1/joy $1/motor_pwms $1/servo/states $1/uav/nav $1/flight_config_ack $1/flight_state ${@:2}
