#!/bin/sh

rosbag record /aerial_robot/ground_pose /aerial_robot/pose /baro /battery_voltage_status /camera_velocity /distance /gps /imu /joy /motor_pwms /servo/states /uav/nav /flight_config_ack /flight_state $*

