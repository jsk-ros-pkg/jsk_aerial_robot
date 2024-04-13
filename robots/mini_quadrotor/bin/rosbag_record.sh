#!/bin/bash

rosrun aerial_robot_base rosbag_control_data.sh ${1:-quadrotor} quadrotor/livox/lidar quadrotor/livox/imu quadrotor/Odometry quadrotor/Odometry_precede ${@:2}
