#!/bin/bash

rosrun mini_quadrotor rosbag_record.sh ${1:-quadrotor} quadrotor/livox/lidar quadrotor/livox/imu quadrotor/Odometry quadrotor/Odometry_precede ${@:2}
