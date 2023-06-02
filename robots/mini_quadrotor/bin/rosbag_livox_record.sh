#!/bin/bash

rosrun mini_quadrotor rosbag_record.sh ${1:-quadrotor} /livox/lidar /livox/imu /Odometry /Odometry_precede ${@:2}
