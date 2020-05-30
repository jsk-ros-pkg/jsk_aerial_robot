#!/bin/bash

rosrun aerial_robot_base rosbag_raw_sensors.sh ${1:-hydrus} ${1:-hydrus}/zed/odom ${1:-hydrus}/realsense1/odom/throttle ${1:-hydrus}/realsense1/odom ${1:-hydrus}/realsense2/odom/throttle ${1:-hydrus}/realsense2/odom ${@:2}
