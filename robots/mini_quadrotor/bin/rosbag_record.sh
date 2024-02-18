#!/bin/bash

rosrun aerial_robot_base rosbag_control_data.sh ${1:-quadrotor}  ${@:2}
