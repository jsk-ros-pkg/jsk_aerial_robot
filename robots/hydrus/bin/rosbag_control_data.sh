#!/bin/sh

rosrun aerial_robot_base rosbag_control_data.sh /hydrus/joints_ctrl /hydrus/joint_states /zed/odom /realsense1/odom/throttle /realsense1/odom /realsense2/odom/throttle /realsense2/odom $*
