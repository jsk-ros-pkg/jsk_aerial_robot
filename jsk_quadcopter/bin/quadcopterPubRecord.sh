#!/bin/sh
rosbag record /imu/debug /slam/debug /mirror/debug /controller/debug /kduino/imu  /slam/slam_out_pose /scan /px4flow/opt_flow /opticalFlow/debug /drop_position /camera/image_raw_throttle /camera/camera_info
