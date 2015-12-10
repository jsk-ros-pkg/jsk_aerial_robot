#!/bin/sh

rosbag record /camera/camera_info /camera/image_raw /controller/debug /estimator/full_states /imu/data /kduino/imu /navigator/full_states /optical_flow/data /px4flow/opt_flow /red_tracking_out /result /world_cam_coord
