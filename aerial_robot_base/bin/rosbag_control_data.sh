#!/bin/bash

rosrun aerial_robot_base rosbag_raw_sensors.sh $1 rosout rosout_agg $1/four_axes/command $1/rpy/pid  $1/rpy/gain $1/debug/pose/pid $1/desire_coordinate $1/flight_config_cmd $1/kf/imu1/data  $1/kf/alt1/data $1/kf/gps1/data $1/kf/mocap1/data $1/kf/vo1/data $1/ground_truth $1/debug/four_axes/gain $1/motor_info $1/p_matrix_pseudo_inverse_inertia $1/servo/torque_enable $1/servo/target_states /tf_static /tf $1/uav/baselink/odom $1/uav/cog/odom $1/uav/full_state $1/uav_info $1/uav_power $1/uav/nav $1/joints_ctrl $1/joint_states $1/joy ${@:2}
