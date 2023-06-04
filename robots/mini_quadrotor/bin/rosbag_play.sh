#!/bin/bash

rosbag play ${1} --clock --pause quadrotor/debug/pose/pid:=quadrotor/debug/pose/pid_orig quadrotor/kf/imu1/data:=quadrotor/kf/imu1/data_orig quadrotor/kf/mocap1/data:=quadrotor/kf/mocap1/data_orig quadrotor/uav/baselink/odom:=quadrotor/uav/baselink/odom_orig quadrotor/uav/cog/odom:=quadrotor/uav/cog/odom_orig quadrotor/uav/full_state:=quadrotor/uav/full_state_orig
