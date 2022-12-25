#!/bin/bash

rosbag play $1 --pause --clock /spider/joints_ctrl:=/spider/joints_ctrl_orig /spider/joints_torque_ctrl:=/spider/joints_torque_ctrl_orig /spider/debug/pose/pid:=/spider/debug/pose/pid_orig /spider/four_axes/command:=/spider/four_axes/command_orig /spider/debug/target_vectoring_force:=/spider/debug/target_vectoring_force_orig /spider/servo/target_states:=/spider/servo/target_states_orig ${@:2}
