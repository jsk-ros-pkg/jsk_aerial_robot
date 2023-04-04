#!/bin/bash

rosbag play $1 --pause --clock /spidar/joints_ctrl:=/spidar/joints_ctrl_orig /spidar/joints_torque_ctrl:=/spidar/joints_torque_ctrl_orig /spidar/debug/pose/pid:=/spidar/debug/pose/pid_orig /spidar/four_axes/command:=/spidar/four_axes/command_orig /spidar/debug/target_vectoring_force:=/spidar/debug/target_vectoring_force_orig /spidar/servo/target_states:=/spidar/servo/target_states_orig ${@:2}
