#!/bin/bash

rosbag play $1 --pause --clock /tiger/joints_ctrl:=/tiger/joints_ctrl_orig /tiger/joints_torque_ctrl:=/tiger/joints_torque_ctrl_orig /tiger/debug/pose/pid:=/tiger/debug/pose/pid_orig /tiger/four_axes/command:=/tiger/four_axes/command_orig ${@:2}
