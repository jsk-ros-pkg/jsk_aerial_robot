#!/usr/bin/env python

from http import server
import queue
import sndhdr
import sys
import time
from tkinter import SE
import rospy
import math
import copy
import select, termios, tty
import numpy as np
import matplotlib.pyplot as plt
import tf.transformations as tf
from spinal.msg import FourAxisCommand
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import PoseControlPid
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped


class analisys():

    def __init__(self):

        self.start_time = 60.0 # fill in start time of rosbag

        self.four_axes_sub = rospy.Subscriber('/hydrus_xi/four_axes/command', FourAxisCommand, self.four_axes_cb)
        self.uav_sub = rospy.Subscriber('/hydrus_xi/uav/cog/odom', Odometry, self.uav_cb)
        self.debug_errp_sub = rospy.Subscriber('/hydrus_xi/debug/pose/pid', PoseControlPid, self.debug_cb)
        self.hand_force_switch_sub = rospy.Subscriber('/hydrus_xi/hand_force_switch', Int8, self.hand_force_switch_cb)
        self.avatar_sub = rospy.Subscriber('/avatar_mocap_node/avatar/pose', PoseStamped, self.avatar_cb)
        self.four_axes = [0,0,0]
        self.uav_euler = [0,0,0]
        self.diff_array_roll = []
        self.diff_array_pitch = []
        self.diff_array_yaw = []
        self.time_array = []
        self.err_x_array = []
        self.err_y_array = []
        self.err_z_array = []
        self.err_yaw_array = []
        self.debug_time_array = []
        self.time = 0
        self.seq_0 = 0
        self.debug_time = 0
        self.debug_seq_0 = 0
        self.std_dev_roll = 0
        self.std_dev_pitch = 0
        self.std_dev_yaw = 0
        self.std_dev_x = 0
        self.std_dev_y = 0
        self.std_dev_z = 0
        self.std_dev_err_yaw = 0
        self.seq_init_flag = False
        self.debug_seq_init_flag = False
        self.hand_force_switch_flag = False
        self.avatar_flag = False

    def uav_cb(self,msg):
        uav_quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] 
        self.uav_euler = list(tf.euler_from_quaternion(uav_quaternion))
        # print("uav=", self.uav_euler)
        seq = msg.header.seq
        if self.seq_init_flag == False:
            self.seq_0 = seq
            self.seq_init_flag = True
        self.time = (seq-self.seq_0)/100 + self.start_time

    def four_axes_cb(self,msg):
        self.four_axes = list(msg.angles)
        # print("four_axes=", self.four_axes)
        if self.uav_euler[0] != 0:
            roll_diff = self.four_axes[0] - self.uav_euler[0]
            pitch_diff = self.four_axes[1] - self.uav_euler[1]
            yaw_diff = self.four_axes[2] - self.uav_euler[2]
            self.diff_array_roll.append(roll_diff)
            self.diff_array_pitch.append(pitch_diff)
            self.diff_array_yaw.append(yaw_diff)
            self.std_dev_roll = np.std(self.diff_array_roll)
            self.std_dev_pitch= np.std(self.diff_array_pitch)
            self.std_dev_yaw = np.std(self.diff_array_yaw)
            self.time_array.append(self.time)

    def debug_cb(self,msg):
        # if self.hand_force_switch_flag == True:
        #     self.err_x_array.append(msg.x.err_p)
        #     self.err_y_array.append(msg.y.err_p)
        #     self.err_z_array.append(msg.z.err_p)
        #     self.err_yaw_array.append(msg.yaw.err_p)
        # if self.hand_force_switch_flag == False:
        #     self.err_x_array.append(0.0)
        #     self.err_y_array.append(0.0)
        #     self.err_z_array.append(0.0)
        #     self.err_yaw_array.append(0.0)

        if self.avatar_flag == True:
            self.err_x_array.append(msg.x.err_p)
            self.err_y_array.append(msg.y.err_p)
            self.err_z_array.append(msg.z.err_p)
            self.err_yaw_array.append(msg.yaw.err_p)
        if self.avatar_flag == False:
            self.err_x_array.append(0.0)
            self.err_y_array.append(0.0)
            self.err_z_array.append(0.0)
            self.err_yaw_array.append(0.0)

        seq = msg.header.seq
        if self.debug_seq_init_flag == False:
            self.debuf_seq_0 = seq
            self.debug_seq_init_flag = True
        self.debug_time = (seq-self.debug_seq_0)/100 + self.start_time
        self.debug_time_array.append(self.debug_time)
        self.std_dev_x = np.std(self.err_x_array)
        self.std_dev_y = np.std(self.err_y_array)
        self.std_dev_z = np.std(self.err_z_array)
        self.std_dev_error_yaw = np.std(self.err_yaw_array)

    def hand_force_switch_cb(self,msg):
        if msg.data == 1:
            self.hand_force_switch_flag = True
            print("flag True")
        if msg.data == 0:
            self.hand_force_switch_flag = False
            print("flag False")

    def avatar_cb(self,msg):
        avatar_pos_z = msg.pose.position.z
        if avatar_pos_z > 0.75:
            self.avatar_flag = True
        else:
            self.avatar_flag = False


    def main(self):

        r = rospy.Rate(200)
        while not rospy.is_shutdown():

            r.sleep()

    def print(self):
        print("once")
        print("std dev of roll = ", self.std_dev_roll)
        print("std dev of pitch = ", self.std_dev_pitch)
        print("std dev of yaw = ", self.std_dev_yaw)
        print("std dev of x = ", self.std_dev_x)
        print("std dev of y = ", self.std_dev_y)
        print("std dev of z = ", self.std_dev_z)
        print("std dev of error yaw = ", self.std_dev_error_yaw)


        # plot
        plt.rcParams["font.size"] = 24
        plt.rcParams["svg.fonttype"] = "none"

        plt.grid(color='k', linestyle=':',linewidth=0.3)
        plt.title("attitude_diff")
        plt.plot(self.time_array, self.diff_array_roll, 'r', label="roll_diff")
        plt.plot(self.time_array, self.diff_array_pitch, 'b', label="pitch_diff")
        plt.plot(self.time_array, self.diff_array_yaw, 'g', label="yaw_diff")
        plt.xlabel("time[s]")
        plt.ylabel("angle_diff[rad]")
        plt.xlim()
        plt.ylim()
        plt.legend()
        plt.show()

        # plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("pitch_diff")
        # plt.plot(self.time_array, self.diff_array_pitch, 'r', label="pitch_diff")
        # plt.xlabel("time[s]")
        # plt.ylabel("angle_diff[rad]")
        # plt.xlim()
        # plt.ylim()
        # plt.legend()
        # plt.show()

        # plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("yaw_diff")
        # plt.plot(self.time_array, self.diff_array_yaw, 'r', label="yaw_diff")
        # plt.xlabel("time[s]")
        # plt.ylabel("angle_diff[rad]")
        # plt.xlim()
        # plt.ylim()
        # plt.legend()
        # plt.show()

        plt.grid(color='k', linestyle=':',linewidth=0.3)
        plt.title("pos_err")
        plt.plot(self.debug_time_array, self.err_x_array, 'r', label="x_err")
        plt.plot(self.debug_time_array, self.err_y_array, 'b', label="y_err")
        plt.plot(self.debug_time_array, self.err_z_array, 'g', label="z_err")
        plt.xlabel("time[s]")
        plt.ylabel("position_error[m]")
        plt.xlim()
        plt.ylim()
        plt.legend()
        plt.show()

        # plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("y_err")
        # plt.plot(self.debug_time_array, self.err_y_array, 'r', label="y_err")
        # plt.xlabel("time[s]")
        # plt.ylabel("position_error[m]")
        # plt.xlim()
        # plt.ylim()
        # plt.legend()
        # plt.show()

        # plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("z_err")
        # plt.plot(self.debug_time_array, self.err_z_array, 'r', label="z_err")
        # plt.xlabel("time[s]")
        # plt.ylabel("position_error[m]")
        # plt.xlim()
        # plt.ylim()
        # plt.legend()
        # plt.show()
                
        plt.grid(color='k', linestyle=':',linewidth=0.3)
        plt.title("yaw_err")
        plt.plot(self.debug_time_array, self.err_yaw_array, 'r', label="yaw_err")
        plt.xlabel("time[s]")
        plt.ylabel("angle_error[rad]")
        plt.xlim()
        plt.ylim()
        plt.legend()
        plt.show()




if __name__ == "__main__":
  rospy.init_node("analisys")
  Tracker = analisys()
  Tracker.main()
  Tracker.print()


