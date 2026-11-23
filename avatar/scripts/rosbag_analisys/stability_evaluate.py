#!/usr/bin/env python

from tkinter import SE
import rospy
import numpy as np
import matplotlib.pyplot as plt
from spinal.msg import FourAxisCommand
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import PoseControlPid
from std_msgs.msg import Int8
from geometry_msgs.msg import WrenchStamped, PoseStamped
import tf
import tf2_ros
from tf.transformations import quaternion_matrix, quaternion_multiply, quaternion_inverse, quaternion_from_euler, euler_from_quaternion


class analisys():

    def __init__(self):

        self.start_time = 115.0 # fill in start time of rosbag
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.four_axes_sub = rospy.Subscriber('/hydrus_xi/four_axes/command', FourAxisCommand, self.four_axes_cb)
        self.uav_sub = rospy.Subscriber('/hydrus_xi/uav/cog/odom', Odometry, self.uav_cb)
        self.debug_errp_sub = rospy.Subscriber('/hydrus_xi/debug/pose/pid', PoseControlPid, self.debug_cb)
        self.hand_force_switch_sub = rospy.Subscriber('/hydrus_xi/hand_force_switch', Int8, self.hand_force_switch_cb)
        self.ft_sensor_sub = rospy.Subscriber('/filterd_ftsensor', WrenchStamped, self.ftsensor_cb)
        self.avatar_sub = rospy.Subscriber('/avatar_mocap_node/avatar/pose', PoseStamped, self.avatar_cb)   
        self.four_axes = [0,0,0]
        self.uav_euler = [0,0,0]
        self.pos_err_cog = np.array([0,0,0])
        self.yaw_err_cog = 0.0
        self.diff_array_roll = []
        self.diff_array_pitch = []
        self.diff_array_yaw = []
        self.time_array = []
        self.err_x_array = []
        self.err_y_array = []
        self.err_z_array = []
        self.err_yaw_array = []
        self.link1_end_err_x_array = []
        self.link1_end_err_y_array = []
        self.link1_end_err_z_array = []
        self.link1_end_err_yaw_array = []
        self.ftsensor_x_array = []
        self.ftsensor_y_array = []
        self.ftsensor_z_array = []
        self.debug_time_array = []
        self.ftsensor_time_array = []
        self.time = 0
        self.sec_0 = 0
        self.nsec_0 = 0
        self.debug_time = 0
        self.debug_sec_0 = 0
        self.debug_nsec_0 = 0
        self.ftsensor_time = 0
        self.ftsensor_sec_0 = 0
        self.ftsensor_nsec_0 = 0
        self.std_dev_roll = 0
        self.std_dev_pitch = 0
        self.std_dev_yaw = 0
        self.std_dev_x = 0
        self.std_dev_y = 0
        self.std_dev_z = 0
        self.std_dev_err_yaw = 0
        self.ave_x = 0
        self.ave_y = 0
        self.ave_z = 0
        self.ave_err_yaw = 0

        self.sec_init_flag = False
        self.debug_sec_init_flag = False
        self.ftsensor_sec_init_flag = False
        self.hand_force_switch_flag = False
        self.avatar_flag = False

    def transform_position(self, position, translation, rotation_quaternion):
        rotation_matrix = quaternion_matrix(rotation_quaternion)[:3, :3]
        transformed_position = np.dot(rotation_matrix, position)
        return transformed_position
    
    def transform_orientation(self, yaw, rotation_quataernion):
        q_cog = quaternion_from_euler(0.0, 0.0, yaw)
        q_link1_end_inv = quaternion_inverse(rotation_quataernion)
        q_transformed = quaternion_multiply(q_link1_end_inv, q_cog)
        transformed_orientation = euler_from_quaternion(q_transformed)
        return transformed_orientation
    
    def transform_link1_end_to_world(self, position):
        transform = self.tfBuffer.lookup_transform('hydrus_xi/link1_end', 'world', rospy.Time(0))
        trans_x = transform.transform.translation.x
        trans_y = transform.transform.translation.y
        trans_z = transform.transform.translation.z
        trans = np.array([trans_x, trans_y, trans_z])
        rot_x = transform.transform.rotation.x
        rot_y = transform.transform.rotation.y
        rot_z = transform.transform.rotation.z
        rot_w = transform.transform.rotation.w
        rot_q = np.array([rot_x, rot_y, rot_z, rot_w])

        return self.transform_position(position, trans, rot_q)


    def calculate_link1_end_err(self):
        # try:
            transform = self.tfBuffer.lookup_transform('hydrus_xi/cog', 'hydrus_xi/link1_end', rospy.Time(0))
            trans_x = transform.transform.translation.x
            trans_y = transform.transform.translation.y
            trans_z = transform.transform.translation.z
            trans = np.array([trans_x, trans_y, trans_z])
            rot_x = transform.transform.rotation.x
            rot_y = transform.transform.rotation.y
            rot_z = transform.transform.rotation.z
            rot_w = transform.transform.rotation.w
            rot_q = np.array([rot_x, rot_y, rot_z, rot_w])
            
            link1_end_pos_err = self.transform_position(self.pos_err_cog, trans, rot_q)
            link1_end_orientation_err = self.transform_orientation(self.yaw_err_cog, rot_q)
            link1_end_pos_err_world = self.transform_link1_end_to_world(link1_end_pos_err)
            self.link1_end_err_x_array.append(link1_end_pos_err_world[0])
            self.link1_end_err_y_array.append(link1_end_pos_err_world[1])
            self.link1_end_err_z_array.append(link1_end_pos_err_world[2])
            self.link1_end_err_yaw_array.append(link1_end_orientation_err[2])

            # print(link1_end_pos_err)

        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     continue
        

    def uav_cb(self,msg):
        uav_quaternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w] 
        self.uav_euler = list(tf.transformations.euler_from_quaternion(uav_quaternion))
        # print("uav=", self.uav_euler)
        # seq = msg.header.seq
        # if self.seq_init_flag == False:
        #     self.seq_0 = seq
        #     self.seq_init_flag = True
        # self.time = (seq-self.seq_0)/100 + self.start_time
        sec = msg.header.stamp.secs
        nsec = msg.header.stamp.nsecs
        if self.sec_init_flag == False:
            self.sec_0 = sec
            self.nsec_0 = nsec
            self.sec_init_flag = True
        self.time = (sec-self.sec_0) + (nsec-self.nsec_0)/1000000000 + self.start_time
        self.time_array.append(self.time)


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
        if self.hand_force_switch_flag == True:
            self.err_x_array.append(msg.x.err_p)
            self.err_y_array.append(msg.y.err_p)
            self.err_z_array.append(msg.z.err_p)
            self.err_yaw_array.append(msg.yaw.err_p)
            self.pos_err_cog = np.array([msg.x.err_p,msg.y.err_p,msg.z.err_p])
            self.yaw_err_cog = msg.yaw.err_p
        if self.hand_force_switch_flag == False:
            self.err_x_array.append(0.0)
            self.err_y_array.append(0.0)
            self.err_z_array.append(0.0)
            self.err_yaw_array.append(0.0)
            self.pos_err_cog = np.array([0.0,0.0,0.0])
            self.yaw_err_cog = 0.0
        # if self.avatar_flag == True:
        #     self.err_x_array.append(msg.x.err_p)
        #     self.err_y_array.append(msg.y.err_p)
        #     self.err_z_array.append(msg.z.err_p)
        #     self.err_yaw_array.append(msg.yaw.err_p)
        #     self.pos_err_cog = np.array([msg.x.err_p,msg.y.err_p,msg.z.err_p])
        #     self.yaw_err_cog = msg.yaw.err_p
        # if self.avatar_flag == False:
        #     self.err_x_array.append(0.0)
        #     self.err_y_array.append(0.0)
        #     self.err_z_array.append(0.0)
        #     self.err_yaw_array.append(0.0)
        #     self.pos_err_cog = np.array([0.0,0.0,0.0])
        #     self.yaw_err_cog = 0.0

        sec = msg.header.stamp.secs
        nsec = msg.header.stamp.nsecs
        if self.debug_sec_init_flag == False:
            self.debug_sec_0 = sec
            self.debug_nsec_0 = nsec
            self.debug_sec_init_flag = True
        self.debug_time = (sec-self.debug_sec_0) + (nsec-self.debug_nsec_0)/1000000000 + self.start_time
        self.debug_time_array.append(self.debug_time)
        self.std_dev_x = np.std(self.err_x_array)
        self.std_dev_y = np.std(self.err_y_array)
        self.std_dev_z = np.std(self.err_z_array)
        self.std_dev_error_yaw = np.std(self.err_yaw_array)
        self.ave_x = np.mean(self.err_x_array)
        self.ave_y = np.mean(self.err_y_array)
        self.ave_z = np.mean(self.err_z_array)
        self.ave_err_yaw = np.mean(self.err_yaw_array)

        self.calculate_link1_end_err()


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


    def ftsensor_cb(self,msg):
        force = [msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z]
        force_world = self.transform_link1_end_to_world(force)
        # self.ftsensor_x_array.append(msg.wrench.force.x)
        # self.ftsensor_y_array.append(msg.wrench.force.y)
        # self.ftsensor_z_array.append(msg.wrench.force.z)
        self.ftsensor_x_array.append(force_world[0])
        self.ftsensor_y_array.append(force_world[1])
        self.ftsensor_z_array.append(force_world[2])

        sec = msg.header.stamp.secs
        nsec = msg.header.stamp.nsecs
        if self.ftsensor_sec_init_flag == False:
            self.ftsensor_sec_0 = sec
            self.ftsensor_nsec_0 = nsec
            self.ftsensor_sec_init_flag = True
        self.ftsensor_time = (sec-self.ftsensor_sec_0) + (nsec-self.ftsensor_nsec_0)/1000000000 + self.start_time
        self.ftsensor_time_array.append(self.ftsensor_time)

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
        print("std ave of x = ", self.ave_x)
        print("std ave of y = ", self.ave_y)
        print("std ave of z = ", self.ave_z)
        print("std ave of error yaw = ", self.ave_err_yaw)



        # plot
        plt.rcParams["font.size"] = 24
        plt.rcParams["svg.fonttype"] = "none"

        # plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("attitude_diff")
        # plt.plot(self.time_array, self.diff_array_roll, 'r', label="roll_diff")
        # plt.plot(self.time_array, self.diff_array_pitch, 'b', label="pitch_diff")
        # plt.plot(self.time_array, self.diff_array_yaw, 'g', label="yaw_diff")
        # plt.xlabel("time[s]")
        # plt.ylabel("angle_diff[rad]")
        # plt.xlim()
        # plt.ylim()
        # plt.legend()
        # plt.show()

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

        # del self.debug_time_array[0]
        # del self.debug_time_array[1]
        plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("link1_end_pos_err")
        plt.plot(self.debug_time_array, self.link1_end_err_x_array, 'r', label="x_err")
        plt.plot(self.debug_time_array, self.link1_end_err_y_array, 'b', label="y_err")
        plt.plot(self.debug_time_array, self.link1_end_err_z_array, 'g', label="z_err")
        # plt.plot(self.debug_time_array, self.link1_end_err_yaw_array, 'b', label="yaw_err")
        plt.xlabel("time[s]")
        plt.ylabel("position_error[m]")
        plt.xlim()
        plt.ylim()
        plt.legend()
        plt.show()

        plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("ftsensor_val")
        # plt.plot(self.ftsensor_time_array, self.ftsensor_x_array, 'r', label="force_x")
        plt.plot(self.ftsensor_time_array, self.ftsensor_y_array, 'b', label="force_y")
        # plt.plot(self.ftsensor_time_array, self.ftsensor_z_array, 'g', label="force_z")
        plt.xlabel("time[s]")
        plt.ylabel("force[N]")
        plt.xlim()
        plt.ylim()
        plt.legend()
        plt.show()

        plt.grid(color='k', linestyle=':',linewidth=0.3)
        plt.title("pos_err")
        plt.plot(self.debug_time_array, self.err_x_array, 'r', label="x_err")
        # plt.plot(self.debug_time_array, self.err_y_array, 'b', label="y_err")
        plt.plot(self.debug_time_array, self.err_z_array, 'g', label="z_err")
        # plt.plot(self.debug_time_array, self.err_yaw_array, 'b', label="yaw_err")
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
                
        # plt.grid(color='k', linestyle=':',linewidth=0.3)
        # plt.title("yaw_err")
        # plt.plot(self.debug_time_array, self.err_yaw_array, 'r', label="yaw_err")
        # plt.xlabel("time[s]")
        # plt.ylabel("angle_error[rad]")
        # plt.xlim()
        # plt.ylim()
        # plt.legend()
        # plt.show()




if __name__ == "__main__":
  rospy.init_node("analisys")
  Tracker = analisys()
  Tracker.main()
  Tracker.print()


