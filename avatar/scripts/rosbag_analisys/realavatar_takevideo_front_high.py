#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os
import tf.transformations as tf


fig = plt.figure()
filename = "/home/kaneko/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/avatar/rosbag/2024-01-12-16-28-53-realavatar-takevideo-front-high.bag"

# read from bag file
bag = rosbag.Bag(filename)
np_uav_nav = None
np_uav_cog_odom = None
np_joints_ctrl = None
np_joint_states = None


for topic, msg, t in bag.read_messages():
    if topic=="/hydrus_xi/uav/nav":
        np_array=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np_array[0,0]=msg.target_pos_x
        np_array[0,1]=msg.target_pos_y
        np_array[0,2]=msg.target_pos_z
        np_array[0,3]=msg.target_yaw
        np_array[0,4]=t.secs
        np_array[0,5]=t.nsecs
        if np_uav_nav is None:
            np_uav_nav=np_array
        else:
            np_uav_nav=np.append(np_uav_nav,np_array,axis=0)
            
    if topic=="/hydrus_xi/uav/cog/odom":
        np_array=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np_array[0,0]=msg.pose.pose.position.x
        np_array[0,1]=msg.pose.pose.position.y
        np_array[0,2]=msg.pose.pose.position.z
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        euler = tf.euler_from_quaternion(quaternion)
        np_array[0,3]=euler[2]
        np_array[0,4]=t.secs
        np_array[0,5]=t.nsecs
        if np_uav_cog_odom is None:
            np_uav_cog_odom=np_array
        else:
            np_uav_cog_odom=np.append(np_uav_cog_odom,np_array,axis=0)
            
    if topic=="/hydrus_xi/joints_ctrl":
        np_array=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_array[0,0]=msg.position[0]
        np_array[0,1]=msg.position[1]
        np_array[0,2]=msg.position[2]
        np_array[0,3]=t.secs
        np_array[0,4]=t.nsecs
        if np_joints_ctrl is None:
            np_joints_ctrl=np_array
        else:
            np_joints_ctrl=np.append(np_joints_ctrl,np_array,axis=0)

    if topic=="/hydrus_xi/joint_states":
        np_array=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
        np_array[0,0]=msg.position[4]
        np_array[0,1]=msg.position[5]
        np_array[0,2]=msg.position[6]
        np_array[0,3]=t.secs
        np_array[0,4]=t.nsecs
        if np_joint_states is None:
            np_joint_states=np_array
        else:
            np_joint_states=np.append(np_joint_states,np_array,axis=0)


# reform time
start_sec_cog_odom=np_uav_cog_odom[0,4]
start_nsec_cog_odom=np_uav_cog_odom[0,5]
t_cog_odom=np.zeros(np_uav_cog_odom.shape[0],dtype='float32')
for i in range(np_uav_cog_odom.shape[0]):
    t_cog_odom[i]=(np_uav_cog_odom[i,4]-start_sec_cog_odom)+(np_uav_cog_odom[i,5]-start_nsec_cog_odom)/1000000000.0

start_sec_nav=np_uav_nav[0,4]
start_nsec_nav=np_uav_nav[0,5]
t_nav=np.zeros(np_uav_nav.shape[0],dtype='float32')
for i in range(np_uav_nav.shape[0]):
    t_nav[i]=(np_uav_nav[i,4]-start_sec_cog_odom)+(np_uav_nav[i,5]-start_nsec_cog_odom)/1000000000.0

start_sec_joint_states=np_joint_states[0,3]
start_nsec_joint_states=np_joint_states[0,4]
t_joint_states=np.zeros(np_joint_states.shape[0],dtype='float32')
for i in range(np_joint_states.shape[0]):
    t_joint_states[i]=(np_joint_states[i,3]-start_sec_joint_states)+(np_joint_states[i,4]-start_nsec_joint_states)/1000000000.0

start_sec_joints_ctrl=np_joints_ctrl[0,3]
start_nsec_joints_ctrl=np_joints_ctrl[0,4]
t_joints_ctrl=np.zeros(np_joints_ctrl.shape[0],dtype='float32')
for i in range(np_joints_ctrl.shape[0]):
    t_joints_ctrl[i]=(np_joints_ctrl[i,3]-start_sec_joint_states)+(np_joints_ctrl[i,4]-start_nsec_joint_states)/1000000000.0


# plot
plt.rcParams["font.size"] = 24
plt.rcParams["svg.fonttype"] = "none"

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("position x")
plt.plot(t_nav, np_uav_nav[:,0], 'r', label="des_position")
plt.plot(t_cog_odom, np_uav_cog_odom[:,0], 'b', label="robot_position")
plt.xlabel("time[s]")
plt.ylabel("position[m]")
plt.xlim(70,140)
plt.ylim()
plt.legend()
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("position y")
plt.plot(t_nav, np_uav_nav[:,1], 'r', label="des_position")
plt.plot(t_cog_odom, np_uav_cog_odom[:,1], 'b', label="robot_position")
plt.xlabel("time[s]")
plt.ylabel("position[m]")
plt.xlim(70,140)
plt.ylim()
plt.legend()
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("position z")
plt.plot(t_nav, np_uav_nav[:,2], 'r', label="des_position")
plt.plot(t_cog_odom, np_uav_cog_odom[:,2], 'b', label="robot_position")
plt.xlabel("time[s]")
plt.ylabel("position[m]")
plt.xlim(70,140)
plt.ylim()
plt.legend()
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("yaw angle")
plt.plot(t_nav, np_uav_nav[:,3], 'r', label="des_angle")
plt.plot(t_cog_odom, np_uav_cog_odom[:,3], 'b', label="robot_angle")
plt.xlabel("time[s]")
plt.ylabel("angle[rad]")
plt.xlim(70,140)
plt.ylim()
plt.legend()
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("joint1")
plt.plot(t_joints_ctrl, np_joints_ctrl[:,0], 'r', label="des_angle")
plt.plot(t_joint_states, np_joint_states[:,0], 'b', label="robot_angle")
plt.xlabel("time[s]")
plt.ylabel("angle[rad]")
plt.xlim(80,150)
plt.ylim()
plt.legend()
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("joint2")
plt.plot(t_joints_ctrl, np_joints_ctrl[:,1], 'r', label="des_angle")
plt.plot(t_joint_states, np_joint_states[:,1], 'b', label="robot_angle")
plt.xlabel("time[s]")
plt.ylabel("angle[rad]")
plt.xlim(80,150)
plt.ylim()
plt.legend()
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("joint3")
plt.plot(t_joints_ctrl, np_joints_ctrl[:,2], 'r', label="des_angle")
plt.plot(t_joint_states, np_joint_states[:,2], 'b', label="robot_angle")
plt.xlabel("time[s]")
plt.ylabel("angle[rad]")
plt.xlim(80,150)
plt.ylim()
plt.legend()
plt.show()

bag.close()
