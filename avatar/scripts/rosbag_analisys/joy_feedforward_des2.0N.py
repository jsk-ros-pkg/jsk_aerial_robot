#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

fig = plt.figure()
filename = "/home/kaneko/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/avatar/rosbag/2024-01-07-15-06-29-joy-feedforward-des2.0N.bag"

# read from bag file
bag = rosbag.Bag(filename)
np_est_external_wrench = None
np_fc_t_min = None
np_angle = None
np_rpy_motors0_pitch_p = None
np_rpy_motors1_pitch_p = None
np_rpy_motors2_pitch_p = None
np_rpy_motors3_pitch_p = None
np_rpy_motors0_pitch_p_gain = None
np_rpy_motors1_pitch_p_gain = None
np_rpy_motors2_pitch_p_gain = None
np_rpy_motors3_pitch_p_gain = None


for topic, msg, t in bag.read_messages():
    if topic=="/hydrus_xi/filtered_est_external_wrench":
        np_array=np.array([[0.0, 0.0, 0.0]])
        np_array[0,0]=msg.wrench.force.y
        np_array[0,1]=t.secs
        np_array[0,2]=t.nsecs
        if np_est_external_wrench is None:
            np_est_external_wrench=np_array
        else:
            np_est_external_wrench=np.append(np_est_external_wrench,np_array,axis=0)
            
    if topic=="/hydrus_xi/fc_t_min":
        np_array=np.array([[0.0, 0.0, 0.0]])
        np_array[0,0]=msg.data
        np_array[0,1]=t.secs
        np_array[0,2]=t.nsecs
        if np_fc_t_min is None:
            np_fc_t_min=np_array
        else:
            np_fc_t_min=np.append(np_fc_t_min,np_array,axis=0)
            
    if topic=="/hydrus_xi/imu":
        np_array=np.array([[0.0, 0.0, 0.0, 0.0]])
        np_array[0,0]=msg.angles[0]
        np_array[0,1]=msg.angles[1]
        np_array[0,2]=t.secs
        np_array[0,3]=t.nsecs
        if np_angle is None:
            np_angle=np_array
        else:
            np_angle=np.append(np_angle,np_array,axis=0)

    if topic=="/hydrus_xi/rpy/pid":
        np_array=np.array([[0.0, 0.0, 0.0]])
        np_array[0,0]=msg.motors[0].pitch_p
        np_array[0,1]=t.secs
        np_array[0,2]=t.nsecs
        if np_rpy_motors0_pitch_p is None:
            np_rpy_motors0_pitch_p=np_array
        else:
            np_rpy_motors0_pitch_p=np.append(np_rpy_motors0_pitch_p,np_array,axis=0)


# reform time
start_sec_angle=np_angle[0,2]
start_nsec_angle=np_angle[0,3]
t_angle=np.zeros(np_angle.shape[0],dtype='float32')
for i in range(np_angle.shape[0]):
    t_angle[i]=(np_angle[i,2]-start_sec_angle)+(np_angle[i,3]-start_nsec_angle)/1000000000.0

start_sec_fct=np_fc_t_min[0,1]
start_nsec_fct=np_fc_t_min[0,2]
t_fct=np.zeros(np_fc_t_min.shape[0],dtype='float32')
for i in range(np_fc_t_min.shape[0]):
    t_fct[i]=(np_fc_t_min[i,1]-start_sec_angle)+(np_fc_t_min[i,2]-start_nsec_angle)/1000000000.0

start_sec_wrench=np_est_external_wrench[0,1]
start_nsec_wrench=np_est_external_wrench[0,2]
t_wrench=np.zeros(np_est_external_wrench.shape[0],dtype='float32')
for i in range(np_est_external_wrench.shape[0]):
    t_wrench[i]=(np_est_external_wrench[i,1]-start_sec_angle)+(np_est_external_wrench[i,2]-start_nsec_angle)/1000000000.0

start_sec_rpy_pid=np_rpy_motors0_pitch_p[0,1]
start_nsec_rpy_pid=np_rpy_motors0_pitch_p[0,2]
t_rpy_pid=np.zeros(np_rpy_motors0_pitch_p.shape[0],dtype='float32')
for i in range(np_rpy_motors0_pitch_p.shape[0]):
    t_rpy_pid[i]=(np_rpy_motors0_pitch_p[i,1]-start_sec_angle)+(np_rpy_motors0_pitch_p[i,2]-start_nsec_angle)/1000000000.0


# plot
plt.rcParams["font.size"] = 24
plt.rcParams["svg.fonttype"] = "none"

# plt.subplot(121)
plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("external_hand_force")
plt.plot(t_wrench, np_est_external_wrench[:,0], 'r', label="est_external_force")
plt.xlabel("time[s]")
plt.ylabel("est_external_force[N]")
# plt.xlim(90,190)
plt.xlim()
plt.ylim()
plt.legend()
# fig.savefig("joy_feedforward_des2.0N_wrench.svg")
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("fc_t_min")
plt.plot(t_fct, np_fc_t_min[:,0], 'r', label="fc_t_min")
plt.xlabel("time[s]")
plt.ylabel("fc_t_min[N]")
plt.xlim()
plt.ylim(4.2,4.8)
plt.legend()
# fig.savefig("joy_feedforward_des2.0N_fctmin.svg")
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("angle")
plt.plot(t_angle, np_angle[:,0], 'r', label="roll")
plt.plot(t_angle, np_angle[:,1], 'b', label="pitch")
plt.xlabel("time[s]")
plt.ylabel("angle[rad]")
plt.xlim()
plt.ylim()
plt.legend()
# fig.savefig("joy_feedforward_des2.0N_angle.svg")
plt.show()

plt.grid(color='k', linestyle=':',linewidth=0.3)
plt.title("angle")
plt.plot(t_rpy_pid, np_rpy_motors0_pitch_p[:,0], 'r', label="pitch")
plt.xlabel("time[s]")
plt.ylabel("angle[rad]")
plt.xlim(0,50)
plt.ylim()
plt.legend()
plt.show()

bag.close()
