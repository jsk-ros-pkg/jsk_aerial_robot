#!/usr/bin/env python3
import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys
import os

args = sys.argv
print(len(args))
assert len(args)>=2, "you must specify the argument."

# get path
filename=os.path.normpath(os.path.join(os.getcwd(),args[1]))
print(filename)

# read from bag file
bag = rosbag.Bag(filename)
# np_poses=None
np_est_external_wrench = None

for topic, msg, t in bag.read_messages():
    # if topic=="/pose":
    #     np_pose=np.array([[0.0, 0.0, 0.0, 0.0, 0.0]])
    #     np_pose[0,0]=msg.position.x
    #     np_pose[0,1]=msg.position.y
    #     np_pose[0,2]=msg.position.z
    #     np_pose[0,3]=t.secs
    #     np_pose[0,4]=t.nsecs
    #     if np_poses is None:
    #         np_poses=np_pose
    #     else:
    #         np_poses=np.append(np_poses,np_pose,axis=0)
    # print(t.secs)
    # print(t.nsecs)
    if topic=="/hydrus_xi/filtered_est_external_wrench":
        np_array=np.array([[0.0, 0.0, 0.0]])
        np_array[0,0]=msg.wrench.force.y
        # np_array[0,1]=msg.header.seq
        np_array[0,1]=t.secs
        np_array[0,2]=t.nsecs
        if np_est_external_wrench is None:
            np_est_external_wrench=np_array
        else:
            np_est_external_wrench=np.append(np_est_external_wrench,np_array,axis=0)


# reform time
start_sec=np_est_external_wrench[0,1]
start_nsec=np_est_external_wrench[0,2]
t=np.zeros(np_est_external_wrench.shape[0],dtype='float32')
for i in range(np_est_external_wrench.shape[0]):
    t[i]=(np_est_external_wrench[i,1]-start_sec)+(np_est_external_wrench[i,2]-start_nsec)/1000000000.0

# plot  
plt.subplot(121)
plt.title("external_hand_force")
plt.plot(t, np_est_external_wrench[:,0], 'r', label="x")
plt.xlabel("time[s]")
plt.ylabel("est_external_force[N]")
plt.legend()
plt.show()

bag.close()
