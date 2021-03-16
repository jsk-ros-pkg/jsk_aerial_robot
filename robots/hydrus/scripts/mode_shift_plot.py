#!/usr/bin/env python

import sys
import time
import rospy
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as colors
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
import pandas as pd
import seaborn as sns
from pylab import rcParams
import signal

def sig_handler(signum, frame):
    plt.close()
    sys.exit(0)

def main():
    rospy.init_node("hydrus_mode_shift_plot")

    link_num = rospy.get_param("~link_num", 8)

    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/hydrus/joint_states")
    mode_eigen_topic_name = rospy.get_param("~mode_eigen_topic_name", "/hydrus/torsion_eigens")
    mode_matrix_topic_name = rospy.get_param("~mode_matrix_topic_name", "/hydrus/torsion_mode_matrix")
    mode_matrix_data = rospy.wait_for_message(mode_matrix_topic_name, Float32MultiArray)
    mode_eigen_data = rospy.wait_for_message(mode_eigen_topic_name, Float32MultiArray)
    joint_state_data = rospy.wait_for_message(joint_state_topic_name, JointState)

    torsion_num = len(mode_matrix_data.data)/(link_num-1)
    print(mode_matrix_data)
    mode_matrix = np.array(mode_matrix_data.data).reshape(torsion_num, link_num-1)
    print("mode matrix")
    print(mode_matrix)

    kernel_matrix_topic_name = rospy.get_param("~kernel_matrix_topic_name", "/hydrus/B_eom_kernel")
    kernel_matrix_data = rospy.wait_for_message(kernel_matrix_topic_name, Float32MultiArray)
    kernel_matrix = np.array(kernel_matrix_data.data).reshape(link_num, len(kernel_matrix_data.data)/link_num)
    print("kernel matrix")
    print(kernel_matrix)

    k_gain_topic_name = rospy.get_param("~k_gain_topic_name", "/hydrus/K_gain_for_shift")
    k_gain_data = rospy.wait_for_message(k_gain_topic_name, Float32MultiArray)
    k_gain_matrix = np.array(k_gain_data.data).reshape(k_gain_data.layout.dim[0].size, k_gain_data.layout.dim[1].size)
    print("k gain matrix")
    print(k_gain_matrix)

    k_mode_topic_name = rospy.get_param("~k_mode_topic_name", "/hydrus/K_mode")
    k_mode_data = rospy.wait_for_message(k_mode_topic_name, Float32MultiArray)
    k_mode_matrix = np.array(k_mode_data.data).reshape(k_mode_data.layout.dim[0].size, k_mode_data.layout.dim[1].size)
    print("k mode matrix")
    print(k_mode_matrix)

    k_gain_shifted_topic_name = rospy.get_param("~k_gain_shifted_topic_name", "/hydrus/K_gain_null_space_shifted")
    k_gain_shifted_data = rospy.wait_for_message(k_gain_shifted_topic_name, Float32MultiArray)
    k_gain_shifted_matrix = np.array(k_gain_shifted_data.data).reshape(k_gain_shifted_data.layout.dim[0].size, k_gain_shifted_data.layout.dim[1].size)
    print("k gain shifted matrix")
    print(k_gain_shifted_matrix)

    kernel_mix_ratio_topic_name = rospy.get_param("~kernel_mix_ratio_topic_name", "/hydrus/kernel_mix_ratio")
    kernel_mix_ratio_data = rospy.wait_for_message(kernel_mix_ratio_topic_name, Float32MultiArray)
    kernel_mix_ratio = np.array(kernel_mix_ratio_data.data).reshape(kernel_mix_ratio_data.layout.dim[0].size, kernel_mix_ratio_data.layout.dim[1].size)

    thrust_x = np.zeros(link_num)
    thrust_y = np.zeros(link_num)
    joint_x = np.zeros(link_num-1)
    joint_y = np.zeros(link_num-1)
    link_lines = []
    link_length = rospy.get_param("~link_length", 0.6)
    link_direction = 0.0
    for i in range(link_num-1):
        joint_x[i] = thrust_x[i] + link_length/2 * np.cos(link_direction)
        joint_y[i] = thrust_y[i] + link_length/2 * np.sin(link_direction)

        link_direction = link_direction + joint_state_data.position[i]

        thrust_x[i+1] = joint_x[i] + link_length/2 * np.cos(link_direction)
        thrust_y[i+1] = joint_y[i] + link_length/2 * np.sin(link_direction)

        link_lines.append([[thrust_x[i], thrust_y[i]], [joint_x[i],joint_y[i]]])
        link_lines.append([[joint_x[i],joint_y[i]], [thrust_x[i+1], thrust_y[i+1]]])


    # figure 1
    fig_cm_name = 'seismic'
    fig, axs = plt.subplots(4, max(k_gain_matrix.shape[1], torsion_num, kernel_matrix.shape[1]), figsize=(15.0, 10.0))
    rows = ['LQI Gain', 'LQI Torsion Gain', 'Shifted Gain', 'Kernel Gain']
    for ax, row in zip(axs[:,0], rows):
        ax.set_title(row)
    for ax_row in axs:
        for ax in ax_row:
            ax.set_xlim(auto=True)
            ax.set_ylim(auto=True)
            ax.set_xmargin(0.6)
            ax.set_ymargin(0.6)
            ax.axis('off')

    ## k lqi
    j = 0
    if k_gain_matrix.shape[1] == 3:
        k_gain_titles = ["roll", "pitch", "yaw"]
    elif k_gain_matrix.shape[1] == 6:
        k_gain_titles = ["x", "y", "z", "roll", "pitch", "yaw"]
    for i in range(k_gain_matrix.shape[1]):
        ax = axs[0, j]
        j+=1
        gain = k_gain_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    ## k torsion
    j = 0
    for i in range(k_mode_matrix.shape[1]):
        ax = axs[1, j]
        j+=1
        gain = k_mode_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title("torsion" + str(i+1) + ", mag: " + '{:.3f}'.format(np.linalg.norm(gain)) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
        if i >= 12:
            mode = mode_matrix[(i-12)/2]
            ax.scatter(joint_x, joint_y, c=mode, s=200*np.power(abs(mode)/max(abs(mode)),1), marker='*',cmap='PRGn')
    ## k shifted
    j = 0
    for i in range(k_gain_shifted_matrix.shape[1]):
        ax = axs[2, j]
        j+=1
        gain = k_gain_shifted_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.set_title(k_gain_titles[i] + "\n" + np.array2string(kernel_mix_ratio[i], precision=2) )
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)
    ## kernel
    for i in range(kernel_matrix.shape[1]):
        ax = axs[3, i]
        gain = kernel_matrix[:,i]
        ax.axis('equal')
        ax.add_collection(LineCollection(link_lines, colors='black', linewidth=2))
        ax.scatter(thrust_x, thrust_y, c=gain, s=400*np.power(abs(gain)/max(abs(gain)),1), cmap=fig_cm_name)

    axpos = axs[0,0].get_position()
    cbar_ax = fig.add_axes([0.87, axpos.y0, 0.02, axpos.height])
    norm = colors.Normalize(vmin=-1,vmax=1)
    mappable = ScalarMappable(cmap=fig_cm_name, norm=norm)
    mappable._A = []
    cbar = fig.colorbar(mappable,cax=cbar_ax)
    # fig.tight_layout()
    plt.title("summary of gain shift \n"+str(np.array(joint_state_data.position)))

    # figure 2
    plt.figure(2)
    for i in range(torsion_num):
        mode_eigen = mode_eigen_data.data[i]
        mode_freq = np.sqrt(-mode_eigen) /2 /np.pi
        plt.plot(range(1,len(mode_matrix[i])+1), mode_matrix[i], label="mode "+str(i+1)+" , "+str(int(100*mode_freq)/100.0)+" Hz")
    plt.legend()
    np.set_printoptions(precision=2, floatmode='maxprec')
    #plt.title("shape of each mode: joint state \n"+str(np.array(joint_state_data.position)))
    plt.xlabel("torsion joint no.")
    plt.ylabel("magnitude of deformation in each mode")

    plt.show()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, sig_handler)
    sys.exit(main())
