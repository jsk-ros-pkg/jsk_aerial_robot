#!/usr/bin/env python

import rospy
from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

rospy.init_node("rotor_tilt_plot")

filepath = rospy.get_param("filepath", "../data/opt_0_1935_direction_sum_1.txt")

f = open(filepath)
line = f.read().split('\n')

rotor1_list = []
rotor2_list = []
rotor3_list = []
value_list = []

for i in range(len(line) - 2):
    line_i  = line[i].split(" ")
    rotor1_list.append(float(line_i[0]))
    rotor2_list.append(float(line_i[1]))
    rotor3_list.append(float(line_i[2]))
    value_list.append(float(line_i[3]))

fig = plt.figure()
ax = fig.add_subplot(1, 1, 1, projection='3d')
ax.set_title("FCTMin", size = 20)
ax.set_xlabel("rotor1", size = 14, color = "r")
ax.set_ylabel("rotor2", size = 14, color = "r")
ax.set_zlabel("rotor3", size = 14, color = "r")

cm = plt.cm.get_cmap('RdYlBu')
mappable = ax.scatter(rotor1_list, rotor2_list, rotor3_list, s=1, c=value_list)

fig.colorbar(mappable, ax=ax)

plt.show()
