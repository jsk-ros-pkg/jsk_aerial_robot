#!/usr/bin/env python

# rosrun delta joint_load_plot.py ~/.ros/filename

import argparse
import matplotlib.pyplot as plt
import sys
import numpy as np

class JointLoadPlot:
    def __init__(self, file_name):
        print(file_name)
        self.joint_torques = np.loadtxt(file_name)

        print(self.joint_torques)
        self.joint1_torque = self.joint_torques[:, 1]
        self.joint2_torque = self.joint_torques[:, 3]

        print(self.joint1_torque)
        print(self.joint2_torque)

        plt.scatter(self.joint1_torque, self.joint2_torque)
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_name", help="file name")

    args = parser.parse_args()

    JointLoadPlot(args.file_name)
