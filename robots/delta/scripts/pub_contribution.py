#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

from std_msgs.msg import Float64MultiArray, Float32MultiArray
from aerial_robot_msgs.msg import PoseControlPid
from spinal.msg import FourAxisCommand
import numpy as np

class PubContributionNode:
    def __init__(self):
        self.q_matrix_sub = rospy.Subscriber("q_matrix", Float32MultiArray, self.q_matrix_cb, queue_size=1)
        self.base_thrust = None
        self.timestamp = None
        self.roll_term = None
        self.pitch_term = None
        self.base_thrust_sub = rospy.Subscriber("four_axes/command", FourAxisCommand, self.base_thrust_cb, queue_size=1)
        self.roll_pitch_term_sub = rospy.Subscriber("debug/pose/pid", PoseControlPid, self.roll_pitch_term_cb, queue_size=1)
        self.roll_contribution_pub = rospy.Publisher("thrust_contribution_to_roll", Float32MultiArray, queue_size=1)
        self.pitch_contribution_pub = rospy.Publisher("thrust_contribution_to_pitch", Float32MultiArray, queue_size=1)

    def base_thrust_cb(self, msg):
        self.base_thrust = msg.base_thrust
    
    def roll_pitch_term_cb(self, msg):
        self.timestamp = msg.header.stamp
        self.roll_term = msg.roll.total[0]
        self.pitch_term = msg.pitch.total[0]
    
    def q_matrix_cb(self, msg):
        if self.base_thrust is None:
            print("Base thrust not received yet.")
            return

        q_flat = np.asarray(msg.data, dtype=np.float32)
        q_mat = q_flat.reshape((msg.layout.dim[0].size, msg.layout.dim[1].size))
        print("Received Q matrix:")
        print(q_mat)
        # q_mat_roll = [q_mat[5], q_mat[6], q_mat[7], q_mat[8], q_mat[9]]
        # q_mat_pitch = [q_mat[10], q_mat[11], q_mat[12], q_mat[13], q_mat[14]]

        # roll_contribution = [q * t for q, t in zip(q_mat_roll, self.base_thrust)]
        # pitch_contribution = [q * t for q, t in zip(q_mat_pitch, self.base_thrust)]

        # roll_contribution_msg = Float32MultiArray()
        # pitch_contribution_msg = Float32MultiArray()

        # roll_contribution_msg.data = roll_contribution
        # pitch_contribution_msg.data = pitch_contribution
        # self.roll_contribution_pub.publish(roll_contribution_msg)
        # self.pitch_contribution_pub.publish(pitch_contribution_msg)
        # print("Published thrust contributions to roll and pitch.")

        q_mat_inv = np.linalg.pinv(q_mat)
        
        roll_idx = 1
        pitch_idx = 2

        lambda_for_roll  = q_mat_inv[:, roll_idx]  * float(self.roll_term)   # shape: (n_thrusters,)
        lambda_for_pitch = q_mat_inv[:, pitch_idx] * float(self.pitch_term)  # shape: (n_thrusters,)

        roll_contributions  = q_mat[roll_idx, :]  * lambda_for_roll          # shape: (n_thrusters,)
        pitch_contributions = q_mat[pitch_idx, :] * lambda_for_pitch         # shape: (n_thrusters,)

        roll_contribution_msg = Float32MultiArray()
        pitch_contribution_msg = Float32MultiArray()
        roll_contribution_msg.data = roll_contributions.astype(np.float32).tolist()
        pitch_contribution_msg.data = pitch_contributions.astype(np.float32).tolist()
        self.roll_contribution_pub.publish(roll_contribution_msg)
        self.pitch_contribution_pub.publish(pitch_contribution_msg)
        print("Published thrust contributions to roll and pitch.")

def main():
    rospy.init_node("pub_contribution_node")
    PubContributionNode()
    rospy.spin()

if __name__ == "__main__":
    main()
