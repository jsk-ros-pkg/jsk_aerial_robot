#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy

from aerial_robot_base.robot_interface import RobotInterface
from aerial_robot_base.state_machine import *
from geometry_msgs.msg import PoseStamped, Wrench, Vector3, Vector3Stamped, WrenchStamped, Quaternion, QuaternionStamped

from geometry_msgs.msg import Quaternion
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf.transformations import quaternion_inverse, quaternion_multiply, euler_from_quaternion, random_quaternion
import numpy as np

class TailState():
    def __init__(self):
        self.ri = RobotInterface()
        rospy.sleep(1.0) # wait for joint updated

        self.update_hz = 50

        self.angles = [0.0]

        joint0_sub = Subscriber("/joint0/mocap/pose", PoseStamped)
        # joint1_sub = rospy.Subscriber("/joint1/mocap/pose", PoseStamped)
        joint4_sub = Subscriber("/joint4/mocap/pose", PoseStamped)
        # angle_subs = [joint0_sub, joint1_sub]
        # tss = ApproximateTimeSynchronizer(angle_subs, queue_size=10, slop=0.01)
        # tss.registerCallback(self.update_angle)

        angle_subs = [joint0_sub, joint4_sub]
        tss = ApproximateTimeSynchronizer(angle_subs, queue_size=10, slop=0.01)
        tss.registerCallback(self.get_end_effector_local_pos)

    def update_angle(self, msg0, msg1):
        # print("Received messages for angles:")
        # print(f"Joint 0: {msg0.pose.orientation}")
        # print(f"Joint 1: {msg1.pose.orientation}")
        # print(type(random_quaternion()))
        # print(type(msg0.pose.orientation))
        quaternions = [
            np.array([msg0.pose.orientation.x, msg0.pose.orientation.y, msg0.pose.orientation.z, msg0.pose.orientation.w]),
            np.array([msg1.pose.orientation.x, msg1.pose.orientation.y, msg1.pose.orientation.z, msg1.pose.orientation.w]),
        ]
        for q in quaternions:
            print("euler angles: ", euler_from_quaternion(q, axes='sxyz'))
        for i in range(len(self.angles)):
            q1_inv = quaternion_inverse(quaternions[i])
            q2 = quaternions[i + 1]
            q_rel = quaternion_multiply(q2, q1_inv)
            angle = euler_from_quaternion(q_rel, axes='sxyz')
            # axes = sにすることで、回転順番によらないと思っている
            print("x: ", math.degrees(angle[0]))
            print("y: ", math.degrees(angle[1]))
            print("z: ", math.degrees(angle[2]))
            print("----")
            self.angles[i] = angle[2]

    def get_end_effector_local_pos(self, msg0, msg4):
        # print("Received messages for end effector position:")
        # print(f"Joint 0: {msg0.pose.position}")
        # print(f"Joint 4: {msg4.pose.position}")
        base_world_pos = np.array([msg0.pose.position.x, msg0.pose.position.y, msg0.pose.position.z])
        base_quaternion = np.array([msg0.pose.orientation.x, msg0.pose.orientation.y, msg0.pose.orientation.z, msg0.pose.orientation.w])
        end_effector_world_pos = np.array([msg4.pose.position.x, msg4.pose.position.y, msg4.pose.position.z])
        end_effector_quaternion = np.array([msg4.pose.orientation.x, msg4.pose.orientation.y, msg4.pose.orientation.z, msg4.pose.orientation.w])
        # delta = end_effector_world_pos - base_world_pos
        # delta = np.append(delta, 0.0) 
        # rot = quaternion_inverse(base_quaternion)
        # local_pos = quaternion_multiply(rot, delta)
        # print("Local Position: ", local_pos)
        # print("Base Quaternion: ", base_quaternion)
        # print("End Effector Quaternion: ", end_effector_quaternion)
        # return local_pos

        delta = end_effector_world_pos - base_world_pos

        base_quat_inv = quaternion_inverse(base_quaternion)
        delta_quat = np.concatenate([delta, [0.0]])
        rotated = quaternion_multiply(
            quaternion_multiply(base_quat_inv, delta_quat),
            base_quaternion
        )
        local_pos = rotated[:3]
        print("Local Position: ", local_pos)

    def run(self):
        r = rospy.Rate(self.update_hz)

        while not rospy.is_shutdown():
            # user code begin
            # user code end
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("mocap_to_angle")
    node = TailState()
    node.run()