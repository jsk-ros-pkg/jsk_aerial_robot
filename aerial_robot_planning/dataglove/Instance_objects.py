#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2024/01/14 12:21
# @Author : JIAXUAN LI
# @File : Instance_objects.py
# @Software: PyCharm

import sys
import rospy
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Quaternion, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from ..scripts.pub_mpc_joint_traj import MPCPubJointTraj


##########################################
# one-to-one mapping of all instantiation classes required.
##########################################


class HandPosition:
    def __init__(self, topic_name="/hand/mocap/pose"):
        self.hand_position = PoseStamped()
        self.topic_name = topic_name
        self.subscriber = rospy.Subscriber(
            self.topic_name, PoseStamped, self.hand_position_callback
        )
        rospy.loginfo(f"Subscribed to {self.topic_name}")

    def hand_position_callback(self, msg: PoseStamped):
        self.hand_position = msg

    def get_position(self):
        return self.hand_position

class ArmPosition:
    def __init__(self):
        self.arm_position = PoseStamped()
        self.arm_pose_sub = rospy.Subscriber(
            "/arm/mocap/pose", PoseStamped, self.arm_position_callback
        )
        rospy.loginfo("Subscribed to /arm/mocap/pose")

    def arm_position_callback(self, msg: PoseStamped):
        self.arm_position = msg

    def get_position(self):
        return self.arm_position

class DronePosition:
    def __init__(self, robot_name: str) -> None:
        self.robot_name = robot_name
        self.drone_position = Odometry()
        self.drone_pose_sub = rospy.Subscriber(
            f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback, queue_size=1
        )
        rospy.loginfo(f"Subscribed to {robot_name}/uav/cog/odom")

    def sub_odom_callback(self, msg: Odometry):
        self.drone_position = msg

    def get_position(self):
        return self.drone_position

class ControlMode:
    def __init__(self):
        self.control_mode = UInt8()

        self.control_mode_sub = rospy.Subscriber('hand/control_mode', UInt8, self.callback_control_mode)

    def callback_control_mode(self, data):
        self.control_mode = data.data

##########################################
# Derived Class : OnetoOnePubJointTraj
##########################################
class OnetoOnePubJointTraj(MPCPubJointTraj):
    def __init__(self, robot_name: str, hand: HandPosition,arm: ArmPosition,drone: DronePosition):
        super().__init__(robot_name=robot_name, node_name="1to1map_traj_pub")

        self.hand = hand

        self.arm = arm

        self.drone = drone

        # self.control_mode = ControlMode()

        self.initial_hand_position = None
        self.initial_drone_position = None
        self.position_change = None

        # initialize vel_twist and acc_twist
        r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
        r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0
        self.vel_twist = Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate))
        self.acc_twist = Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc))

        self.start_time = rospy.Time.now().to_sec()

        # Frequency
        self.ts_pt_pub = 0.02  # [s]  callback


    def _call_back_traj_pub(self,event):

        if self.initial_hand_position is None:
            self.initial_hand_position = [
                self.hand.hand_position.pose.position.x,
                self.hand.hand_position.pose.position.y,
                self.hand.hand_position.pose.position.z
            ]
            self.initial_drone_position = [
                self.drone.drone_position.pose.pose.position.x,
                self.drone.drone_position.pose.pose.position.y,
                self.drone.drone_position.pose.pose.position.z
            ]
        else :
            current_position = [
                self.hand.hand_position.pose.position.x,
                self.hand.hand_position.pose.position.y,
                self.hand.hand_position.pose.position.z
            ]

            self.position_change = [
                current_position[i] - self.initial_hand_position[i] for i in range(3)
            ]

            direction = [

                self.initial_drone_position[0] + self.position_change[0],
                self.initial_drone_position[1] + self.position_change[1],
                self.initial_drone_position[2] + self.position_change[2]
            ]

            hand_orientation = [self.hand.hand_position.pose.orientation.x,
                                self.hand.hand_position.pose.orientation.y,
                                self.hand.hand_position.pose.orientation.z,
                                self.hand.hand_position.pose.orientation.w]

            sys.stdout.write(
                f"\r目标地点 {direction[0]:6.1f}, {direction[1]:6.1f}, {direction[2]:6.1f} "
                f"目标姿势 {hand_orientation[0]:6.1f}, {hand_orientation[1]:6.1f}, {hand_orientation[2]:6.1f}, {hand_orientation[3]:6.1f} "

            )
            sys.stdout.flush()

            # 2.Path Calculation
            multi_dof_joint_traj = MultiDOFJointTrajectory()
            t_has_started = rospy.Time.now().to_sec() - self.start_time

            for i in range(self.N_nmpc + 1):
                traj_pt = MultiDOFJointTrajectoryPoint()
                traj_pt.transforms.append(
                    Transform(translation=Vector3(*direction), rotation=Quaternion(*hand_orientation)))
                traj_pt.velocities.append(self.vel_twist)
                traj_pt.accelerations.append(self.acc_twist)

                t_pred = i * 0.1
                t_cal = t_pred + t_has_started
                traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

                multi_dof_joint_traj.points.append(traj_pt)

            multi_dof_joint_traj.header.stamp = rospy.Time.now()
            multi_dof_joint_traj.header.frame_id = "map"

            self.pub_ref_traj.publish(multi_dof_joint_traj)