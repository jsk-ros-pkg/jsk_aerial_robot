#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2024/01/14 12:21
# @Author : JIAXUAN LI
# @File : object_position_mapping.py
# @Software: PyCharm

import os
import sys

import time
import rospy
from functools import wraps
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry
from abc import ABC, abstractmethod
from geometry_msgs.msg import PoseStamped, Twist, Vector3, Quaternion, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from util import check_position_initialized, check_topic_subscription
from pub_mpc_joint_traj import MPCPubJointTraj

##########################################
# one-to-one mapping of all instantiation classes required.
##########################################

import rospy


class PositionObjectBase(ABC):
    def __init__(self, object_name: str, topic_name: str, msg_type):
        """
        Base class for subscribing to position information.
        :param topic_name: The name of the topic to subscribe to.
        :param msg_type: The message type for position data.
        """
        self.object_name = object_name
        self.position = None

        self.subscriber = self._sub_topic(topic_name, msg_type)

        check_position_initialized(self, "position", self.object_name)

    @check_topic_subscription
    def _sub_topic(self, topic_name, msg_type):
        rospy.loginfo(f"Subscribed to {topic_name}")
        return rospy.Subscriber(topic_name, msg_type, self._position_callback, queue_size=3)

    def _position_callback(self, msg):
        """
        Callback function to process incoming position messages.
        Should be implemented by subclasses.
        :param msg: The message containing position data.
        """
        self.position = msg

    def get_position(self):
        """
        Return position information from the message.
        :return: Extracted position data.
        """
        return self.position


class Hand(PositionObjectBase):
    def __init__(self):
        super().__init__(object_name="Hand", topic_name="/hand/mocap/pose", msg_type=PoseStamped)


class Arm(PositionObjectBase):
    def __init__(self):
        super().__init__(object_name="Arm", topic_name="/arm/mocap/pose", msg_type=PoseStamped)


class Drone(PositionObjectBase):
    def __init__(self, robot_name):
        super().__init__(object_name="Drone", topic_name=f"/{robot_name}/uav/cog/odom", msg_type=Odometry)


class Glove:
    def __init__(self):
        self.object_name = "Glove"
        self.control_mode = None
        self.control_mode_sub = self._sub_topic("/hand/control_mode", UInt8, self._glove_data_callback)

        self._check_data_initialized()

    @check_topic_subscription
    def _sub_topic(self, topic_name, msg_type, callback_func):
        rospy.loginfo(f"Subscribed to {topic_name}")
        return rospy.Subscriber(topic_name, msg_type, callback_func, queue_size=3)

    def _check_data_initialized(self):
        """
        Waits until the position is initialized. Logs a message repeatedly
        until a valid position is received.
        """
        while not rospy.is_shutdown() and self.control_mode is None:
            rospy.loginfo(f"Waiting for {self.object_name}'s data...")
            rospy.sleep(0.2)
        if self.control_mode is not None:
            rospy.loginfo(f"{self.object_name}'s data received for the first time")

    def _glove_data_callback(self, msg: UInt8):
        """
        Callback function to process incoming messages.
        Should be implemented by subclasses.
        :param msg: The message containing glove data.
        """
        self.control_mode = msg.data

    def get_control_mode(self):
        """
        Return position information from the message.
        :return: Extracted glove data.
        """
        return self.control_mode


##########################################
# Derived Class : OnetoOnePubJointTraj
##########################################
class OneToOnePubJointTraj(MPCPubJointTraj):
    def __init__(
            self,
            robot_name: str,
            hand: Hand,
            arm: Arm,
            control_mode: Glove,
    ):
        super().__init__(robot_name=robot_name, node_name="1to1map_traj_pub")
        self.hand = hand
        self.arm = arm
        self.control_mode = control_mode

        self.initial_hand_position = None
        self.initial_drone_position = None
        self.position_change = None

        self.is_finished = False
        # initialize vel_twist and acc_twist
        r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
        r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0
        self.vel_twist = Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate))
        self.acc_twist = Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc))

        self._check_last_time = None
        self._check_last_position = None
        self._check_check_orientation = None
        self._check_time_threshold = 5
        self._check_position_tolerance = 0.1
        self._check_orientation_tolerance = 6

        self.last_state = None

    def _check_finish_auto(self):
        current_time = rospy.Time.now().to_sec()
        if not hasattr(self, "last_check_time"):
            self.last_check_time = current_time
            self._check_last_position = [
                self.hand.position.pose.position.x,
                self.hand.position.pose.position.y,
                self.hand.position.pose.position.z,
            ]
            self._check_check_orientation = [
                self.hand.position.pose.orientation.x,
                self.hand.position.pose.orientation.y,
                self.hand.position.pose.orientation.z,
                self.hand.position.pose.orientation.w,
            ]
            return

        current_check_position = [
            self.hand.position.pose.position.x,
            self.hand.position.pose.position.y,
            self.hand.position.pose.position.z,
        ]
        current_check_orientation = [
            self.hand.position.pose.orientation.x,
            self.hand.position.pose.orientation.y,
            self.hand.position.pose.orientation.z,
            self.hand.position.pose.orientation.w,
        ]

        position_change = [abs(current_check_position[i] - self._check_last_position[i]) for i in range(3)]

        orientation_change = [abs(current_check_orientation[i] - self._check_check_orientation[i]) for i in range(4)]

        # self._check_last_position[:] = current_check_position
        # self._check_check_orientation[:] = current_check_orientation

        goal_reached = all(change < self._check_position_tolerance for change in position_change) and all(
            change < self._check_orientation_tolerance for change in orientation_change
        )

        new_state = "goal_reached" if goal_reached else "goal_not_reached"
        if new_state != self.last_state:
            rospy.loginfo("Now state: reach the goal" if goal_reached else "Now state:not reach the goal")
            self.last_state = new_state

        if goal_reached:
            if current_time - self.last_check_time > self._check_time_threshold:
                rospy.loginfo("Exit mapping mode")
                self.is_finished = True

        else:
            self.last_check_time = current_time
            self.last_hand_position = current_check_position
            self.last_hand_orientation = current_check_orientation

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:

        if self.initial_hand_position is None:
            self.initial_hand_position = [
                self.hand.position.pose.position.x,
                self.hand.position.pose.position.y,
                self.hand.position.pose.position.z,
            ]
            self.initial_drone_position = [
                self.uav_odom.pose.pose.position.x,
                self.uav_odom.pose.pose.position.y,
                self.hand.position.pose.position.z,
            ]
            rospy.loginfo(f"initial drone position is {self.initial_drone_position}")
            rospy.loginfo(f"initial hand position is {self.initial_hand_position}")

        current_position = [
            self.hand.position.pose.position.x,
            self.hand.position.pose.position.y,
            self.hand.position.pose.position.z,
        ]

        self.position_change = [current_position[i] - self.initial_hand_position[i] for i in range(3)]

        direction = [
            self.initial_drone_position[0] + self.position_change[0],
            self.initial_drone_position[1] + self.position_change[1],
            self.initial_drone_position[2] + self.position_change[2],
        ]

        hand_orientation = [
            self.hand.position.pose.orientation.x,
            self.hand.position.pose.orientation.y,
            self.hand.position.pose.orientation.z,
            self.hand.position.pose.orientation.w,
        ]

        multi_dof_joint_traj = MultiDOFJointTrajectory()
        t_has_started = rospy.Time.now().to_sec() - self.start_time

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(
                    translation=Vector3(*direction),
                    rotation=Quaternion(*hand_orientation),
                )
            )
            traj_pt.velocities.append(self.vel_twist)
            traj_pt.accelerations.append(self.acc_twist)

            t_pred = i * 0.1
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        return multi_dof_joint_traj

    def pub_trajectory_points(self, traj_msg: MultiDOFJointTrajectory):
        """Publish the MultiDOFJointTrajectory message."""
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = "map"
        self.pub_ref_traj.publish(traj_msg)

    def check_finished(self, t_elapsed=None):
        if self.control_mode:
            if self.control_mode.control_mode == 2:
                self.is_finished = True
        else:
            self._check_finish_auto()

        return self.is_finished
