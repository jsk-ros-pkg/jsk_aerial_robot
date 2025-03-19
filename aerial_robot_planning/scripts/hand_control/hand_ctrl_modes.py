#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2025/3/19 12:22
# @Author : Jason.LI
# @File : hand_ctrl_modes.py
# @Software: PyCharm

import math
import rospy

from geometry_msgs.msg import Twist, Vector3, Quaternion, Transform
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from pub_mpc_joint_traj import MPCPubJointTraj
from sub_pos_objects import Hand, Arm, Glove


##########################################
# Derived Class : HandControlBaseMode
##########################################
class HandControlBaseMode(MPCPubJointTraj):
    def __init__(self,
             robot_name: str,
             hand: Hand,
             arm: Arm,
             control_mode: Glove,
             node_name: str,
             ):
        super().__init__(robot_name=robot_name, node_name=node_name)
        self.hand = hand
        self.arm = arm
        self.control_mode = control_mode

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
        self._check_last_orientation = None
        self._check_time_threshold = 5
        self._check_position_tolerance = 0.1
        self._check_orientation_tolerance = 6

        self.to_return_control_mode = None
        self.last_state = None

    def _check_finish_auto(self):
        current_time = rospy.Time.now().to_sec()
        if not hasattr(self, "last_check_time"):
            self._last_check_time = current_time
            self._check_last_position = [
                self.hand.position.pose.position.x,
                self.hand.position.pose.position.y,
                self.hand.position.pose.position.z,
            ]
            self._check_last_orientation = [
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
        orientation_change = [abs(current_check_orientation[i] - self._check_last_orientation[i]) for i in range(4)]

        goal_reached = all(change < self._check_position_tolerance for change in position_change) and all(
            change < self._check_orientation_tolerance for change in orientation_change
        )

        new_state = "goal_reached" if goal_reached else "goal_not_reached"
        if new_state != self.last_state:
            rospy.loginfo("Now state: reach the goal" if goal_reached else "Now state:not reach the goal")
            self.last_state = new_state

        if goal_reached:
            if current_time - self._last_check_time > self._check_time_threshold:
                rospy.loginfo("Exit control")
                self.is_finished = True
        else:
            self._last_check_time = current_time

    def check_finished(self, t_elapsed=None):
        if self.control_mode:
            if self.control_mode.control_mode != self.mode_num:
                self.to_return_control_mode = self.control_mode.control_mode
                self.is_finished = True
        else:
            self._check_finish_auto()

        return self.is_finished

    def get_control_mode(self):
        return self.to_return_control_mode


##########################################
# Derived Class : MappingMode
##########################################
class MappingMode(HandControlBaseMode):
    def __init__(
            self,
            robot_name: str,
            hand: Hand,
            arm: Arm,
            control_mode: Glove,
    ):
        super().__init__(robot_name, hand, arm, control_mode, node_name="mapping_mode_traj_pub")

        self.initial_hand_position = None
        self.initial_drone_position = None
        self.position_hand_change = None

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

        current_hand_position = [
            self.hand.position.pose.position.x,
            self.hand.position.pose.position.y,
            self.hand.position.pose.position.z,
        ]

        self.position_hand_change = [current_hand_position[i] - self.initial_hand_position[i] for i in range(3)]

        target_position = [
            self.initial_drone_position[0] + self.position_hand_change[0],
            self.initial_drone_position[1] + self.position_hand_change[1],
            self.initial_drone_position[2] + self.position_hand_change[2],
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
                    translation=Vector3(*target_position),
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


##########################################
# Derived Class : SphericalMode
##########################################
class SphericalMode(HandControlBaseMode):
    def __init__(self, robot_name: str, hand: Hand, arm: Arm, control_mode: Glove):
        super().__init__(robot_name, hand, arm, control_mode, node_name="spherical_mode_traj_pub")
        self.expected_a_d_distance = 2.2

        self.mode_num = 3

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        a_h_direction = [
            self.hand.position.pose.position.x - self.arm.position.pose.position.x,
            self.hand.position.pose.position.y - self.arm.position.pose.position.y,
        ]
        a_h_distance = math.hypot(a_h_direction[0], a_h_direction[1])
        a_h_unit_vector = [a_h_direction[0] / a_h_distance, a_h_direction[1] / a_h_distance]

        hand_orientation = [
            self.hand.position.pose.orientation.x,
            self.hand.position.pose.orientation.y,
            self.hand.position.pose.orientation.z,
            self.hand.position.pose.orientation.w,
        ]

        if a_h_distance > 0.4:
            self.expected_a_d_distance = self.expected_a_d_distance + 0.01
        elif 0.4 > a_h_distance > 0.2:
            pass
        else:
            self.expected_a_d_distance = self.expected_a_d_distance - 0.01

        self.expected_a_d_distance = max(1, min(20, self.expected_a_d_distance))

        target_position = [
                              a_h_unit_vector[i] * self.expected_a_d_distance + getattr(
                                  self.hand.position.pose.position, axis)
                              for i, axis in enumerate(["x", "y"])
                          ] + [self.hand.position.pose.position.z]

        multi_dof_joint_traj = MultiDOFJointTrajectory()
        t_has_started = rospy.Time.now().to_sec() - self.start_time

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(
                    translation=Vector3(*target_position),
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


##########################################
# Derived Class : CartesianMode
##########################################
class CartesianMode(HandControlBaseMode):
    def __init__(self, robot_name: str, hand: Hand, arm: Arm, control_mode: Glove):
        super().__init__(robot_name, hand, arm, control_mode, node_name="cartesian_mode_traj_pub")
        self.expected_d_target_distance = 0.0
        self.last_target_position = None
        self.origin_position = None

        self.mode_num = 2

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        self.origin_position = [
            self.arm.position.pose.position.x + 0.3,
            self.arm.position.pose.position.y,
        ]

        current_hand_position = [
            self.hand.position.pose.position.x,
            self.hand.position.pose.position.y,
            self.hand.position.pose.position.z,
        ]
        hand_orientation = [
            self.hand.position.pose.orientation.x,
            self.hand.position.pose.orientation.y,
            self.hand.position.pose.orientation.z,
            self.hand.position.pose.orientation.w,
        ]

        o_h_direction = [
            current_hand_position[0] - self.origin_position[0],
            current_hand_position[1] - self.origin_position[1],
        ]
        o_h_distance = math.hypot(o_h_direction[0], o_h_direction[1])
        o_h_unit_vector = [o_h_direction[0] / o_h_distance, o_h_direction[1] / o_h_distance]

        if o_h_distance < 0.15:
            self.expected_d_target_distance = 0.0
        else:
            self.expected_d_target_distance = 0.2

        target_position = [
                              o_h_unit_vector[i] * self.expected_d_target_distance + getattr(
                                  self.uav_odom.pose.pose.position, axis)
                              for i, axis in enumerate(["x", "y"])
                          ] + [self.hand.position.pose.position.z]

        target_hand_distance = math.hypot(
            target_position[0] - current_hand_position[0], target_position[1] - current_hand_position[1]
        )

        if target_hand_distance < 1.5:
            target_position = [
                self.uav_odom.pose.pose.position.x,
                self.uav_odom.pose.pose.position.y,
                self.uav_odom.pose.pose.position.z,
            ]

        multi_dof_joint_traj = MultiDOFJointTrajectory()
        t_has_started = rospy.Time.now().to_sec() - self.start_time

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(
                    translation=Vector3(*target_position),
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


##########################################
# Derived Class : LockMode
##########################################
class LockMode(HandControlBaseMode):
    def __init__(self, robot_name: str, hand: Hand, arm: Arm, control_mode: Glove):
        super().__init__(robot_name, hand, arm, control_mode, node_name="lock_mode_traj_pub")
        self._init_origin_drone_position = None
        self._init_origin_hand_orientation = None

        self.mode_num = 4

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        if self._init_origin_drone_position is None:
            self._init_origin_drone_position = [
                self.uav_odom.pose.pose.position.x,
                self.uav_odom.pose.pose.position.y,
                self.uav_odom.pose.pose.position.z,
            ]
            self._init_origin_hand_orientation = [
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
                    translation=Vector3(*self._init_origin_drone_position),
                    rotation=Quaternion(*self._init_origin_hand_orientation),
                )
            )
            traj_pt.velocities.append(self.vel_twist)
            traj_pt.accelerations.append(self.acc_twist)

            t_pred = i * 0.1
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        return multi_dof_joint_traj
