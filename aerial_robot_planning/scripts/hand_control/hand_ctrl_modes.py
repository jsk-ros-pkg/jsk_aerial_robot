#!/usr/bin/env python
# -*- coding: utf-8 -*-
# @Time : 2025/3/19 12:22
# @Author : Jason.LI
# @File : hand_ctrl_modes.py
# @Software: PyCharm

import math
from abc import ABC, abstractmethod
from tf_conversions import transformations as tf
import numpy as np

import rospy
from geometry_msgs.msg import Twist, Vector3, Quaternion, Transform
from pub_mpc_joint_traj import MPCPubJointTraj
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from sub_pos_objects import HandPose, ArmPose, Glove


##########################################
# Derived Class : HandControlBaseMode
##########################################
"""
For example: From arm to hand, denoted as a2h; from hand to drone, denoted as h2d.
"""


class HandControlBaseMode(MPCPubJointTraj, ABC):
    def __init__(
        self,
        robot_name: str,
        hand_pose: HandPose,
        arm_pose: ArmPose,
        glove: Glove,
        node_name: str,
    ):
        super().__init__(robot_name=robot_name, node_name=node_name, is_calc_rmse=False)
        self.hand_pose = hand_pose
        self.arm_pose = arm_pose
        self.glove = glove

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

        self.mode_num = self._init_mode_num()

        self.start_timer()

    @staticmethod
    @abstractmethod
    def _init_mode_num():
        return int()

    def _check_finish_auto(self):
        """
        When the hand maintains its pose for a certain time, it exits the control.
        """
        current_time = rospy.Time.now().to_sec()
        if not hasattr(self, "last_check_time"):
            self._last_check_time = current_time
            self._check_last_position = [
                self.hand_pose.pose_msg.pose.position.x,
                self.hand_pose.pose_msg.pose.position.y,
                self.hand_pose.pose_msg.pose.position.z,
            ]
            self._check_last_orientation = [
                self.hand_pose.pose_msg.pose.orientation.x,
                self.hand_pose.pose_msg.pose.orientation.y,
                self.hand_pose.pose_msg.pose.orientation.z,
                self.hand_pose.pose_msg.pose.orientation.w,
            ]
            return

        current_check_position = [
            self.hand_pose.pose_msg.pose.position.x,
            self.hand_pose.pose_msg.pose.position.y,
            self.hand_pose.pose_msg.pose.position.z,
        ]
        current_check_orientation = [
            self.hand_pose.pose_msg.pose.orientation.x,
            self.hand_pose.pose_msg.pose.orientation.y,
            self.hand_pose.pose_msg.pose.orientation.z,
            self.hand_pose.pose_msg.pose.orientation.w,
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
        """
        Checks if the task is finished. If the glove is not enabled, uses _check_finish_auto
        to check if the task is completed.
        """
        if self.glove is None:
            self._check_finish_auto()
            return self.is_finished

        current_ctrl_mode = self.glove.get_control_mode()
        if self.mode_num != current_ctrl_mode:  # if the control mode is changed by the glove
            self.to_return_control_mode = current_ctrl_mode
            self.is_finished = True

        return self.is_finished

    def get_control_mode(self):
        return self.to_return_control_mode


##########################################
# Derived Class : OperationMode
##########################################
class OperationMode(HandControlBaseMode):
    def __init__(
        self,
        robot_name: str,
        hand_pose: HandPose,
        arm_pose: ArmPose,
        glove: Glove,
    ):
        super().__init__(robot_name, hand_pose, arm_pose, glove, node_name="operation_mode_traj_pub")

        self.initial_hand_position = None
        self.initial_drone_position = None
        self.position_hand_change = None

        # for continuous rotation
        self.cont_rot_gen = ContRotationGen()

    @staticmethod
    def _init_mode_num():
        return 1

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        """
        When entering this state, record the first pose of the drone and hand.
        Check the changes in the hand's pose compared to the first recorded pose.
        Add the change in the hand's pose to the initial pose recorded by the drone to determine the target pose for the drone at the next moment.
        """
        if self.initial_hand_position is None:
            self.initial_hand_position = [
                self.hand_pose.pose_msg.pose.position.x,
                self.hand_pose.pose_msg.pose.position.y,
                self.hand_pose.pose_msg.pose.position.z,
            ]
            self.initial_drone_position = [
                self.uav_odom.pose.pose.position.x,
                self.uav_odom.pose.pose.position.y,
                self.hand_pose.pose_msg.pose.position.z,
            ]
            rospy.loginfo(f"initial drone position is {self.initial_drone_position}")
            rospy.loginfo(f"initial hand position is {self.initial_hand_position}")

        current_hand_position = [
            self.hand_pose.pose_msg.pose.position.x,
            self.hand_pose.pose_msg.pose.position.y,
            self.hand_pose.pose_msg.pose.position.z,
        ]

        self.position_hand_change = [current_hand_position[i] - self.initial_hand_position[i] for i in range(3)]

        target_position = [
            self.initial_drone_position[0] + self.position_hand_change[0],
            self.initial_drone_position[1] + self.position_hand_change[1],
            self.initial_drone_position[2] + self.position_hand_change[2],
        ]

        # === detect continuous rotation ===  TODO: add a parameter to enable/disable this feature
        hand_quat = self.hand_pose.pose_msg.pose.orientation
        robot_quat = self.uav_odom.pose.pose.orientation
        hand_ori_proc, self.vel_twist = self.cont_rot_gen.update(hand_quat, robot_quat, self.vel_twist)
        # ==================================

        multi_dof_joint_traj = MultiDOFJointTrajectory()
        t_has_started = rospy.Time.now().to_sec() - self.start_time

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(
                    translation=Vector3(*target_position),
                    rotation=hand_ori_proc,
                )
            )
            traj_pt.velocities.append(self.vel_twist)
            traj_pt.accelerations.append(self.acc_twist)

            t_pred = i * 0.1
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        return multi_dof_joint_traj


class ContRotationGen:
    """
    Note: the quat_start should be generated from RPY.
    But the second condition should use quaternion rotation axis and angle.
    """

    def __init__(self, roll_start=0.0, pitch_start=np.pi / 2.0, yaw_start=0.0, axes="rzyx"):  # pitch = 90 degrees
        self.cont_rot_start_t = None
        self.cont_rot_dir = None  # +1 for right rotation, -1 for left rotation
        self.cont_rot_start_theta = None

        self.cont_rot_period = 30.0  # seconds

        q_list = tf.quaternion_from_euler(roll_start, pitch_start, yaw_start, axes=axes)
        self.quat_start = Quaternion(*q_list)

    def is_rotating(self) -> bool:
        return self.cont_rot_start_t is not None

    def reset(self):
        self.cont_rot_start_t = None
        self.cont_rot_dir = None
        self.cont_rot_start_theta = None

    @staticmethod
    def get_quat_error(quat: Quaternion, quat_ref: Quaternion):
        # error = x - x_r = q_r^* @ q
        quat_error = tf.quaternion_multiply(
            tf.quaternion_conjugate([quat_ref.x, quat_ref.y, quat_ref.z, quat_ref.w]), [quat.x, quat.y, quat.z, quat.w]
        )

        qe_w = quat_error[3]
        qe_vec = quat_error[:3]

        return np.sign(qe_w) * np.array(qe_vec)

    def update(self, hand_quat: Quaternion, robot_quat: Quaternion, vel_twist: Twist) -> (Quaternion, Twist):
        qe_vec = self.get_quat_error(hand_quat, self.quat_start)
        qe_norm = np.linalg.norm(qe_vec)

        qe_axis = qe_vec / qe_norm
        qe_angle = 2 * np.arccos(qe_norm)  # angle in radians

        if not self.is_rotating():
            # Entry
            if np.linalg.norm(qe_axis - np.array([-1.0, 0.0, 0.0])) < 0.2:  # close to vertical
                rospy.loginfo_throttle(1, "Enter vertical mode: rotate {:.2f} degrees".format(np.rad2deg(qe_angle)))

                if abs(qe_angle) > np.deg2rad(60):
                    self.cont_rot_start_t = rospy.Time.now().to_sec()
                    self.cont_rot_start_theta = qe_angle
                    self.cont_rot_dir = 1 if qe_angle > 0 else -1

                    direction = "right" if qe_angle > 0 else "left"
                    rospy.loginfo(f"Enter vertical mode: {direction} rotation")

            return hand_quat, vel_twist

        # Continuous rotation
        rotate_t = rospy.Time.now().to_sec() - self.cont_rot_start_t

        # Exit
        qe_r2h = self.get_quat_error(robot_quat, hand_quat)
        qe_r2h_angle = 2 * np.arccos(np.linalg.norm(qe_r2h))  # angle in radians
        cond_1 = qe_r2h_angle < np.deg2rad(10)  # robot is close to hand
        cond_2 = rotate_t > self.cont_rot_period  # seconds
        if (cond_1 or cond_2) and rotate_t > 5.0:
            rospy.loginfo("Exit vertical mode")
            self.reset()
            return hand_quat, vel_twist

        # Stay
        omega = 2 * np.pi / self.cont_rot_period * self.cont_rot_dir
        target_quat = tf.quaternion_from_euler(
            0.0, np.pi / 2.0, omega * rotate_t + self.cont_rot_start_theta, axes="rxyz"
        )

        hand_ori = Quaternion(*target_quat)
        vel_twist.angular.z = omega

        return hand_ori, vel_twist


##########################################
# Derived Class : SphericalMode
##########################################
class SphericalMode(HandControlBaseMode):
    def __init__(self, robot_name: str, hand_pose: HandPose, arm_pose: ArmPose, glove: Glove):
        super().__init__(robot_name, hand_pose, arm_pose, glove, node_name="spherical_mode_traj_pub")
        self.expected_a2d_distance = 2.2

    @staticmethod
    def _init_mode_num():
        return 3

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        """
        The shoulder as the origin of the polar coordinate system.
        Calculate the directional vector from the shoulder to the hand as the direction of the polar axis.
        Calculate the distance from the shoulder to the hand to adjust the radial distance of the target point.
        """
        a2h_direction = [
            self.hand_pose.pose_msg.pose.position.x - self.arm_pose.pose_msg.pose.position.x,
            self.hand_pose.pose_msg.pose.position.y - self.arm_pose.pose_msg.pose.position.y,
        ]
        a2h_distance = math.hypot(a2h_direction[0], a2h_direction[1])
        a2h_unit_vector = [a2h_direction[0] / a2h_distance, a2h_direction[1] / a2h_distance]

        hand_orientation = [
            self.hand_pose.pose_msg.pose.orientation.x,
            self.hand_pose.pose_msg.pose.orientation.y,
            self.hand_pose.pose_msg.pose.orientation.z,
            self.hand_pose.pose_msg.pose.orientation.w,
        ]

        if a2h_distance > 0.4:
            self.expected_a2d_distance = self.expected_a2d_distance + 0.01
        elif 0.4 > a2h_distance > 0.2:
            pass
        else:
            self.expected_a2d_distance = self.expected_a2d_distance - 0.01

        self.expected_a2d_distance = max(1, min(20, self.expected_a2d_distance))

        target_position = [
            a2h_unit_vector[i] * self.expected_a2d_distance + getattr(self.hand_pose.pose_msg.pose.position, axis)
            for i, axis in enumerate(["x", "y"])
        ] + [self.hand_pose.pose_msg.pose.position.z]

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
    def __init__(self, robot_name: str, hand_pose: HandPose, arm_pose: ArmPose, glove: Glove):
        super().__init__(robot_name, hand_pose, arm_pose, glove, node_name="cartesian_mode_traj_pub")
        self.expected_d2target_distance = 0.0
        self.last_target_position = None
        self.origin_position = None

    @staticmethod
    def _init_mode_num():
        return 2

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        """
        The point 0.3 meters in front of the shoulder as the origin of the Cartesian coordinate system.
        The vector from the origin to the hand is the direction of movement for the drone.
        When the vector from the origin to the hand reaches a certain length, the drone moves.
        """
        self.origin_position = [
            self.arm_pose.pose_msg.pose.position.x + 0.3,
            self.arm_pose.pose_msg.pose.position.y,
        ]

        current_hand_position = [
            self.hand_pose.pose_msg.pose.position.x,
            self.hand_pose.pose_msg.pose.position.y,
            self.hand_pose.pose_msg.pose.position.z,
        ]
        hand_orientation = [
            self.hand_pose.pose_msg.pose.orientation.x,
            self.hand_pose.pose_msg.pose.orientation.y,
            self.hand_pose.pose_msg.pose.orientation.z,
            self.hand_pose.pose_msg.pose.orientation.w,
        ]

        o2h_direction = [
            current_hand_position[0] - self.origin_position[0],
            current_hand_position[1] - self.origin_position[1],
        ]
        o2h_distance = math.hypot(o2h_direction[0], o2h_direction[1])
        o2h_unit_vector = [o2h_direction[0] / o2h_distance, o2h_direction[1] / o2h_distance]

        """ The movement distance, maintained at a constant value. """
        if o2h_distance < 0.15:
            self.expected_d2target_distance = 0.0
        else:
            self.expected_d2target_distance = 0.2

        target_position = [
            o2h_unit_vector[i] * self.expected_d2target_distance + getattr(self.uav_odom.pose.pose.position, axis)
            for i, axis in enumerate(["x", "y"])
        ] + [self.hand_pose.pose_msg.pose.position.z]

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
# Derived Class : LockingMode
##########################################
class LockingMode(HandControlBaseMode):
    def __init__(self, robot_name: str, hand_pose: HandPose, arm_pose: ArmPose, glove: Glove):
        super().__init__(robot_name, hand_pose, arm_pose, glove, node_name="locking_mode_traj_pub")
        self._init_origin_drone_position = None
        self._init_origin_hand_orientation = None

    @staticmethod
    def _init_mode_num():
        return 4

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        """The first pose recorded when the drone enters this state."""

        if self._init_origin_drone_position is None:
            self._init_origin_drone_position = [
                self.uav_odom.pose.pose.position.x,
                self.uav_odom.pose.pose.position.y,
                self.uav_odom.pose.pose.position.z,
            ]
            self._init_origin_hand_orientation = [
                self.hand_pose.pose_msg.pose.orientation.x,
                self.hand_pose.pose_msg.pose.orientation.y,
                self.hand_pose.pose_msg.pose.orientation.z,
                self.hand_pose.pose_msg.pose.orientation.w,
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
