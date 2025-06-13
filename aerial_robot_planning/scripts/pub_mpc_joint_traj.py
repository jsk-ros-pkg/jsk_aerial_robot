"""
 Created by li-jinjie on 24-1-3.
"""
# !/usr/bin/env python3

import sys
import os
import math
from abc import ABC, abstractmethod
import rospy
import tf_conversions as tf

from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from aerial_robot_msgs.msg import FixRotor

from pub_mpc_base import MPCPubBase

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)


##########################################
# Derived Class: MPCPubJointTraj
##########################################
class MPCPubJointTraj(MPCPubBase, ABC):
    def __init__(self, robot_name: str, node_name: str, is_calc_rmse=True):
        super().__init__(robot_name=robot_name, node_name=node_name, is_calc_rmse=is_calc_rmse)
        # Publisher for reference trajectory
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=3)
        self.pub_fixed_rotor = rospy.Publisher(f"/{robot_name}/set_fixed_rotor", FixRotor, queue_size=3)

    def pub_trajectory_points(self, traj_msg: MultiDOFJointTrajectory):
        """Publish the MultiDOFJointTrajectory message."""
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.header.frame_id = "map"
        self.pub_ref_traj.publish(traj_msg)


##########################################
# Derived Class #1: MPCTrajPtPub
##########################################
class MPCTrajPtPub(MPCPubJointTraj):
    """
    Publishes an entire trajectory (multiple changing waypoints)
    to the NMPC controller, based on a 'traj' object (e.g. CircleTraj, etc.).
    """

    def __init__(self, robot_name: str, traj):
        super().__init__(robot_name=robot_name, node_name="mpc_traj_pt_pub")
        self.traj = traj
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Using trajectory {str(self.traj)}")

        self.start_timer()

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        """
        Build a MultiDOFJointTrajectory of length N_nmpc+1 from the 'traj' object,
        evaluating at times t_elapsed + (0..N)*T_integ (if it's time-varying)
        or always t_elapsed if the reference is the same across the horizon.
        """
        multi_dof_joint_traj = MultiDOFJointTrajectory()

        # For certain rotation-based trajectories, we might keep the same reference
        # across the entire horizon
        if hasattr(self.traj, "use_constant_ref") and self.traj.use_constant_ref is True:
            is_ref_different = False
        else:
            # E.g., if it's a circle, lemniscate, etc.
            is_ref_different = True

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()

            # Time used for trajectory generation
            if is_ref_different:
                t_pred = i * self.T_step
            else:
                t_pred = 0.0

            t_cal = t_elapsed + t_pred

            # position
            x, y, z, vx, vy, vz, ax, ay, az = self.traj.get_3d_pt(t_cal)

            # orientation
            try:
                qw, qx, qy, qz, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc = self.traj.get_3d_orientation(t_cal)
            except AttributeError:
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
                r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
                r_acc, p_acc, y_acc = 0.0, 0.0, 0.0

            # new feature: fix rotor  TODO: think of a better place to put this function. Only JointTraj has this function.
            try:
                rotor_id, ft_fixed, alpha_fixed = self.traj.get_fixed_rotor(t_elapsed)

                if self.traj.use_fix_rotor_flag:
                    fixed_rotor_msg = FixRotor()
                    fixed_rotor_msg.header.stamp = rospy.Time.now()
                    fixed_rotor_msg.rotor_id = rotor_id
                    fixed_rotor_msg.fix_ft = ft_fixed
                    fixed_rotor_msg.fix_alpha = alpha_fixed
                    self.pub_fixed_rotor.publish(fixed_rotor_msg)
            except AttributeError:
                pass

            # Fill the transforms / velocities / accelerations
            traj_pt.transforms.append(Transform(translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw)))
            traj_pt.velocities.append(Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate)))
            traj_pt.accelerations.append(Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc)))
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        return multi_dof_joint_traj

    def check_finished(self, t_elapsed: float) -> bool:
        """
        Use the 'traj.check_finished()' method to see if the trajectory is complete.
        """
        return self.traj.check_finished(t_elapsed)


##########################################
# Derived Class #2: MPCSinglePtPub
##########################################
class MPCSinglePtPub(MPCPubJointTraj):
    """
    Publishes a single constant point for the entire horizon (N_nmpc+1),
    and checks if the robot has reached it within a certain error threshold.
    """

    def __init__(self, robot_name: str, target_pose: Pose,
                 pos_tol=0.2, ang_tol=0.3, vel_tol=0.1, rate_tol=0.1):
        super().__init__(robot_name=robot_name, node_name="mpc_single_pt_pub", is_calc_rmse=False)
        self.target_pose = target_pose
        rospy.loginfo(f"{self.namespace}/{self.node_name}: \n"
                      f"Target pose: x {self.target_pose.position.x}, "
                      f"y {self.target_pose.position.y}, z {self.target_pose.position.z}; "
                      f"qw {self.target_pose.orientation.w}, qx {self.target_pose.orientation.x}, "
                      f"qy {self.target_pose.orientation.y}, qz {self.target_pose.orientation.z}")

        # Tolerances for considering the target "reached"
        self.pos_tol = pos_tol  # e.g. 0.1 m
        self.ang_tol = ang_tol  # e.g. 0.1 rad
        self.vel_tol = vel_tol  # e.g. 0.1 m/s
        self.rate_tol = rate_tol  # e.g. 0.1 rad/s

        self.start_timer()

    def fill_trajectory_points(self, t_elapsed: float) -> MultiDOFJointTrajectory:
        """
        Build a MultiDOFJointTrajectory where every point is the same (target_pose).
        """
        traj_msg = MultiDOFJointTrajectory()

        x = self.target_pose.position.x
        y = self.target_pose.position.y
        z = self.target_pose.position.z
        qx = self.target_pose.orientation.x
        qy = self.target_pose.orientation.y
        qz = self.target_pose.orientation.z
        qw = self.target_pose.orientation.w

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(
                    translation=Vector3(x, y, z),
                    rotation=Quaternion(qx, qy, qz, qw)
                )
            )
            # No velocity or acceleration
            traj_pt.velocities.append(Twist())
            traj_pt.accelerations.append(Twist())

            t_pred = i * self.T_step
            traj_pt.time_from_start = rospy.Duration.from_sec(t_elapsed + t_pred)

            traj_msg.points.append(traj_pt)

        return traj_msg

    def check_finished(self, t_elapsed: float) -> bool:
        """
        Check if the robot has reached (or is close enough to) target_pose,
        including minimal velocity and angular rate, etc.
        """
        # Current pose
        cur_pos = self.uav_odom.pose.pose.position
        dx = cur_pos.x - self.target_pose.position.x
        dy = cur_pos.y - self.target_pose.position.y
        dz = cur_pos.z - self.target_pose.position.z
        pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)

        # Current velocity
        vx = self.uav_odom.twist.twist.linear.x
        vy = self.uav_odom.twist.twist.linear.y
        vz = self.uav_odom.twist.twist.linear.z
        vel_err = math.sqrt(vx * vx + vy * vy + vz * vz)

        # Orientation
        q = self.uav_odom.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        tq = self.target_pose.orientation
        (roll_ref, pitch_ref, yaw_ref) = tf.transformations.euler_from_quaternion([tq.x, tq.y, tq.z, tq.w])
        ang_err = math.sqrt((roll - roll_ref) ** 2 + (pitch - pitch_ref) ** 2 + (yaw - yaw_ref) ** 2)

        # Angular velocity
        wx = self.uav_odom.twist.twist.angular.x
        wy = self.uav_odom.twist.twist.angular.y
        wz = self.uav_odom.twist.twist.angular.z
        rate_err = math.sqrt(wx * wx + wy * wy + wz * wz)

        # Check thresholds
        if (pos_err < self.pos_tol and ang_err < self.ang_tol
                and vel_err < self.vel_tol and rate_err < self.rate_tol):
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Target reached!")
            return True
        else:
            return False
