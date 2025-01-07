"""
 Created by li-jinjie on 24-1-3.
"""
# !/usr/bin/env python3

import sys
import os
import time
import argparse

import rospy

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

import tf_conversions as tf
import math
from abc import ABC, abstractmethod

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


##########################################
# Base Class
##########################################
class MPCPubBase(ABC):
    """
    A base class that handles:
      - Robot name / node namespace
      - Parameter loading (T_pred, T_integ, T_samp)
      - Odom subscription & storing latest state
      - set_ref_traj publisher
      - Timer callback scaffolding (frequency check, set 'finished' flag)
      - Abstract methods for building the MultiDOFJointTrajectory and checking finish conditions
    """

    def __init__(self, robot_name: str, node_name: str):
        # Basic config
        self.robot_name = robot_name
        self.node_name = node_name
        self.namespace = rospy.get_namespace().rstrip("/")
        self.finished = False  # Flag to indicate trajectory is complete

        # Load NMPC parameters
        try:
            self.T_pred = rospy.get_param(f"{robot_name}/controller/nmpc/T_pred")
            self.T_integ = rospy.get_param(f"{robot_name}/controller/nmpc/T_integ")
            self.T_samp = rospy.get_param(f"{robot_name}/controller/nmpc/T_samp")
        except KeyError:
            raise KeyError("Parameters for NMPC not found! Ensure the NMPC controller is running!")

        self.N_nmpc = int(self.T_pred / self.T_integ)

        # Store latest odometry here
        self.uav_odom = Odometry()
        self.odom_sub = rospy.Subscriber(f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Start time
        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Timer for publishing
        self.ts_pt_pub = 0.02  # ~50Hz
        self.tmr_pt_pub = rospy.Timer(
            rospy.Duration.from_sec(self.ts_pt_pub),
            self._timer_callback
        )
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def sub_odom_callback(self, msg: Odometry):
        """Store the latest odometry data."""
        self.uav_odom = msg

    def _timer_callback(self, timer_event: rospy.timer.TimerEvent):
        """Common timer callback that handles frequency checking and calls user-defined steps."""
        # 1) Check frequency
        if (timer_event.last_duration is not None) and (self.ts_pt_pub < timer_event.last_duration):
            rospy.logwarn(
                f"{self.namespace}: Control loop too slow! "
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < "
                f"timer event duration: {timer_event.last_duration * 1000:.3f} ms"
            )

        # 2) Fill the trajectory from a child-class method
        t_has_started = rospy.Time.now().to_sec() - self.start_time
        traj_msg = self.fill_trajectory_points(t_has_started)

        # 3) Publish
        self.pub_trajectory_points(traj_msg)

        # 4) Check if done from a child-class method
        if self.check_finished(t_has_started):
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory finished or target reached!")
            self.tmr_pt_pub.shutdown()
            self.finished = True

    @abstractmethod
    def fill_trajectory_points(self, t_elapsed: float):
        """
        Construct and return a MultiDOFJointTrajectory or PredXU for the current time.
        Must be implemented by the child class.
        """
        pass

    @abstractmethod
    def pub_trajectory_points(self, traj_msg):
        """
        Publish the MultiDOFJointTrajectory or PredXU message.
        Must be implemented by the child class.
        """
        pass

    @abstractmethod
    def check_finished(self, t_elapsed: float) -> bool:
        """
        Return True if we are done publishing / have reached the target.
        Must be implemented by the child class.
        """
        pass


class MPCPubJointTraj(MPCPubBase, ABC):
    def __init__(self, robot_name: str, node_name: str):
        super().__init__(robot_name=robot_name, node_name=node_name)
        # Publisher for reference trajectory
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=2)

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
                t_pred = i * self.T_integ
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
                 pos_tol=0.1, ang_tol=0.1, vel_tol=0.1, rate_tol=0.1):
        super().__init__(robot_name=robot_name, node_name="mpc_single_pt_pub")
        self.target_pose = target_pose

        # Tolerances for considering the target "reached"
        self.pos_tol = pos_tol  # e.g. 0.1 m
        self.ang_tol = ang_tol  # e.g. 0.1 rad
        self.vel_tol = vel_tol  # e.g. 0.1 m/s
        self.rate_tol = rate_tol  # e.g. 0.1 rad/s

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

            t_pred = i * self.T_integ
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
