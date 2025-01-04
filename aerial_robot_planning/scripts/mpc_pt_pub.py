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
import numpy as np  # Assuming you need numpy for np.inf
from trajs import BaseTraj, PitchRotationTraj, RollRotationTraj

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3, Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint


###############################################
# Original MPCPtPubNode
###############################################
class MPCTrajPtPub:
    def __init__(self, robot_name: str, traj: BaseTraj):
        self.robot_name = robot_name
        self.node_name = "mpc_traj_pt_pub"

        self.namespace = rospy.get_namespace().rstrip("/")
        self.finished = False  # Flag to indicate trajectory is done

        self.traj = traj
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory type: {self.traj.__str__()}")

        # get the parameters
        try:
            self.T_pred = rospy.get_param(f"{robot_name}/controller/nmpc/T_pred")
            self.T_integ = rospy.get_param(f"{robot_name}/controller/nmpc/T_integ")
            self.T_samp = rospy.get_param(f"{robot_name}/controller/nmpc/T_samp")
        except KeyError:
            raise KeyError("Parameters for NMPC not found! Please ensure that the NMPC controller is running!")

        # Sub --> feedback
        self.uav_odom = Odometry()
        rospy.Subscriber(f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Pub
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=5)

        # nmpc and robot related
        self.N_nmpc = int(self.T_pred / self.T_integ)

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Timer
        self.ts_pt_pub = 0.02  # [s]  ~50Hz
        self.tmr_pt_pub = rospy.Timer(rospy.Duration.from_sec(self.ts_pt_pub), self.callback_tmr_pt_pub)
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def callback_tmr_pt_pub(self, timer: rospy.timer.TimerEvent):
        """publish the reference points to the controller"""
        # 1. check if the frequency is too slow
        if timer.last_duration is not None and self.ts_pt_pub < timer.last_duration:
            rospy.logwarn(
                f"{self.namespace}: Control is too slow!"
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < ts_one_round: {timer.last_duration * 1000:.3f} ms"
            )

        multi_dof_joint_traj = MultiDOFJointTrajectory()

        # 2. calculate the reference points
        is_ref_different = True
        if isinstance(self.traj, PitchRotationTraj) or isinstance(self.traj, RollRotationTraj):
            # For these trajectories, the same reference is used for the entire horizon
            is_ref_different = False

        t_has_started = rospy.Time.now().to_sec() - self.start_time
        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()

            if is_ref_different:
                t_pred = i * self.T_integ
            else:
                t_pred = 0.0

            t_cal = t_pred + t_has_started

            # position
            x, y, z, vx, vy, vz, ax, ay, az = self.traj.get_3d_pt(t_cal)

            # orientation
            try:
                (roll, pitch, yaw, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc) = self.traj.get_3d_orientation(t_cal)
            except AttributeError:
                roll, pitch, yaw = 0.0, 0.0, 0.0
                r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
                r_acc, p_acc, y_acc = 0.0, 0.0, 0.0

            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            qx, qy, qz, qw = q

            traj_pt.transforms.append(Transform(translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw)))
            traj_pt.velocities.append(Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate)))
            traj_pt.accelerations.append(Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc)))
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        # 3. send the reference points to the controller
        multi_dof_joint_traj.header.stamp = rospy.Time.now()
        multi_dof_joint_traj.header.frame_id = "map"
        self.pub_ref_traj.publish(multi_dof_joint_traj)

        # 4. check if the trajectory is finished
        if self.traj.check_finished(rospy.Time.now().to_sec() - self.start_time):
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory finished!")
            self.tmr_pt_pub.shutdown()
            self.finished = True  # Instead of shutting down, mark as finished
            return

    def sub_odom_callback(self, msg: Odometry):
        self.uav_odom = msg


class MPCSinglePtPub:
    def __init__(self, robot_name: str, target_pose: Pose):
        self.robot_name = robot_name
        self.node_name = "mpc_single_pt_pub"

        self.target_pose = target_pose

        self.namespace = rospy.get_namespace().rstrip("/")
        self.finished = False  # Flag to indicate trajectory is done

        # get the parameters
        try:
            self.T_pred = rospy.get_param(f"{robot_name}/controller/nmpc/T_pred")
            self.T_integ = rospy.get_param(f"{robot_name}/controller/nmpc/T_integ")
            self.T_samp = rospy.get_param(f"{robot_name}/controller/nmpc/T_samp")
        except KeyError:
            raise KeyError("Parameters for NMPC not found! Please ensure that the NMPC controller is running!")

        # Sub --> feedback
        self.uav_odom = Odometry()
        rospy.Subscriber(f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Pub
        self.pub_ref_traj = rospy.Publisher(f"/{robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=5)

        # nmpc and robot related
        self.N_nmpc = int(self.T_pred / self.T_integ)

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Timer
        self.ts_pt_pub = 0.02  # [s]  ~50Hz
        self.tmr_pt_pub = rospy.Timer(rospy.Duration.from_sec(self.ts_pt_pub), self.callback_tmr_pt_pub)
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def callback_tmr_pt_pub(self, timer: rospy.timer.TimerEvent):
        """publish the reference points to the controller"""
        # 1. check if the frequency is too slow
        if timer.last_duration is not None and self.ts_pt_pub < timer.last_duration:
            rospy.logwarn(
                f"{self.namespace}: Control is too slow!"
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < ts_one_round: {timer.last_duration * 1000:.3f} ms"
            )

        multi_dof_joint_traj = MultiDOFJointTrajectory()

        # 2. calculate the reference points
        t_has_started = rospy.Time.now().to_sec() - self.start_time
        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()

            x = self.target_pose.position.x
            y = self.target_pose.position.y
            z = self.target_pose.position.z
            qw = self.target_pose.orientation.w
            qx = self.target_pose.orientation.x
            qy = self.target_pose.orientation.y
            qz = self.target_pose.orientation.z

            traj_pt.transforms.append(Transform(translation=Vector3(x, y, z), rotation=Quaternion(qx, qy, qz, qw)))
            traj_pt.velocities.append(Twist(linear=Vector3(), angular=Vector3()))
            traj_pt.accelerations.append(Twist(linear=Vector3(), angular=Vector3()))

            t_pred = i * self.T_integ
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        # 3. send the reference points to the controller
        multi_dof_joint_traj.header.stamp = rospy.Time.now()
        multi_dof_joint_traj.header.frame_id = "map"
        self.pub_ref_traj.publish(multi_dof_joint_traj)

        # 4. check if the target point is reached
        position_error = np.linalg.norm(
            np.array([self.uav_odom.pose.pose.position.x
                      - self.target_pose.position.x,
                      self.uav_odom.pose.pose.position.y
                      - self.target_pose.position.y,
                      self.uav_odom.pose.pose.position.z
                      - self.target_pose.position.z])
        )

        velocity_error = np.linalg.norm(
            np.array([self.uav_odom.twist.twist.linear.x,
                      self.uav_odom.twist.twist.linear.y,
                      self.uav_odom.twist.twist.linear.z])
        )

        roll, pitch, yaw = tf.transformations.euler_from_quaternion([
            self.uav_odom.pose.pose.orientation.x,
            self.uav_odom.pose.pose.orientation.y,
            self.uav_odom.pose.pose.orientation.z,
            self.uav_odom.pose.pose.orientation.w
        ])
        roll_ref, pitch_ref, yaw_ref = tf.transformations.euler_from_quaternion([
            self.target_pose.orientation.x,
            self.target_pose.orientation.y,
            self.target_pose.orientation.z,
            self.target_pose.orientation.w
        ])
        orientation_error = np.linalg.norm(np.array([roll - roll_ref, pitch - pitch_ref, yaw - yaw_ref]))

        body_rate_error = np.linalg.norm(
            np.array([self.uav_odom.twist.twist.angular.x,
                      self.uav_odom.twist.twist.angular.y,
                      self.uav_odom.twist.twist.angular.z])
        )

        if (position_error < 0.1 and orientation_error < 0.1
                and velocity_error < 0.1 and body_rate_error < 0.1):  # 10 cm and 0.1 rad = 5.7 deg
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Target reached!")
            self.tmr_pt_pub.shutdown()
            self.finished = True
            return

    def sub_odom_callback(self, msg: Odometry):
        self.uav_odom = msg
