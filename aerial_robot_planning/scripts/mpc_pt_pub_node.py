"""
 Created by li-jinjie on 24-1-3.
"""

import sys
import os
import argparse

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

import numpy as np
import rospy
import rospkg
import yaml
import tf_conversions as tf
import actionlib
from typing import List, Tuple
from trajs import *

from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray, TransformStamped
from aerial_robot_msgs.msg import (
    TrackTrajAction,
    TrackTrajGoal,
    TrackTrajResult,
    TrackTrajFeedback,
    PredXU,
)

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = param_dict["controller"]["nmpc"]

physical_params = param_dict["controller"]["physical"]
mass = physical_params["mass"]
gravity = physical_params["gravity"]
Ixx = physical_params["inertia_diag"][0]
Iyy = physical_params["inertia_diag"][1]
Izz = physical_params["inertia_diag"][2]
dr1 = physical_params["dr1"]
p1_b = physical_params["p1"]
dr2 = physical_params["dr2"]
p2_b = physical_params["p2"]
dr3 = physical_params["dr3"]
p3_b = physical_params["p3"]
dr4 = physical_params["dr4"]
p4_b = physical_params["p4"]
kq_d_kt = physical_params["kq_d_kt"]


def prepare_x_u(n_nmpc: int, x_ref: np.array, u_ref: np.array):
    x_u = PredXU()

    x_dim_1 = MultiArrayDimension()
    x_dim_1.label = "horizon"
    x_dim_1.size = n_nmpc + 1
    x_dim_1.stride = (n_nmpc + 1) * 17
    x_u.x.layout.dim.append(x_dim_1)
    x_dim_2 = MultiArrayDimension()
    x_dim_2.label = "state"
    x_dim_2.size = 17
    x_dim_2.stride = 17
    x_u.x.layout.dim.append(x_dim_2)
    x_u.x.layout.data_offset = 0
    x_u.x.data = np.resize(x_ref, (n_nmpc + 1) * 17).astype(np.float64).tolist()

    u_dim_1 = MultiArrayDimension()
    u_dim_1.label = "horizon"
    u_dim_1.size = n_nmpc
    u_dim_1.stride = n_nmpc * 8
    x_u.u.layout.dim.append(u_dim_1)
    u_dim_2 = MultiArrayDimension()
    u_dim_2.label = "input"
    u_dim_2.size = 8
    u_dim_2.stride = 8
    x_u.u.layout.dim.append(u_dim_2)
    x_u.u.layout.data_offset = 0
    x_u.u.data = np.resize(u_ref, n_nmpc * 8).astype(np.float64).tolist()

    return x_u


def construct_allocation_mat_pinv():
    # get allocation matrix
    alloc_mat = np.zeros((6, 8))
    sqrt_p1b_xy = np.sqrt(p1_b[0] ** 2 + p1_b[1] ** 2)
    sqrt_p2b_xy = np.sqrt(p2_b[0] ** 2 + p2_b[1] ** 2)
    sqrt_p3b_xy = np.sqrt(p3_b[0] ** 2 + p3_b[1] ** 2)
    sqrt_p4b_xy = np.sqrt(p4_b[0] ** 2 + p4_b[1] ** 2)

    # - force
    alloc_mat[0, 0] = p1_b[1] / sqrt_p1b_xy
    alloc_mat[1, 0] = -p1_b[0] / sqrt_p1b_xy
    alloc_mat[2, 1] = 1

    alloc_mat[0, 2] = p2_b[1] / sqrt_p2b_xy
    alloc_mat[1, 2] = -p2_b[0] / sqrt_p2b_xy
    alloc_mat[2, 3] = 1

    alloc_mat[0, 4] = p3_b[1] / sqrt_p3b_xy
    alloc_mat[1, 4] = -p3_b[0] / sqrt_p3b_xy
    alloc_mat[2, 5] = 1

    alloc_mat[0, 6] = p4_b[1] / sqrt_p4b_xy
    alloc_mat[1, 6] = -p4_b[0] / sqrt_p4b_xy
    alloc_mat[2, 7] = 1

    # - torque
    alloc_mat[3, 0] = -dr1 * kq_d_kt * p1_b[1] / sqrt_p1b_xy + p1_b[0] * p1_b[2] / sqrt_p1b_xy
    alloc_mat[4, 0] = dr1 * kq_d_kt * p1_b[0] / sqrt_p1b_xy + p1_b[1] * p1_b[2] / sqrt_p1b_xy
    alloc_mat[5, 0] = -p1_b[0] ** 2 / sqrt_p1b_xy - p1_b[1] ** 2 / sqrt_p1b_xy

    alloc_mat[3, 1] = p1_b[1]
    alloc_mat[4, 1] = -p1_b[0]
    alloc_mat[5, 1] = -dr1 * kq_d_kt

    alloc_mat[3, 2] = -dr2 * kq_d_kt * p2_b[1] / sqrt_p2b_xy + p2_b[0] * p2_b[2] / sqrt_p2b_xy
    alloc_mat[4, 2] = dr2 * kq_d_kt * p2_b[0] / sqrt_p2b_xy + p2_b[1] * p2_b[2] / sqrt_p2b_xy
    alloc_mat[5, 2] = -p2_b[0] ** 2 / sqrt_p2b_xy - p2_b[1] ** 2 / sqrt_p2b_xy

    alloc_mat[3, 3] = p2_b[1]
    alloc_mat[4, 3] = -p2_b[0]
    alloc_mat[5, 3] = -dr2 * kq_d_kt

    alloc_mat[3, 4] = -dr3 * kq_d_kt * p3_b[1] / sqrt_p3b_xy + p3_b[0] * p3_b[2] / sqrt_p3b_xy
    alloc_mat[4, 4] = dr3 * kq_d_kt * p3_b[0] / sqrt_p3b_xy + p3_b[1] * p3_b[2] / sqrt_p3b_xy
    alloc_mat[5, 4] = -p3_b[0] ** 2 / sqrt_p3b_xy - p3_b[1] ** 2 / sqrt_p3b_xy

    alloc_mat[3, 5] = p3_b[1]
    alloc_mat[4, 5] = -p3_b[0]
    alloc_mat[5, 5] = -dr3 * kq_d_kt

    alloc_mat[3, 6] = -dr4 * kq_d_kt * p4_b[1] / sqrt_p4b_xy + p4_b[0] * p4_b[2] / sqrt_p4b_xy
    alloc_mat[4, 6] = dr4 * kq_d_kt * p4_b[0] / sqrt_p4b_xy + p4_b[1] * p4_b[2] / sqrt_p4b_xy
    alloc_mat[5, 6] = -p4_b[0] ** 2 / sqrt_p4b_xy - p4_b[1] ** 2 / sqrt_p4b_xy

    alloc_mat[3, 7] = p4_b[1]
    alloc_mat[4, 7] = -p4_b[0]
    alloc_mat[5, 7] = -dr4 * kq_d_kt

    alloc_mat_pinv = np.linalg.pinv(alloc_mat)

    return alloc_mat_pinv, alloc_mat


class MPCPtPubNode:
    def __init__(self, traj_type: int) -> None:
        self.node_name = "mpc_pt_pub_node"
        rospy.init_node(self.node_name, anonymous=False)
        self.namespace = rospy.get_namespace().rstrip("/")

        # Sub --> feedback
        self.uav_odom = Odometry()
        rospy.Subscriber("/beetle1/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Pub
        self.pub_ref_traj = rospy.Publisher("/beetle1/set_ref_traj", PredXU, queue_size=5)

        # nmpc and robot related
        self.alloc_mat_pinv, self.alloc_mat = construct_allocation_mat_pinv()
        self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

        # traj
        if traj_type == 0:
            self.traj = SetPointTraj()
        elif traj_type == 1:
            self.traj = CircleTraj()
        elif traj_type == 2:
            self.traj = LemniscateTraj()
        elif traj_type == 3:
            self.traj = LemniscateTrajOmni()
        else:
            raise ValueError("Invalid trajectory type!")

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory type: {self.traj.__str__()}")

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Timer
        self.ts_pt_pub = 0.02  # [s]  ~50Hz
        self.tmr_pt_pub = rospy.Timer(rospy.Duration(self.ts_pt_pub), self.callback_tmr_pt_pub)
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def get_one_xr_ur_from_target(
            self,
            target_pos,
            target_vel=np.array([[0.0, 0.0, 0.0]]).T,
            target_qwxyz=np.array([[1.0, 0.0, 0.0, 0.0]]).T,
            target_omega=np.array([[0.0, 0.0, 0.0]]).T,
            target_non_gravity_wrench=np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T,
    ):
        anti_gravity_wrench = np.array([[0, 0, mass * gravity, 0, 0, 0]]).T
        target_wrench = anti_gravity_wrench + target_non_gravity_wrench

        x = self.alloc_mat_pinv @ target_wrench

        a1_ref = np.arctan2(x[0, 0], x[1, 0])
        ft1_ref = np.sqrt(x[0, 0] ** 2 + x[1, 0] ** 2)
        a2_ref = np.arctan2(x[2, 0], x[3, 0])
        ft2_ref = np.sqrt(x[2, 0] ** 2 + x[3, 0] ** 2)
        a3_ref = np.arctan2(x[4, 0], x[5, 0])
        ft3_ref = np.sqrt(x[4, 0] ** 2 + x[5, 0] ** 2)
        a4_ref = np.arctan2(x[6, 0], x[7, 0])
        ft4_ref = np.sqrt(x[6, 0] ** 2 + x[7, 0] ** 2)

        # get x and u, set reference
        xr = np.zeros([1, 17])
        ur = np.zeros([1, 8])
        xr[0, 0] = target_pos.item(0)  # x
        xr[0, 1] = target_pos.item(1)  # y
        xr[0, 2] = target_pos.item(2)  # z
        xr[0, 3] = target_vel.item(0)
        xr[0, 4] = target_vel.item(1)
        xr[0, 5] = target_vel.item(2)
        xr[0, 6] = target_qwxyz.item(0)  # qw
        xr[0, 7] = target_qwxyz.item(1)  # qx
        xr[0, 8] = target_qwxyz.item(2)  # qy
        xr[0, 9] = target_qwxyz.item(3)  # qz
        xr[0, 10] = target_omega.item(0)
        xr[0, 11] = target_omega.item(1)
        xr[0, 12] = target_omega.item(2)
        xr[0, 13] = a1_ref
        xr[0, 14] = a2_ref
        xr[0, 15] = a3_ref
        xr[0, 16] = a4_ref
        ur[0, 0] = ft1_ref
        ur[0, 1] = ft2_ref
        ur[0, 2] = ft3_ref
        ur[0, 3] = ft4_ref

        return xr, ur

    def callback_tmr_pt_pub(self, timer: rospy.timer.TimerEvent):
        """publish the reference points to the controller"""
        # 1. check if the frequency is too slow
        if timer.last_duration is not None and self.ts_pt_pub < timer.last_duration:
            rospy.logwarn(
                f"{self.namespace}: Control is too slow!"
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < ts_one_round: {timer.last_duration * 1000:.3f} ms"
            )

        # 2. calculate the reference points
        x_ref = np.zeros([self.N_nmpc + 1, 17])
        u_ref = np.zeros([self.N_nmpc, 8])

        is_ref_different = True

        for i in range(self.N_nmpc + 1):
            if is_ref_different:
                t_pred = i * nmpc_params["T_integ"]  # all future reference points are different
            else:
                t_pred = 0.0

            t_cal = t_pred + rospy.Time.now().to_sec() - self.start_time

            # position
            x, y, z, vx, vy, vz, ax, ay, az = self.traj.get_3d_pt(t_cal)

            target_pos = np.array([[x, y, z]]).T
            target_vel = np.array([[vx, vy, vz]]).T

            # orientation
            # check if there is get_3d_orientation method inside self.traj
            try:
                roll, pitch, yaw, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc = self.traj.get_3d_orientation(t_cal)
            except AttributeError:
                roll, pitch, yaw = 0.0, 0.0, 0.0
                r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
                r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

            target_qwxyz = np.array([[q[3], q[0], q[1], q[2]]]).T
            target_body_rate = np.array([[r_rate, p_rate, y_rate]]).T

            target_non_gravity_wrench = np.array(
                [[mass * ax, mass * ay, mass * az, Ixx * r_acc, Iyy * p_acc, Izz * y_acc]]).T

            xr, ur = self.get_one_xr_ur_from_target(
                target_pos=target_pos,
                target_vel=target_vel,
                target_qwxyz=target_qwxyz,
                target_omega=target_body_rate,
                target_non_gravity_wrench=target_non_gravity_wrench,
            )

            x_ref[i] = xr
            if i == self.N_nmpc:
                break
            u_ref[i] = ur

        # 3. send the reference points to the controller
        ros_x_u = prepare_x_u(self.N_nmpc, x_ref, u_ref)
        ros_x_u.header.stamp = rospy.Time.now()
        ros_x_u.header.frame_id = "map"
        self.pub_ref_traj.publish(ros_x_u)

    def sub_odom_callback(self, msg: Odometry):
        self.uav_odom = msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="MPC Point Trajectory Publisher Node")
    parser.add_argument("traj_type", type=int,
                        help="Trajectory type: 0 for set-point, 1 for Circular, 2 for Lemniscate, 3 for Lemniscate omni")
    args = parser.parse_args()

    try:
        node = MPCPtPubNode(args.traj_type)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
