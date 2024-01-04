"""
 Created by li-jinjie on 24-1-3.
"""

import sys
import os

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

import time
import numpy as np
import rospy
import rospkg
import yaml
import tf2_ros
import actionlib
from typing import List, Tuple

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
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull_sim.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = param_dict["controller"]["nmpc"]
N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])


def prepare_x_u(x_ref: np.array, u_ref: np.array):
    x_u = PredXU()

    x_dim_1 = MultiArrayDimension()
    x_dim_1.label = "horizon"
    x_dim_1.size = N_nmpc + 1
    x_dim_1.stride = (N_nmpc + 1) * 17
    x_u.x.layout.dim.append(x_dim_1)
    x_dim_2 = MultiArrayDimension()
    x_dim_2.label = "state"
    x_dim_2.size = 17
    x_dim_2.stride = 17
    x_u.x.layout.dim.append(x_dim_2)
    x_u.x.layout.data_offset = 0
    x_u.x.data = np.resize(x_ref, (N_nmpc + 1) * 17).astype(np.float64).tolist()

    u_dim_1 = MultiArrayDimension()
    u_dim_1.label = "horizon"
    u_dim_1.size = N_nmpc
    u_dim_1.stride = N_nmpc * 8
    x_u.u.layout.dim.append(u_dim_1)
    u_dim_2 = MultiArrayDimension()
    u_dim_2.label = "input"
    u_dim_2.size = 8
    u_dim_2.stride = 8
    x_u.u.layout.dim.append(u_dim_2)
    x_u.u.layout.data_offset = 0
    x_u.u.data = np.resize(u_ref, N_nmpc * 8).astype(np.float64).tolist()

    return x_u


class MPCPtPubNode:
    def __init__(self) -> None:
        self.node_name = "mpc_pt_pub_node"
        rospy.init_node(self.node_name, anonymous=False)
        self.namespace = rospy.get_namespace().rstrip("/")

        # Timer
        self.ts_pt_pub = 0.025  # [s]  ~40Hz
        self.tmr_pt_pub = rospy.Timer(rospy.Duration(self.ts_pt_pub), self.callback_tmr_pt_pub)

        # Sub --> feedback
        self.uav_odom = Odometry()
        rospy.Subscriber("/beetle1/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Pub
        self.pub_ref_traj = rospy.Publisher("/beetle1/set_ref_traj", PredXU, queue_size=5)

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

    def callback_tmr_pt_pub(self, timer: rospy.timer.TimerEvent):
        """publish the reference points to the controller"""
        # 1. check if the frequency is too slow
        if timer.last_duration is not None and self.ts_pt_pub < timer.last_duration:
            rospy.logwarn(
                f"{self.namespace}: Control is too slow!"
                f"ts_pt_pub: {self.ts_pt_pub * 1000:.3f} ms < ts_one_round: {timer.last_duration * 1000:.3f} ms"
            )

        # 2. send the reference points to the controller
        x_init = np.array([0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        x_ref = np.tile(x_init, (1, N_nmpc + 1)).reshape(-1, x_init.shape[0])

        u_init = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        u_ref = np.tile(u_init, (1, N_nmpc)).reshape(-1, u_init.shape[0])

        ref_x_u = prepare_x_u(x_ref, u_ref)

        ref_x_u.header.stamp = rospy.Time.now()
        ref_x_u.header.frame_id = "map"
        self.pub_ref_traj.publish(ref_x_u)

    def sub_odom_callback(self, msg: Odometry):
        self.uav_odom = msg


if __name__ == "__main__":
    try:
        node = MPCPtPubNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
