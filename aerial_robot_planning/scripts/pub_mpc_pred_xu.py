"""
Created by li-jinjie on 24-1-3.
"""

# !/usr/bin/env python3

import sys
import os
import numpy as np
import rospy
from abc import ABC, abstractmethod

from std_msgs.msg import MultiArrayDimension
from aerial_robot_msgs.msg import PredXU
from nav_msgs.msg import Path

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from pub_mpc_joint_traj import MPCPubBase
from util import check_traj_info


##########################################
# Derived Class: MPCPubPredXU
##########################################
class MPCPubPredXU(MPCPubBase, ABC):
    def __init__(self, robot_name: str, node_name: str):
        super().__init__(robot_name=robot_name, node_name=node_name)
        # Publisher for PredXU
        self.pub_ref_xu = rospy.Publisher(f"/{robot_name}/set_ref_x_u", PredXU, queue_size=3)

    def pub_trajectory_points(self, pred_xu_msg: PredXU):
        """Publish the PredXU message."""
        pred_xu_msg.header.stamp = rospy.Time.now()
        pred_xu_msg.header.frame_id = "map"
        self.pub_ref_xu.publish(pred_xu_msg)


##########################################
# Derived Class #1: MPCPubCSVPredXU
##########################################
class MPCPubCSVPredXU(MPCPubPredXU):
    """
    Derived from MPCPubPredXU, which already inherits from MPCPubBase.
    This class loads a trajectory from CSV, interpolates it, and publishes
    PredXU messages at the rate defined by the base class (~50Hz).
    """

    def __init__(self, robot_name: str, file_path: str) -> None:
        # Initialize parent classes
        super().__init__(robot_name=robot_name, node_name="mpc_xu_pub_node")

        # Prepare the PredXU message (dimensions, etc.)
        self.ref_xu_msg = PredXU()
        if len(self.ref_xu_msg.x.layout.dim) < 2:
            self.ref_xu_msg.x.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]
        if len(self.ref_xu_msg.u.layout.dim) < 2:
            self.ref_xu_msg.u.layout.dim = [MultiArrayDimension(), MultiArrayDimension()]

        # Robot/trajectory dimensionalities
        if self.nx != 23 or self.nu != 8:
            raise ValueError("This class is designed for 23 states and 8 control inputs! Check the NMPC type!")

        # Load trajectory from a CSV
        self.scvx_traj = np.loadtxt(file_path, delimiter=",")  # one row for one time step
        self.x_traj = self.scvx_traj[:, 0:19]
        self.u_traj = self.scvx_traj[:, 19:28]

        # Check trajectory information and visualize it
        traj_path_msg = check_traj_info(self.x_traj, if_return_path=True)
        self.traj_path_pub = rospy.Publisher(f"/{self.robot_name}/traj_path", Path, queue_size=1, latch=True)
        self.traj_path_pub.publish(traj_path_msg)

        input_str = input(
            "Please check the traj info and Rviz visualization. Press 'Enter' to continue or 'q' to quit..."
        )
        while True:
            if input_str.lower() == "q":
                self.cleanup_traj_viz()
                self.is_finished = True
                return
            elif input_str.lower() == "":
                break
            else:
                input_str = input("Invalid input. Please press 'Enter' to continue or 'q' to quit...")

        # Adjust your control inputs if needed
        self.u_traj[:, 4:8] = self.x_traj[:, 13:17]

        self.start_timer()

    def fill_trajectory_points(self, t_elapsed: float) -> PredXU:
        """
        Construct and return a PredXU message for the current time `t_elapsed`.
        This method is called automatically by the base class timer (~50 Hz).
        """
        # If we are still within our trajectory time window:
        if t_elapsed <= self.x_traj[-1, -2]:
            # Create time nodes for interpolation
            t_nodes = np.linspace(0, self.T_horizon, self.N_nmpc + 1)
            t_nodes += t_elapsed

            # Allocate storage for interpolation results
            x_traj = np.zeros((self.N_nmpc + 1, self.nx))
            u_traj = np.zeros((self.N_nmpc, self.nu))

            # Interpolate each state dimension
            for i in range(self.nx - 6):
                x_traj[:, i] = np.interp(t_nodes, self.x_traj[:, -2], self.x_traj[:, i])

            # Interpolate each control dimension
            for i in range(self.nu):
                u_traj[:, i] = np.interp(t_nodes[:-1], self.x_traj[:, -2], self.u_traj[:, i])

            # Populate the PredXU message
            self.ref_xu_msg.x.layout.dim[1].stride = self.nx
            self.ref_xu_msg.x.layout.dim[0].size = self.N_nmpc + 1
            self.ref_xu_msg.u.layout.dim[1].stride = self.nu
            self.ref_xu_msg.u.layout.dim[0].size = self.N_nmpc

            self.ref_xu_msg.x.data = x_traj.flatten().tolist()
            self.ref_xu_msg.u.data = u_traj.flatten().tolist()

        # Return the reference message (used by pub_trajectory_points in the parent)
        return self.ref_xu_msg

    def check_finished(self, t_elapsed: float) -> bool:
        """
        Return True if we have exceeded the final trajectory time.
        This will cause the base class timer to shut down automatically.
        """
        if t_elapsed > self.x_traj[-1, -2]:
            self.cleanup_traj_viz()
            rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory time finished!")
            return True

        return False

    def cleanup_traj_viz(self):
        """
        Clean up the trajectory visualization in Rviz.
        """
        empty = Path()
        empty.header.frame_id = "world"
        self.traj_path_pub.publish(empty)
