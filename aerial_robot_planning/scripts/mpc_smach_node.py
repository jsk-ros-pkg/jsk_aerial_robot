'''
 Created by li-jinjie on 25-1-4.
'''

import os
import sys
import time
import argparse
import rospy
import smach
import smach_ros
import numpy as np

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)
from mpc_pt_pub import MPCTrajPtPub, MPCSinglePtPub

from trajs import (
    SetPointTraj,
    CircleTraj,
    LemniscateTraj,
    LemniscateTrajOmni,
    PitchRotationTraj,
    RollRotationTraj,
    PitchSetPtTraj,
    PitchRotationTrajOpposite,
    VerticalYawRotationTraj,
)

import tf_conversions as tf
from geometry_msgs.msg import Pose, Quaternion, Vector3

traj_cls_list = [
    SetPointTraj,
    CircleTraj,
    LemniscateTraj,
    LemniscateTrajOmni,
    PitchRotationTraj,
    RollRotationTraj,
    PitchSetPtTraj,
    PitchRotationTrajOpposite,
    VerticalYawRotationTraj,
]


def traj_factory(traj_type, loop_num):
    if traj_type not in range(len(traj_cls_list)):
        raise ValueError("Invalid trajectory type!")

    return traj_cls_list[traj_type](loop_num)


###############################################
# SMACH States
###############################################
class IdleState(smach.State):
    """
    IDLE State:
    - Prompt user for robot_name, traj_type, loop_num.
    - On valid input, go INIT; otherwise, stay in IDLE.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_init", "stay_idle"],
            input_keys=[],
            output_keys=["robot_name", "traj_type", "loop_num"],
        )

    def execute(self, userdata):
        rospy.loginfo("State: IDLE -- Waiting for user input...")

        try:
            # robot_name = input("Enter robot name (e.g., 'beetle1'): ")
            # if not robot_name:
            #     rospy.logwarn("Empty robot name! Staying in IDLE.")
            #     return "stay_idle"

            # print available trajectory types
            print("Available trajectory types:")
            for i, traj_cls in enumerate(traj_cls_list):
                print(f"{i}: {traj_cls.__name__}")

            traj_type_str = input(f"Enter trajectory type (0..{len(traj_cls_list)}) or 'q' to quit: ")
            if traj_type_str.lower() == "q":
                rospy.signal_shutdown("User requested shutdown.")
                sys.exit(0)

            traj_type = int(traj_type_str)
            if not (0 <= traj_type <= len(traj_cls_list)):
                rospy.logwarn("Invalid trajectory type!")
                return "stay_idle"

            loop_str = input("Enter loop number (or press Enter for infinite): ")
            if loop_str.strip() == "":
                loop_num = np.inf
            else:
                loop_num = float(loop_str)

            # Set user data
            # userdata.robot_name = robot_name
            userdata.traj_type = traj_type
            userdata.loop_num = loop_num

            return "go_init"

        except ValueError:
            rospy.logwarn("Invalid input. Please enter a valid number!")
            return "stay_idle"
        except EOFError:
            rospy.logwarn("No input detected (EOF). Staying in IDLE.")
            return "stay_idle"


class InitState(smach.State):
    """
    INIT State:
    - Sleep for 5 seconds to simulate “initialization”.
    - Then go to TRACK.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_track"],
            input_keys=["robot_name", "traj_type", "loop_num"],
            output_keys=[],
        )
        self.rate = rospy.Rate(20)  # 20 Hz loop checking if finished

    def execute(self, userdata):
        rospy.loginfo("State: INIT -- Start to reach the first point of the trajectory.")

        # traj
        traj = traj_factory(userdata.traj_type, userdata.loop_num)

        # position
        x, y, z, vx, vy, vz, ax, ay, az = traj.get_3d_pt(0.0)

        # orientation
        try:
            (qw, qx, qy, qz, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc) = traj.get_3d_orientation(0.0)
        except AttributeError:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
            r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
            r_acc, p_acc, y_acc = 0.0, 0.0, 0.0

        init_pose = Pose(
            position=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
        )

        # Create the node instance
        mpc_node = MPCSinglePtPub(userdata.robot_name, init_pose)

        # Wait here until the node signals it is finished or ROS shuts down
        while not rospy.is_shutdown():
            if mpc_node.finished:
                rospy.loginfo("INIT: MPCSinglePtPub says the init pose is reached.")
                break
            self.rate.sleep()

        rospy.loginfo("Initialization done. Going to TRACK state.")
        return "go_track"


class TrackState(smach.State):
    """
    TRACK State:
    - Create an MPCPtPubNode with the user’s chosen parameters.
    - Keep running until MPCPtPubNode says it's finished or ROS is shutdown.
    - Then return "done_track".
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["done_track"],
            input_keys=["robot_name", "traj_type", "loop_num"],
            output_keys=[],
        )
        self.rate = rospy.Rate(20)  # 20 Hz loop check if finished

    def execute(self, userdata):
        rospy.loginfo(
            f"State: TRACK -- Starting MPCPtPubNode for robot={userdata.robot_name}, "
            f"traj_type={userdata.traj_type}, loop_num={userdata.loop_num}"
        )

        # traj
        traj = traj_factory(userdata.traj_type, userdata.loop_num)

        # Create the node instance
        mpc_node = MPCTrajPtPub(
            robot_name=userdata.robot_name,
            traj=traj,
        )

        # Wait here until the node signals it is finished or ROS shuts down
        while not rospy.is_shutdown():
            if mpc_node.finished:
                rospy.loginfo("TRACK: MPCPtPubNode says the trajectory is finished.")
                break
            self.rate.sleep()

        rospy.loginfo("TRACK: Done tracking. Going back to IDLE.")
        return "done_track"


###############################################
# Main SMACH Entry Point
###############################################
def main():
    parser = argparse.ArgumentParser(description="SMACH-based MPC Trajectory Publisher")
    parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")
    args = parser.parse_args()

    # Initialize a single ROS node for the entire SMACH-based system
    rospy.init_node("mpc_smach_node")

    # Create a top-level SMACH state machine
    sm = smach.StateMachine(outcomes=["DONE"])

    # Declare user data fields
    sm.userdata.robot_name = args.robot_name
    sm.userdata.traj_type = None
    sm.userdata.loop_num = np.inf

    # Open the container
    with sm:
        # IDLE
        smach.StateMachine.add(
            "IDLE",
            IdleState(),
            transitions={
                "go_init": "INIT",
                "stay_idle": "IDLE",
            },
        )

        # INIT
        smach.StateMachine.add(
            "INIT",
            InitState(),
            transitions={
                "go_track": "TRACK",
            },
        )

        # TRACK
        smach.StateMachine.add(
            "TRACK",
            TrackState(),
            transitions={
                "done_track": "IDLE",
            },
        )

    # (Optional) Start an introspection server to visualize SMACH in smach_viewer
    sis = smach_ros.IntrospectionServer("mpc_smach_introspection", sm, "/MPC_SMACH")
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    main()
