'''
 Created by li-jinjie on 25-1-4.
'''

import os
import sys
import argparse
import rospy
import smach
import smach_ros
import numpy as np
import inspect

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)
from pub_mpc_joint_traj import MPCTrajPtPub, MPCSinglePtPub
from pub_mpc_pred_xu import MPCPubCSVPredXU

from geometry_msgs.msg import Pose, Quaternion, Vector3

import trajs

# Collect all classes inside trajs whose name ends with 'Traj'
traj_cls_list = [
    cls
    for name, cls in inspect.getmembers(trajs, inspect.isclass)
    # optionally ensure the class is defined in trajs and not an imported library
    if cls.__module__ == 'trajs'
       # (Optional) filter by name if you only want classes that end with "Traj"
       and name != "BaseTraj"
]
print(f"Found {len(traj_cls_list)} trajectory classes in trajs module.")

# read all CSV files in the folder ./tilt_qd_csv_trajs
csv_folder_path = os.path.join(current_path, 'tilt_qd_csv_trajs')
csv_files = [f for f in os.listdir(csv_folder_path) if f.endswith('.csv')]
print(f"Found {len(csv_files)} CSV files in ./tilt_qd_csv_trajs folder.")


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
            # print available trajectory types
            print("Available trajectory types:")
            for i, traj_cls in enumerate(traj_cls_list):
                print(f"{i}: {traj_cls.__name__}")

            # print available CSV files
            for i, csv_file in enumerate(csv_files):
                print(f"{i + len(traj_cls_list)}: {csv_file}")

            max_traj_idx = len(traj_cls_list) + len(csv_files) - 1

            traj_type_str = input(f"Enter trajectory type (0..{max_traj_idx}) or 'q' to quit: ")
            if traj_type_str.lower() == "q":
                rospy.signal_shutdown("User requested shutdown.")
                sys.exit(0)

            traj_type = int(traj_type_str)
            if not (0 <= traj_type <= max_traj_idx):
                rospy.logwarn("Invalid trajectory type!")
                return "stay_idle"

            loop_num = 1.0  # TODO: add loop support for CSV trajectories
            if traj_type < len(traj_cls_list):
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

        if userdata.traj_type < len(traj_cls_list):
            rospy.loginfo(f"Using trajs.{traj_cls_list[userdata.traj_type].__name__} trajectory.")
            traj = traj_factory(userdata.traj_type, userdata.loop_num)

            x, y, z, vx, vy, vz, ax, ay, az = traj.get_3d_pt(0.0)

            try:
                (qw, qx, qy, qz, r_rate, p_rate, y_rate, r_acc, p_acc, y_acc) = traj.get_3d_orientation(0.0)
            except AttributeError:
                qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        else:
            csv_file = csv_files[userdata.traj_type - len(traj_cls_list)]
            rospy.loginfo(f"Using CSV file: {csv_file}")
            # csv_traj = np.loadtxt(os.path.join(csv_folder_path, csv_file), delimiter=',', max_rows=1)
            # TODO: change the order of csv file. one row for one point is better.
            csv_traj = np.loadtxt(os.path.join(csv_folder_path, csv_file), delimiter=',')
            x, y, z = csv_traj[0:3, 0]
            qw, qx, qy, qz = csv_traj[6:10, 0]

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
            f"State: TRACK -- Starting for robot={userdata.robot_name}, "
            f"traj_type={userdata.traj_type}, loop_num={userdata.loop_num}"
        )

        if userdata.traj_type < len(traj_cls_list):
            traj = traj_factory(userdata.traj_type, userdata.loop_num)
            rospy.loginfo(f"Using {traj} trajectory.")
            mpc_node = MPCTrajPtPub(robot_name=userdata.robot_name, traj=traj)
        else:
            csv_file = csv_files[userdata.traj_type - len(traj_cls_list)]
            rospy.loginfo(f"Using CSV file: {csv_file}")
            mpc_node = MPCPubCSVPredXU(userdata.robot_name, os.path.join(csv_folder_path, csv_file))

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
