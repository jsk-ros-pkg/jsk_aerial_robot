"""
 Created by li-jinjie on 25-1-4.
"""

import os
import sys
import argparse
import rospy
import smach
import smach_ros
import numpy as np
import inspect
import time
import tf.transformations as tft

# Insert current folder into path so we can import from "trajs" or other local files
current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from pub_mpc_joint_traj import MPCTrajPtPub, MPCSinglePtPub

from pub_mpc_pred_xu import MPCPubCSVPredXU

from geometry_msgs.msg import Pose, Quaternion, Vector3

import trajs

from mapping_control.object_position_mapping import (
    Hand,
    Arm,
    Drone,
    Glove,
    MappingMode,
    CartesianMode,
    FreeMode,
    SphericalMode,
)

# Collect all classes inside trajs whose name ends with 'Traj'
traj_cls_list = [
    cls
    for name, cls in inspect.getmembers(trajs, inspect.isclass)
    # optionally ensure the class is defined in trajs and not an imported library
    if cls.__module__ == "trajs"
    # (Optional) filter by name if you only want classes that end with "Traj"
    and name != "BaseTraj"
]
print(f"Found {len(traj_cls_list)} trajectory classes in trajs module.")

# read all CSV files in the folder ./tilt_qd_csv_trajs
csv_folder_path = os.path.join(current_path, "tilt_qd_csv_trajs")
csv_files = [f for f in os.listdir(csv_folder_path) if f.endswith(".csv")]
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
    - Also need to prompt user for mapping_config, if user use mapping control.
    - On valid input, go INIT; otherwise, stay in IDLE.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=["go_init", "stay_idle", "shutdown", "go_mapping_init"],
            input_keys=["mapping_config"],
            output_keys=["robot_name", "traj_type", "loop_num", "mapping_config"],
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

            # print an available hand control state
            print("h :hand-based control")

            max_traj_idx = len(traj_cls_list) + len(csv_files) - 1

            traj_type_str = input(f"Enter trajectory type (0..{max_traj_idx}) or 'q' to quit or 'h' to hand control: ")
            if traj_type_str.lower() == "q":
                return "shutdown"

            if traj_type_str.lower() == "h":
                userdata.traj_type = "h"
                userdata.mapping_config = {"is_arm_active": False, "is_glove_active": False}

                devices = {"is_arm_active": "Arm mocap", "is_glove_active": "Glove"}
                print("Whether activate the following devices. ([Y]/Yes or [N]/No, case-insensitive)")
                for mapping_config_key, device in devices.items():
                    while True:
                        user_input = input(f"Whether activate {device} ([Y]/[N]): ").strip().lower()
                        if user_input == "y":
                            userdata.mapping_config[mapping_config_key] = True
                            rospy.loginfo(f"Decide to activate {device}.")
                            break
                        elif user_input == "n":
                            userdata.mapping_config[mapping_config_key] = False
                            rospy.loginfo(f"Decided not to activate {device}.")
                            break
                        else:
                            rospy.logwarn("Invalid input. Please enter Y or N (case-insensitive).")

                return "go_mapping_init"

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
            csv_traj = np.loadtxt(os.path.join(csv_folder_path, csv_file), delimiter=",")
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


class InitObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_lock"], input_keys=["robot_name", "mapping_config"], output_keys=[])

    def execute(self, userdata):

        # userdata.mapping_config = {"is_arm_active": True, "is_glove_active": True}
        try:
            if userdata.mapping_config.get("is_arm_active", False):
                shared_data["arm"] = Arm()
                rospy.loginfo(f"The arm mocap has been successfully activated.")
            if userdata.mapping_config.get("is_glove_active", False):
                shared_data["control_mode"] = Glove()
                rospy.loginfo(f"The data glove has been successfully activated.")

            shared_data["hand"] = Hand()
            rospy.loginfo(f"The hand mocap has been successfully activated.")
            shared_data["drone"] = Drone(userdata.robot_name)
            rospy.loginfo(f"The drone's position has been successfully activated.")
            return "go_lock"
        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            return "error"


class LockState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_unlock"], input_keys=[], output_keys=[])

        self.last_threshold_time = None
        self.xy_angle_threshold = 10
        self.z_angle_threshold = 45
        self.direction_hold_time = 2
        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        global shared_data

        rospy.loginfo("Current state: Lock")
        rospy.loginfo("Please align the direction of your hand with the drone's hand direction.")

        while not rospy.is_shutdown():
            drone_orientation = [
                shared_data["drone"].position.pose.pose.orientation.x,
                shared_data["drone"].position.pose.pose.orientation.y,
                shared_data["drone"].position.pose.pose.orientation.z,
                shared_data["drone"].position.pose.pose.orientation.w,
            ]
            hand_orientation = [
                shared_data["hand"].position.pose.orientation.x,
                shared_data["hand"].position.pose.orientation.y,
                shared_data["hand"].position.pose.orientation.z,
                shared_data["hand"].position.pose.orientation.w,
            ]

            q_drone_inv = tft.quaternion_inverse(drone_orientation)
            q_relative = tft.quaternion_multiply(hand_orientation, q_drone_inv)
            euler_angles = np.degrees(tft.euler_from_quaternion(q_relative))
            sys.stdout.write(
                f"Deviation:Roll: {euler_angles[0]:6.1f}, Pitch: {euler_angles[1]:6.1f}, Yaw: {euler_angles[2]:6.1f}\r"
            )
            sys.stdout.flush()

            is_x_angular_alignment = abs(euler_angles[0]) < self.xy_angle_threshold
            is_y_angular_alignment = abs(euler_angles[1]) < self.xy_angle_threshold
            is_z_angular_alignment = abs(euler_angles[2]) < self.z_angle_threshold

            is_in_thresh = is_x_angular_alignment and is_y_angular_alignment and is_z_angular_alignment

            if not is_in_thresh:
                self.last_threshold_time = None

            if is_in_thresh:
                if self.last_threshold_time is None:
                    self.last_threshold_time = rospy.get_time()

                if rospy.get_time() - self.last_threshold_time > self.direction_hold_time:
                    break

            self.rate.sleep()

        rospy.loginfo("")
        rospy.loginfo("Current state: Unlock")
        return "go_unlock"


class UnlockState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_mapping_mode"])

    def execute(self, userdata):
        print("Current state: mapping_mode")
        time.sleep(0.5)
        return "go_mapping_mode"


class MappingModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_cartesian_mode", "go_spherical_mode", "go_free_mode", "done_track"],
            input_keys=["robot_name"],
            output_keys=[],
        )

        self.pub_object = None

        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        if self.pub_object is None:
            self.pub_object = MappingMode(
                userdata.robot_name,
                hand=shared_data["hand"],
                arm=shared_data["arm"],
                control_mode=shared_data["control_mode"],
            )

        while not rospy.is_shutdown():
            if self.pub_object.check_finished():
                break
            self.rate.sleep()

        control_mode_state = self.pub_object.get_control_mode()

        del self.pub_object
        self.pub_object = None
        if control_mode_state == 2:
            return "go_cartesian_mode"
        if control_mode_state == 3:
            return "go_spherical_mode"
        if control_mode_state == 4:
            return "go_free_mode"
        if control_mode_state == 5:
            return "done_track"


class SphericalModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_mapping_mode", "go_cartesian_mode", "go_free_mode", "done_track"],
            input_keys=["robot_name"],
            output_keys=[],
        )

        self.pub_object = None

        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        if self.pub_object is None:
            self.pub_object = SphericalMode(
                userdata.robot_name,
                hand=shared_data["hand"],
                arm=shared_data["arm"],
                control_mode=shared_data["control_mode"],
            )

        while not rospy.is_shutdown():
            if self.pub_object.check_finished():
                break
            self.rate.sleep()

        control_mode_state = self.pub_object.get_control_mode()

        del self.pub_object
        self.pub_object = None
        if control_mode_state == 1:
            return "go_mapping_mode"
        if control_mode_state == 2:
            return "go_cartesian_mode"
        if control_mode_state == 4:
            return "go_free_mode"
        if control_mode_state == 5:
            return "done_track"


class CartesianModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_mapping_mode", "go_spherical_mode", "go_free_mode", "done_track"],
            input_keys=["robot_name"],
            output_keys=[],
        )

        self.pub_object = None

        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        if self.pub_object is None:
            self.pub_object = CartesianMode(
                userdata.robot_name,
                hand=shared_data["hand"],
                arm=shared_data["arm"],
                control_mode=shared_data["control_mode"],
            )

        while not rospy.is_shutdown():
            if self.pub_object.check_finished():
                break
            self.rate.sleep()

        control_mode_state = self.pub_object.get_control_mode()

        del self.pub_object
        self.pub_object = None
        if control_mode_state == 1:
            return "go_mapping_mode"
        if control_mode_state == 3:
            return "go_spherical_mode"
        if control_mode_state == 4:
            return "go_free_mode"
        if control_mode_state == 5:
            return "done_track"


class FreeModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_mapping_mode", "go_spherical_mode", "go_cartesian_mode", "done_track"],
            input_keys=["robot_name"],
            output_keys=[],
        )

        self.pub_object = None

        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        if self.pub_object is None:
            self.pub_object = FreeMode(
                userdata.robot_name,
                hand=shared_data["hand"],
                arm=shared_data["arm"],
                control_mode=shared_data["control_mode"],
            )

        while not rospy.is_shutdown():
            if self.pub_object.check_finished():
                break
            self.rate.sleep()

        control_mode_state = self.pub_object.get_control_mode()

        del self.pub_object
        self.pub_object = None
        if control_mode_state == 1:
            return "go_mapping_mode"
        if control_mode_state == 2:
            return "go_cartesian_mode"
        if control_mode_state == 3:
            return "go_spherical_mode"
        if control_mode_state == 5:
            return "done_track"


def create_hand_control_state_machine():
    """HandControlStateMachine"""
    sm_sub = smach.StateMachine(outcomes=["DONE"], input_keys=["robot_name", "mapping_config"])

    with sm_sub:
        # InitObjectState
        smach.StateMachine.add(
            "HAND_CONTROL_INIT",
            InitObjectState(),
            transitions={"go_lock": "LOCK"},
        )

        # LockState
        smach.StateMachine.add(
            "LOCK",
            LockState(),
            transitions={"go_unlock": "UNLOCK"},
        )

        # UnlockState
        smach.StateMachine.add("UNLOCK", UnlockState(), transitions={"go_mapping_mode": "MAPPING_MODE"})

        smach.StateMachine.add(
            "MAPPING_MODE",
            MappingModeState(),
            transitions={
                "go_cartesian_mode": "CARTESIAN_MODE",
                "go_spherical_mode": "SPHERICAL_MODE",
                "go_free_mode": "FREE_MODE",
                "done_track": "DONE",
            },
        )
        smach.StateMachine.add(
            "SPHERICAL_MODE",
            SphericalModeState(),
            transitions={
                "go_cartesian_mode": "CARTESIAN_MODE",
                "go_mapping_mode": "MAPPING_MODE",
                "go_free_mode": "FREE_MODE",
                "done_track": "DONE",
            },
        )
        smach.StateMachine.add(
            "CARTESIAN_MODE",
            CartesianModeState(),
            transitions={
                "go_mapping_mode": "MAPPING_MODE",
                "go_spherical_mode": "SPHERICAL_MODE",
                "go_free_mode": "FREE_MODE",
                "done_track": "DONE",
            },
        )
        smach.StateMachine.add(
            "FREE_MODE",
            FreeModeState(),
            transitions={
                "go_mapping_mode": "MAPPING_MODE",
                "go_spherical_mode": "SPHERICAL_MODE",
                "go_cartesian_mode": "CARTESIAN_MODE",
                "done_track": "DONE",
            },
        )

    return sm_sub


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

    global shared_data
    shared_data = {"hand": None, "arm": None, "drone": None, "control_mode": None}

    # Open the container
    with sm:
        # IDLE
        smach.StateMachine.add(
            "IDLE",
            IdleState(),
            transitions={
                "go_init": "INIT",
                "stay_idle": "IDLE",
                "shutdown": "DONE",
                "go_mapping_init": "HAND_CONTROL",
            },
        )

        # INIT
        smach.StateMachine.add("INIT", InitState(), transitions={"go_track": "TRACK"})

        # TRACK
        smach.StateMachine.add("TRACK", TrackState(), transitions={"done_track": "IDLE"})

        smach.StateMachine.add(
            "HAND_CONTROL",
            create_hand_control_state_machine(),
            transitions={"DONE": "IDLE"},
            remapping={"robot_name": "robot_name", "mapping_config": "mapping_config"},
        )

    # (Optional) Start an introspection server to visualize SMACH in smach_viewer
    sis = smach_ros.IntrospectionServer("mpc_smach_introspection", sm, "/MPC_SMACH")
    sis.start()

    # Execute the state machine
    sm.execute()

    sis.stop()


if __name__ == "__main__":
    main()
