'''
 Refactored by li-jinjie on 25-3-19.
'''
import os
import sys
import rospy
import smach
import numpy as np
import tf.transformations as tft

current_path = os.path.abspath(os.path.dirname(__file__))
if current_path not in sys.path:
    sys.path.insert(0, current_path)

from hand_control.sub_pos_objects import (
    Hand,
    Arm,
    Drone,
    Glove
)

from hand_control.hand_ctrl_modes import (
    OperationMode,
    CartesianMode,
    LockingMode,
    SphericalMode
)

# global variables
shared_data = {"hand": None, "arm": None, "drone": None, "control_mode": None}


class InitObjectState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_wait", "done_track"], input_keys=["robot_name"], output_keys=[])

    @staticmethod
    def get_user_decision(device_name):
        prompt = f"Activate {device_name}? ([Y]/[N]): "
        while True:
            user_input = input(prompt).strip().lower()
            if user_input in ("y", "n"):
                return user_input == "y"
            rospy.logwarn("Invalid input. Please enter Y or N.")

    def execute(self, userdata):
        try:
            shared_data["hand"] = Hand()
            rospy.loginfo("Hand mocap activated.")

            shared_data["drone"] = Drone(userdata.robot_name)
            rospy.loginfo("Drone activated.")

            if self.get_user_decision("Arm mocap"):
                shared_data["arm"] = Arm()
                rospy.loginfo("Arm mocap activated.")

            if self.get_user_decision("Glove"):
                shared_data["control_mode"] = Glove()
                rospy.loginfo("Glove activated.")

            return "go_wait"

        except Exception as e:
            rospy.logerr(f"Initialization failed: {e}")
            return "done_track"


class WaitState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_start"], input_keys=[], output_keys=[])

        self.last_threshold_time = None
        self.xy_angle_threshold = 10
        self.z_angle_threshold = 45
        self.direction_hold_time = 2
        self.rate = rospy.Rate(20)

    def execute(self, userdata):
        global shared_data

        rospy.loginfo("Current state: Wait")
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
        rospy.loginfo("Current state: Start")
        return "go_start"


class StartState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=["go_operation_mode"])

    def execute(self, userdata):
        print("Current state: operation_mode")
        rospy.sleep(0.5)
        return "go_operation_mode"


class OperationModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_cartesian_mode", "go_spherical_mode", "go_locking_mode", "done_track"],
            input_keys=["robot_name"],
            output_keys=[],
        )

        self.pub_object = None

        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        if self.pub_object is None:
            self.pub_object = OperationMode(
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
            return "go_locking_mode"
        if control_mode_state == 5:
            return "done_track"


class SphericalModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_operation_mode", "go_cartesian_mode", "go_locking_mode", "done_track"],
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
            return "go_operation_mode"
        if control_mode_state == 2:
            return "go_cartesian_mode"
        if control_mode_state == 4:
            return "go_locking_mode"
        if control_mode_state == 5:
            return "done_track"


class CartesianModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_operation_mode", "go_spherical_mode", "go_locking_mode", "done_track"],
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
            return "go_operation_mode"
        if control_mode_state == 3:
            return "go_spherical_mode"
        if control_mode_state == 4:
            return "go_locking_mode"
        if control_mode_state == 5:
            return "done_track"


class LockingModeState(smach.State):
    def __init__(self) -> None:

        smach.State.__init__(
            self,
            outcomes=["go_operation_mode", "go_spherical_mode", "go_cartesian_mode", "done_track"],
            input_keys=["robot_name"],
            output_keys=[],
        )

        self.pub_object = None

        self.rate = rospy.Rate(20)

    def execute(self, userdata):

        if self.pub_object is None:
            self.pub_object = LockingMode(
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
            return "go_operation_mode"
        if control_mode_state == 2:
            return "go_cartesian_mode"
        if control_mode_state == 3:
            return "go_spherical_mode"
        if control_mode_state == 5:
            return "done_track"


def create_hand_control_state_machine():
    """HandControlStateMachine"""
    sm_sub = smach.StateMachine(outcomes=["DONE"], input_keys=["robot_name"])

    with sm_sub:
        # InitObjectState
        smach.StateMachine.add(
            "HAND_CONTROL_INIT",
            InitObjectState(),
            transitions={"go_wait": "WAIT",
                         "done_track": "DONE"},
        )

        # WaitState
        smach.StateMachine.add(
            "WAIT",
            WaitState(),
            transitions={"go_start": "START"},
        )

        # StartState
        smach.StateMachine.add("START", StartState(), transitions={"go_operation_mode": "OPERATION_MODE"})

        smach.StateMachine.add(
            "OPERATION_MODE",
            OperationModeState(),
            transitions={
                "go_cartesian_mode": "CARTESIAN_MODE",
                "go_spherical_mode": "SPHERICAL_MODE",
                "go_locking_mode": "LOCKING_MODE",
                "done_track": "DONE",
            },
        )
        smach.StateMachine.add(
            "SPHERICAL_MODE",
            SphericalModeState(),
            transitions={
                "go_cartesian_mode": "CARTESIAN_MODE",
                "go_operation_mode": "OPERATION_MODE",
                "go_locking_mode": "LOCKING_MODE",
                "done_track": "DONE",
            },
        )
        smach.StateMachine.add(
            "CARTESIAN_MODE",
            CartesianModeState(),
            transitions={
                "go_operation_mode": "OPERATION_MODE",
                "go_spherical_mode": "SPHERICAL_MODE",
                "go_locking_mode": "LOCKING_MODE",
                "done_track": "DONE",
            },
        )
        smach.StateMachine.add(
            "LOCKING_MODE",
            LockingModeState(),
            transitions={
                "go_operation_mode": "OPERATION_MODE",
                "go_spherical_mode": "SPHERICAL_MODE",
                "go_cartesian_mode": "CARTESIAN_MODE",
                "done_track": "DONE",
            },
        )

    return sm_sub
