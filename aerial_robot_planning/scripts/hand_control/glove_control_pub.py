#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_pub.py
@Author  : Li JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""

import signal
import socket
import sys
import threading
from typing import Optional

import rospy
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer


class FingerDataManager:
    def __init__(self) -> None:
        self.set_current_mode(5)

        self.last_check_time = None
        self.last_state = None

        # Thresholds for finger bending: high bending (>1.2) and low bending (<0.4)
        self.bending_high_threshold = 1.2
        self.bending_low_threshold = 0.4

        # Mapping of gestures to corresponding states
        # gesture state 1: Only index finger extended
        # gesture state 2: Index and middle fingers extended
        # gesture state 3: Index, middle, and ring fingers extended
        # gesture state 4: Index, middle, ring, and little fingers extended
        # gesture state 5: All fingers extended
        self.gesture_to_state_mapping = {
            (-1, 1, -1, -1, -1): ("gesture_state_1", 1),
            (-1, 1, 1, -1, -1): ("gesture_state_2", 2),
            (-1, 1, 1, 1, -1): ("gesture_state_3", 3),
            (-1, 1, 1, 1, 1): ("gesture_state_4", 4),
            (1, -1, -1, -1, -1): ("gesture_state_5", 5),
        }

    @staticmethod
    def get_current_mode() -> int:
        return rospy.get_param("/hand/control_mode")

    @staticmethod
    def set_current_mode(new_mode: int) -> None:
        rospy.set_param("/hand/control_mode", new_mode)

    def get_finger_status(self, finger_info: float, threshold_plus: float = 0) -> int:
        # Returns the status of the finger based on its bending information
        # If the bending is greater than the high threshold, the finger is considered bent (-1)
        # If the bending is below the low threshold, the finger is considered straight (1)
        # If the bending is in between, the finger is considered moderately bent (0)
        if finger_info > self.bending_high_threshold + threshold_plus:
            return -1  # Finger is bent, status is -1
        elif finger_info < self.bending_low_threshold:
            return 1  # Finger is straight, status is 1
        else:
            return 0  # Finger is moderately bent, status is 0

    def process_glove_data(self, address: str, *args: float) -> None:
        if self.get_current_mode() == 5:
            # If the control mode is 5 (all fingers extended), do not update the control mode
            return

        # Get the bending and straight status for all five fingers.
        # Bent is -1, straight is 1, moderately bent is 0

        thumb_info = self.get_finger_status(args[5] + args[6] + args[8], 0.3)
        index_finger_info = self.get_finger_status(args[9] + args[10])
        middle_finger_info = self.get_finger_status(args[12] + args[13])
        ring_finger_info = self.get_finger_status(args[15] + args[16])
        little_finger_info = self.get_finger_status(args[18] + args[19])

        gesture_state_tuple = (
            thumb_info,
            index_finger_info,
            middle_finger_info,
            ring_finger_info,
            little_finger_info,
        )

        # If the gesture is not valid, get the gesture state number is -1.
        gesture_state_text, gesture_state_num = self.gesture_to_state_mapping.get(
            gesture_state_tuple, ("State_no_change", -1)
        )
        self.update_control_mode(gesture_state_num)

    def update_control_mode(self, gesture_state_num: int) -> None:
        """Updates the control mode using ROS parameters."""
        if gesture_state_num == -1:
            self.last_state = self.get_current_mode()
            self.last_check_time = None
            return

        # If the state has changed, reset the timer
        if gesture_state_num != self.last_state:
            self.last_state = gesture_state_num
            self.last_check_time = rospy.Time.now()
            rospy.loginfo(f"Gesture state changed to {gesture_state_num}, resetting timer.")
            return

        # TODO: simplify the logic
        if self.last_check_time is None:  # if the user choose the same mode again, the last_check_time is None.
            self.last_check_time = rospy.Time.now()

        # If the valid state (not -1) has been stable for 3 seconds, update the self.control_mode.
        current_time = rospy.Time.now()
        if (current_time - self.last_check_time).to_sec() > 3.0:
            rospy.loginfo(f"Gesture state_{gesture_state_num} has been stable for 3 seconds.")
            rospy.loginfo(f"Updated control mode: state_{self.get_current_mode()} -> state_{gesture_state_num}")
            self.set_current_mode(gesture_state_num)
            self.last_check_time = current_time


def shut_publisher(sig, frame) -> None:
    """Shuts down the ROS node and OSC server."""
    print("\nShutting down the OSC server and ROS node...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def get_local_ip() -> Optional[str]:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(("8.8.8.8", 80))
            local_ip = sock.getsockname()[0]
        if not local_ip:
            raise ValueError("Local IP address could not be retrieved.")
        return local_ip
    except Exception as error:
        print(f"Error retrieving local IP address: {error}")
        raise SystemExit("Exiting program due to error.")


if __name__ == "__main__":
    rospy.init_node("glove_data_param_node", anonymous=True)
    port = int(rospy.get_param("~port", 9400))

    local_ip = get_local_ip()
    signal.signal(signal.SIGINT, shut_publisher)

    finger_manager = FingerDataManager()

    dispatcher = Dispatcher()
    dispatcher.map("/v1/animation/slider/all", finger_manager.process_glove_data)

    server = BlockingOSCUDPServer((local_ip, port), dispatcher)

    print(f"Listening for Data Glove OSC messages on {local_ip}:{port} and updating ROS parameters...")

    osc_thread = threading.Thread(target=server.serve_forever)
    osc_thread.daemon = True
    osc_thread.start()

    rospy.spin()
