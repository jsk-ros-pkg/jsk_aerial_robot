#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_pub.py
@Author  : Li JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""

import argparse
import signal
import socket
import sys
import threading
from typing import Optional

import rospy
from std_msgs.msg import UInt8
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer


class FingerDataManager:
    def __init__(self) -> None:
        rospy.set_param("/hand/control_mode", 1)
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
            (1, 1, 1, 1, 1): ("gesture_state_5", 5),
        }
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

    def handle_rotation(self, address: str, *args: float) -> None:

        # Get the bending and straight status for all five fingers.
        # Bent is -1, straight is 1, moderately bent is 0

        thumb_info = self.get_finger_status(args[5] + args[6] + args[8],0.3)
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
        state, state_number = self.gesture_to_state_mapping.get(gesture_state_tuple, ("State_no_change", -1))
        self.update_control_mode(state_number)

    def update_control_mode(self, state: int) -> None:
        """ Updates the control mode using ROS parameters. """
        if state == -1:
            if self.last_state is not None:
                # When switching from a valid gesture to an invalid gesture, remind the operator:
                # "Gesture detected as invalid. To change the state, please maintain a valid gesture."
                self.last_state = None
                self.last_check_time = None
                print("Gesture detected as invalid. To change the gesture state, please maintain a valid gesture.")
            return

        current_time = rospy.Time.now()

        # If the state has changed, reset the timer
        if state != self.last_state:
            self.last_state = state
            self.last_check_time = current_time
            print(f"Gesture state changed to {state}, resetting timer.")

        # If the valid state (not -1) has been stable for 3 seconds, update the self.control_mode.
        elif (current_time - self.last_check_time).to_sec() > 3.0:
            print(f"Gesture state_{state} has been stable for 3 seconds.")
            current_mode = rospy.get_param("/hand/control_mode")
            self.last_check_time = None
            print(f"Updated control mode from state_{current_mode} to state_{state}")
            rospy.set_param("/hand/control_mode", state)

def shut_publisher(sig, frame) -> None:
    """ Shuts down the ROS node and OSC server. """
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
    dispatcher.map("/v1/animation/slider/all", finger_manager.handle_rotation)

    server = BlockingOSCUDPServer((local_ip, port), dispatcher)

    print(f"Listening for Data Glove OSC messages on {local_ip}:{port} and updating ROS parameters...")

    osc_thread = threading.Thread(target=server.serve_forever)
    osc_thread.daemon = True
    osc_thread.start()

    rospy.spin()
