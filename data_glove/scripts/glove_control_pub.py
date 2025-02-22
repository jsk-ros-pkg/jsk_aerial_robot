#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_pub.py
@Author  : Li_JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""

import sys

import rospy
import signal
import socket
import argparse
from typing import Optional
from std_msgs.msg import UInt8
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer


class FingerDataPublisher:
    def __init__(self) -> None:
        """
        Initializes the FingerDataPublisher class, setting up the ROS node
        and the publisher for control mode.
        """
        rospy.init_node("glove_data_pub_node", anonymous=True)
        self.control_mode_pub = rospy.Publisher("hand/control_mode", UInt8, queue_size=10)
        self.last_check_time = None
        self.last_state = None
        self.control_mode = 1

    def publish_control_mode(self, state: int) -> None:
        """
        Publishes the control mode if the state remains unchanged for 3 seconds,
        and always publishes the current control mode.

        Args:
            state (int): The new state to check and potentially publish.
        """
        current_time = rospy.Time.now()

        if state == -1:
            if self.last_state is not None:
                self.last_state = None
                self.last_check_time = None
                print("Invalid state (-1) detected. Resetting timers and state.")

            self.control_mode_pub.publish(self.control_mode)
            return

        if state != self.last_state:
            self.last_state = state
            self.last_check_time = current_time
            print(f"State changed to {state}, resetting timer.")

        if self.last_check_time and (current_time - self.last_check_time).to_sec() > 3.0:
            print(f"State {state} has been stable for 3 seconds. Publishing...")
            self.control_mode = state
            self.control_mode_pub.publish(self.control_mode)
            self.last_check_time = current_time
            print(f"reset control mode to {self.control_mode}")
        self.control_mode_pub.publish(self.control_mode)


def handle_rotation(address: str, *args: float):
    """
    Add the bending data of the two joints of the little finger,
    and then package and send it together with the hand's opening and closing data.

    Args:
        address (str): The OSC address for the rotation event (not used in this function).
        *args (float): A variable number of arguments, expected to include finger data.
    """
    thumb_info = args[5] + args[6] + args[8]
    index_finger_info = args[9] + args[10]
    middle_finger_info = args[12] + args[13]
    ring_finger_info = args[15] + args[16]
    little_finger_info = args[18] + args[19]

    bending_high_threshold = 1.2
    bending_low_threshold = 0.5
    state = -1

    if index_finger_info < bending_low_threshold:
        if not (thumb_info > bending_high_threshold):
            pass
        elif not (middle_finger_info < bending_low_threshold):
            state = 1
        elif not (ring_finger_info < bending_low_threshold):
            state = 2
        elif not (little_finger_info < bending_low_threshold):
            state = 3
        else:
            state = 4

    elif index_finger_info > bending_high_threshold:
        if (
            thumb_info > bending_high_threshold
            and middle_finger_info > bending_high_threshold
            and ring_finger_info > bending_high_threshold
            and little_finger_info > bending_high_threshold
        ):
            state = 5
    finger_publisher.publish_control_mode(state)


def shut_publisher(sig, frame) -> None:
    """
    Handles the shutdown process for the OSC server and ROS node.
    """
    print("\nShutting down the OSC server and ROS node...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


def get_local_ip() -> Optional[str]:
    """
    Retrieve the local IP address of the machine.

    Returns:
        Optional[str]: The local IP address if successfully retrieved;
                       None if an error occurs.
    """
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
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run OSC server for Data Glove")
    parser.add_argument("--port", type=int, default=9400, help="Port number to bind the OSC server")
    args, unknown = parser.parse_known_args()
    local_ip = get_local_ip()

    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, shut_publisher)

    finger_publisher = FingerDataPublisher()

    dispatcher = Dispatcher()
    dispatcher.map("/v1/animation/slider/all", handle_rotation)

    server = BlockingOSCUDPServer((local_ip, args.port), dispatcher)

    print(f"Listening for Data Glove OSC messages on {local_ip}:{args.port} and publishing to ROS topics...")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        shut_publisher(None, None)
