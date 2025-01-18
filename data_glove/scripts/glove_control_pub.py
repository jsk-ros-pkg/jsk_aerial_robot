#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_pub.py
@Author  : Li_JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""

import time
import sys

import rospy
import argparse
from typing import List
from std_msgs.msg import UInt8
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer

from ip_acquire import get_local_ip


class FingerDataPublisher:
    def __init__(self) -> None:
        """
        Initializes the FingerDataPublisher class, setting up the ROS node
        and the publisher for control mode.
        """
        rospy.init_node("palm_openness_pub", anonymous=True)
        self.control_mode_pub = rospy.Publisher(
            "hand/control_mode", UInt8, queue_size=10
        )
        self.last_time_little_finger = None
        self.last_time_openness = None
        self.control_mode = 0
        self.little_finger_bend_threshold = 1.3
        self.little_finger_time_threshold = 2.0
        self.hand_open_threshold = 0.90
        self.hand_open_time_threshold = 5.0

    def publish_control_mode(self, finger_info: List[float]) -> None:
        """
        Publishes the control mode based on the finger information. This method checks the
        openness and little finger values and sets the control mode accordingly.

        Args:
            finger_info (List[float]): A list containing finger data where:
                                       finger_info[0] is the little finger information
                                       finger_info[1] is the openness value.
        """
        if finger_info[1] > self.hand_open_threshold:
            self.last_time_little_finger = None
            if self.last_time_openness is None:
                self.last_time_openness = rospy.Time.now()
            elif (rospy.Time.now() - self.last_time_openness).to_sec() > self.hand_open_time_threshold:
                self.control_mode = 2
                self.last_time_openness = None
                while True:
                    self.control_mode_pub.publish(self.control_mode)
        else:
            self.last_time_openness = None
            if finger_info[0] > self.little_finger_bend_threshold:
                if self.last_time_little_finger is None:
                    self.last_time_little_finger = rospy.Time.now()
                elif (rospy.Time.now() - self.last_time_little_finger).to_sec() > self.little_finger_time_threshold:
                    self.control_mode = 1 - self.control_mode
                    self.last_time_little_finger = None
            else:
                self.last_time_little_finger = None
        self.control_mode_pub.publish(self.control_mode)


def handle_rotation(address: str, *args: float) -> None:
    """
    Add the bending data of the two joints of the little finger,
    and then package and send it together with the hand's opening and closing data.

    Args:
        address (str): The OSC address for the rotation event (not used in this function).
        *args (float): A variable number of arguments, expected to include finger data.
    """
    little_finger_info = args[18] + args[19]
    finger_info = [little_finger_info, args[21]]
    finger_publisher.publish_control_mode(finger_info)


def shut_publisher():
    """
    Handles the shutdown process for the OSC server and ROS node.
    """
    print("\nShutting down the OSC server and ROS node...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)


if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run OSC server for Data Glove")
    parser.add_argument("--port", type=int, default=9400, help="Port number to bind the OSC server")
    args = parser.parse_args()
    local_ip = get_local_ip()

    rospy.on_shutdown(shut_publisher)

    finger_publisher = FingerDataPublisher()

    dispatcher = Dispatcher()
    dispatcher.map("/v1/animation/slider/all", handle_rotation)

    server = BlockingOSCUDPServer((local_ip, args.port), dispatcher)

    print(f"Listening for Data Glove OSC messages on {local_ip}:{args.port} and publishing to ROS topics...")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        shut_publisher()