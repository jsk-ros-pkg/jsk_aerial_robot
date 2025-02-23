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


class FingerDataPublisher:
    def __init__(self) -> None:
        self.control_mode_pub = rospy.Publisher("hand/control_mode", UInt8, queue_size=10)
        self.last_check_time = None
        self.last_state = None
        self.control_mode = 1

    def publish_control_mode(self, state: int) -> None:
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
            print(f"Reset control mode to {self.control_mode}")

        self.control_mode_pub.publish(self.control_mode)

    def handle_rotation(self, address: str, *args: float) -> None:
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

        self.publish_control_mode(state)


def shut_publisher(sig, frame) -> None:
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
    rospy.init_node("glove_data_pub_node", anonymous=True)
    port = int(rospy.get_param("~port", 9400))

    local_ip = get_local_ip()

    signal.signal(signal.SIGINT, shut_publisher)

    finger_publisher = FingerDataPublisher()

    dispatcher = Dispatcher()
    dispatcher.map("/v1/animation/slider/all", finger_publisher.handle_rotation)

    server = BlockingOSCUDPServer((local_ip, port), dispatcher)

    print(f"Listening for Data Glove OSC messages on {local_ip}:{port} and publishing to ROS topics...")

    osc_thread = threading.Thread(target=server.serve_forever)
    osc_thread.daemon = True
    osc_thread.start()

    rospy.spin()
