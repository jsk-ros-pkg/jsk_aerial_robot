#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_pub.py
@Author  : Li_JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""

import time
import rospy
import argparse
from std_msgs.msg import UInt8
from pythonosc.dispatcher import Dispatcher
from pythonosc.osc_server import BlockingOSCUDPServer
import signal
import sys

class FingerDataPublisher:
    def __init__(self):
        rospy.init_node('palm_openness_pub', anonymous=True)
        self.control_mode_pub = rospy.Publisher('hand/control_mode', UInt8, queue_size=10)
        self.last_time_little_finger = None
        self.last_time_openness = None
        self.control_mode = 0

    def publish_control_mode(self, finger_info):
        print(finger_info)
        if finger_info[1] > 0.90:
            self.last_time_little_finger = None
            if self.last_time_openness is None:
                self.last_time_openness = rospy.Time.now()
            elif (rospy.Time.now() - self.last_time_openness).to_sec() > 5:
                self.control_mode = 2
                self.last_time_openness = None
                while True:
                    self.control_mode_pub.publish(self.control_mode)
        else:
            self.last_time_openness = None
            if finger_info[0] > 1.3:
                if self.last_time_little_finger is None:
                    self.last_time_little_finger = rospy.Time.now()
                elif (rospy.Time.now() - self.last_time_little_finger).to_sec() > 2.0:
                    self.control_mode = 1 - self.control_mode
                    self.last_time_little_finger = None
            else:
                self.last_time_little_finger = None
        self.control_mode_pub.publish(self.control_mode)

def handle_rotation(address, *args):
    little_finger_info = args[18] + args[19]
    finger_info = [little_finger_info, args[21]]
    finger_publisher.publish_control_mode(finger_info)

def signal_handler(sig, frame):
    print("\nShutting down the OSC server and ROS node...")
    rospy.signal_shutdown("Ctrl+C pressed")
    sys.exit(0)

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Run OSC server for Data Glove")
    parser.add_argument("--port", type=int, default=9400, help="Port number to bind the OSC server")
    parser.add_argument("--ip", type=str, default="192.168.1.245", help="IP address to bind the OSC server")
    args = parser.parse_args()
    print(args.ip, args.port)

    # Setup signal handler for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)

    finger_publisher = FingerDataPublisher()

    dispatcher = Dispatcher()
    dispatcher.map("/v1/animation/slider/all", handle_rotation)

    # Configure IP address and port number
    server = BlockingOSCUDPServer((args.ip, args.port), dispatcher)

    print(f"Listening for Data Glove OSC messages on {args.ip}:{args.port} and publishing to ROS topics...")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        signal_handler(None, None)
