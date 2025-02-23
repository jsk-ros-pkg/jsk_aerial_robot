#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_sub.py
@Author  : Li_JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""

import rospy
import tkinter as tk
from std_msgs.msg import UInt8


class FingerDataSubscriber:
    def __init__(self) -> None:
        """
        Initializes the FingerDataSubscriber class, setting up the ROS node
        and subscribing to the control mode topic.
        """
        rospy.init_node("glove_data_sub_node", anonymous=True)
        rospy.Subscriber("hand/control_mode", UInt8, self.control_mode_callback)

        self.window = tk.Tk()
        self.window.title("Control Mode Window")
        self.window.geometry("800x800")

        self.state_label = tk.Label(self.window, text="State: 0", font=("Helvetica", 120), fg="white")
        self.state_label.pack(expand=True)

        self.state = 1

    def control_mode_callback(self, msg) -> None:
        """
        Callback function to handle incoming control mode data from the topic.

        Args:
            msg (UInt8): The message containing the control mode state.
        """
        control_mode = msg.data
        self.update_window(control_mode)

    def update_window(self, control_mode: int) -> None:
        """
        Updates the window background color and the displayed number based on the control mode.

        Args:
            control_mode (int): The current state to update the window with.
        """
        color_map = {
            1: "red",
            2: "blue",
            3: "green",
            4: "purple",
            5: "black",
        }

        text_map = {
            1: "State: Mapping Mode",
            2: "State: Cartesian Mode",
            3: "State: Spherical Mode",
            4: "State: Free Mode",
            5: "State: Exit",
        }

        color = color_map.get(control_mode, "white")
        text = text_map.get(control_mode, "State: Unknown")

        self.window.configure(bg=color)
        self.state_label.config(bg=color, text=text)  #

    def start(self) -> None:
        """
        Starts the Tkinter mainloop and listens for ROS messages.
        """
        rospy.loginfo("Waiting for messages on /hand/control_mode")
        self.window.mainloop()


if __name__ == "__main__":
    subscriber = FingerDataSubscriber()
    subscriber.start()
