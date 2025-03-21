#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : glove_control_sub.py
@Author  : Li_JiaXuan
@Date    : 2024-12-04 12:32
@Software: PyCharm
"""
import rosparam
import rospy
import rosgraph.masterapi
import tkinter as tk
from std_msgs.msg import UInt8
import signal


class FingerDataSubscriber:
    def __init__(self) -> None:
        """
        Initializes the FingerDataSubscriber class, setting up the ROS node
        and subscribing to the control mode topic.
        """
        rospy.init_node("glove_data_sub_node", anonymous=True)

        self.window = tk.Tk()
        self.window.title("Control Mode Window")
        self.window.geometry("800x800")

        self.state_label = tk.Label(self.window, text="State: 0", font=("Helvetica", 120), fg="white")
        self.state_label.pack(expand=True)

        # Set up protocol for when the window is closed manually.
        self.window.protocol("WM_DELETE_WINDOW", self.on_close)
        # Set up a custom signal handler for SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.signal_handler)

        # create a timer to check the current control state
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_mode_callback)

    def signal_handler(self, sig, frame):
        rospy.loginfo("Ctrl+C pressed, shutting down...")
        self.on_close()

    def on_close(self):
        """
        Handles shutdown when window is closed or Ctrl+C is pressed.
        """
        rospy.signal_shutdown("Shutdown initiated")
        self.window.destroy()

    def control_mode_callback(self, event):
        """
        Callback function to handle incoming control mode data from the topic.

        Args:
            msg (UInt8): The message containing the control mode state.
        """
        try:
            control_mode = rosparam.get_param("/hand/control_mode")
        except rosgraph.masterapi.MasterError:
            rospy.logwarn_throttle(1, "Failed to get control mode parameter.")
            return

        self.update_window(control_mode)

    def update_window(self, control_mode: int) -> None:
        """
        Updates the window background color and the displayed text based on the control mode.

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
            1: "State: Operation Mode",
            2: "State: Cartesian Mode",
            3: "State: Spherical Mode",
            4: "State: Locking Mode",
            5: "State: Exit",
        }

        color = color_map.get(control_mode, "white")
        text = text_map.get(control_mode, "State: Unknown")

        self.window.configure(bg=color)
        self.state_label.config(bg=color, text=text)

    def start(self) -> None:
        """
        Starts the Tkinter mainloop and periodically checks for ROS shutdown.
        """
        # Schedule periodic check to see if ROS has been shut down.
        self.periodic_check()
        self.window.mainloop()

    def periodic_check(self):
        """
        Periodically checks whether rospy has signaled a shutdown.
        """
        if rospy.is_shutdown():
            self.window.quit()
        else:
            # Check again after 100ms.
            self.window.after(100, self.periodic_check)


if __name__ == "__main__":
    subscriber = FingerDataSubscriber()
    subscriber.start()
