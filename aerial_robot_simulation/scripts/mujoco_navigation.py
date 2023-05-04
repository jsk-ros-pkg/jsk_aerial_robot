#!/usr/bin/env python3

import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
from aerial_robot_msgs.msg import ControlInput
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from spinal.msg import FlightConfigCmd
from pid import PI_D

class mujocoNavigator:
    def __init__(self):
        self.nav_state = 1
        self.flight_config_cmd = FlightConfigCmd.ARM_OFF_CMD

        # ros subscriber
        flight_config_sub = rospy.Subscriber("flight_config_cmd", FlightConfigCmd, self.flightConfigCallback)
        takeoff_sub = rospy.Subscriber("teleop_command/takeoff", Empty, self.takeoffCallback)

    def flightConfigCallback(self, msg):
        pass

    def takeoffCallback(self, msg):
        self.setNavState(1)

    def getNavState(self):
        return self.nav_state

    def setNavState(self, state):
        self.nav_state = state

