#!/usr/bin/env python3

import rospy
import os
import numpy as np
from aerial_robot_msgs.msg import ControlInput, WrenchAllocationMatrix, PoseControlPid
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, UInt8, Float32
from spinal.msg import FourAxisCommand, TorqueAllocationMatrixInv, FlightConfigCmd
import tf

class mujocoFlightControlInterface:
    def __init__(self):
        # ros publisher
        self.flight_config_ack_pub = rospy.Publisher("flight_config_ack", UInt8, queue_size=1)
        self.battery_voltage_pub = rospy.Publisher("battery_voltage_status", Float32, queue_size=1)

        # ros subscriber
        self.flight_config_cmd_sub = rospy.Subscriber("flight_config_cmd", FlightConfigCmd, self.flightConfigCallback)

        # ros timer
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timerCallback)

    def flightConfigCallback(self, msg):
        if msg.cmd == FlightConfigCmd.ARM_ON_CMD:
            flight_config_ack_msg = UInt8()
            flight_config_ack_msg.data = FlightConfigCmd.ARM_ON_CMD
            self.flight_config_ack_pub.publish(flight_config_ack_msg)

        elif msg.cmd == FlightConfigCmd.ARM_OFF_CMD:
            flight_config_ack_msg = UInt8()
            flight_config_ack_msg.data = FlightConfigCmd.ARM_OFF_CMD
            self.flight_config_ack_pub.publish(flight_config_ack_msg)

    def timerCallback(self, event):
        msg = Float32()
        msg.data = 4.2 * 6
        self.battery_voltage_pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node("mujoco_flight_controller_interface" , anonymous=True)
    node = mujocoFlightControlInterface()
    rospy.spin()
