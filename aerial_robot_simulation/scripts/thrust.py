#!/usr/bin/env python3

import time
import rospy
from aerial_robot_msgs.msg import ControlInput

rospy.init_node("init_pose")
control_input_pub = rospy.Publisher("/uav/ctrl_input", ControlInput, queue_size=1)
control_input = ControlInput()
control_input.name = ['rotor1_thrust', 'rotor2_thrust', 'rotor3_thrust', 'rotor4_thrust']
control_input.input = [18, 18, 18, 18]

time.sleep(1.0)
control_input_pub.publish(control_input)
