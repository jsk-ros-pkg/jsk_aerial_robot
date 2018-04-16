#!/usr/bin/env python
import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState
from spinal.msg import DesireCoord
from aerial_robot_msgs.msg import FlightNav

#variable
distance = 1
wait_time = 7

rospy.init_node("move_demo")

joint_control_pub = rospy.Publisher("/dragon/joints_ctrl", JointState, queue_size=10)
att_control_pub = rospy.Publisher("/final_desire_tilt", DesireCoord, queue_size=1)

desire_joint = JointState()
desire_joint.position = [-0, -1.57, -0.6, 0.6, 1.57, 1]

desire_att = DesireCoord()
desire_att.pitch = -0.3

time.sleep(0.5)

joint_control_pub.publish(desire_joint)
att_control_pub.publish(desire_att)

time.sleep(10)

nav_control_pub = rospy.Publisher("/uav/nav", FlightNav)

desire_nav = FlightNav()
desire_nav.pos_xy_nav_mode = 2

desire_nav.target_pos_x = distance
desire_nav.target_pos_y = 0


time.sleep(0.5)
nav_control_pub.publish(desire_nav)
desire_nav.target_pos_x = distance
desire_nav.target_pos_y = distance

time.sleep(wait_time)
nav_control_pub.publish(desire_nav)
desire_nav.target_pos_x = 0
desire_nav.target_pos_y = distance


time.sleep(wait_time)
nav_control_pub.publish(desire_nav)
desire_nav.target_pos_x = 0
desire_nav.target_pos_y = 0


time.sleep(wait_time)
nav_control_pub.publish(desire_nav)
