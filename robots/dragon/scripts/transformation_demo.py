#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState
from aerial_robot_base.msg import DesireCoord

rospy.init_node("dragon_transformation_demo")

joint_control_pub = rospy.Publisher("/dragon/joints_ctrl", JointState, queue_size=10)
att_control_pub = rospy.Publisher("/final_desire_tilt", DesireCoord, queue_size=1)

desire_joint = JointState()
desire_joint.position = [-0.6, -1.57, -0.6, 0, 0, 1.57]

desire_att = DesireCoord()
desire_att.pitch = -0.3

time.sleep(0.5)

joint_control_pub.publish(desire_joint)
att_control_pub.publish(desire_att)

