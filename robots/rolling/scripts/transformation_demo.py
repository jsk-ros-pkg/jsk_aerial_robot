#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState
from spinal.msg import DesireCoord

rospy.init_node("rolling_transformation_demo")

joint_control_pub = rospy.Publisher("/rolling/joints_ctrl", JointState, queue_size=10)
att_control_pub = rospy.Publisher("/rolling/final_target_baselink_rot", DesireCoord, queue_size=1)

demo_mode = rospy.get_param("~mode", 0)
reset = rospy.get_param("~reset", False)
reverse_reset = rospy.get_param("~reverse_reset", False)

desire_joint = JointState()
desire_att = DesireCoord()

half_pi = 1.56 # the limitation of the joint
pi = math.pi

if demo_mode == 0: # circle
    desire_joint.position = [pi / 3] * 5

elif demo_mode == 1: # linear
    desire_joint.position = [0] * 5

time.sleep(0.6)

joint_control_pub.publish(desire_joint)
att_control_pub.publish(desire_att)

