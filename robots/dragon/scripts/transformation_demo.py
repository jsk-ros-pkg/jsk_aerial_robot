#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState
from spinal.msg import DesireCoord

rospy.init_node("dragon_transformation_demo")

joint_control_pub = rospy.Publisher("/dragon/joints_ctrl", JointState, queue_size=10)
att_control_pub = rospy.Publisher("/dragon/final_target_baselink_rot", DesireCoord, queue_size=1)

demo_mode = rospy.get_param("~mode", 0)
reset = rospy.get_param("~reset", False)
reverse_reset = rospy.get_param("~reverse_reset", False)

desire_joint = JointState()
desire_att = DesireCoord()

half_pi = 1.56 # the limitation of the joint

if demo_mode == 0: # sea horse
    desire_joint.position = [-0.6, -half_pi, -0.6, 0, 0, half_pi]
    desire_att.pitch = -0.3
elif demo_mode == 1: # spiral model
    desire_joint.position = [-0.8, 1.2, -0.8, 1.2, -0.8, 1.2]
    desire_att.pitch = -0.4
    desire_att.roll = 0.4
elif demo_mode == 2: # mode model
    desire_joint.position = [0.0, half_pi, -1.5, 0.0, 0.0, half_pi]
    desire_att.pitch = 0.75
else:
    sys.exit()

if reset:
    desire_joint.position = [0.0, half_pi, 0, half_pi, 0.0, half_pi]
    desire_att.pitch = 0
    desire_att.roll = 0

if reverse_reset:
    desire_joint.position = [0.0, -half_pi, 0, -half_pi, 0.0, -half_pi]
    desire_att.pitch = 0
    desire_att.roll = 0

time.sleep(0.6)

joint_control_pub.publish(desire_joint)
att_control_pub.publish(desire_att)

