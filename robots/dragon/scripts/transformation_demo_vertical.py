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
#reset = rospy.get_param("~reset", False)
#reverse_reset = rospy.get_param("~reverse_reset", False)

desire_joint = JointState()
desire_att = DesireCoord()

half_pi = 1.56 # the limitation of the joint

if demo_mode == 0:
  desire_joint.name = ["joint3_yaw"]
  desire_joint.position = [-half_pi]

elif demo_mode == 1:
  desire_joint.name = ["joint2_yaw"]
  desire_joint.position = [0]

elif demo_mode == 2: # mode model
  desire_joint.name = ["joint2_yaw", "joint1_yaw"]
  desire_joint.position = [-half_pi, -half_pi]
else:
  sys.exit()

time.sleep(0.6)

joint_control_pub.publish(desire_joint)


