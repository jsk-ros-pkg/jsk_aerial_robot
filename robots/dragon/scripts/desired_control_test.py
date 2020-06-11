#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState

rospy.init_node("gimbal_control_test")

from spinal.msg import DesireCoord

att_control_pub = rospy.Publisher("/dragon/final_target_baselink_rot", DesireCoord, queue_size=1)

desire_att = DesireCoord()

time.sleep(0.5)

while not rospy.is_shutdown():

  if desire_att.roll == -1.57:
     desire_att.roll = 0
  else:
    desire_att.roll = -1.57
  att_control_pub.publish(desire_att)

  time.sleep(10)
