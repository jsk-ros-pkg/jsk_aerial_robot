#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

rospy.init_node("rolling_joint_test")

joint_control_pub = rospy.Publisher("rolling/joints_ctrl", JointState, queue_size=10)
radian = rospy.get_param("~radian", 0)
joint_state_msg = JointState()
joint_state_msg.name = ["joint1", "joint2"]
joint_state_msg.position = [radian, radian]

time.sleep(0.6)
joint_control_pub.publish(joint_state_msg)
