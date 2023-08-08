#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

rospy.init_node("rolling_gimbal_test")

gimbal_control_pub = rospy.Publisher("rolling/gimbals_ctrl", JointState, queue_size=10)
radian = rospy.get_param("~radian", 0)
joint_state_msg = JointState()
joint_state_msg.name = ["gimbal1", "gimbal2", "gimbal3"]
joint_state_msg.position = [radian, radian, radian]

time.sleep(0.6)
gimbal_control_pub.publish(joint_state_msg)
