#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

rospy.init_node("zero_joint_0_1_2_3")
pub = rospy.Publisher("/gimbalrotor/gimbals_ctrl", JointState, queue_size=1)
radian = rospy.get_param("~radian", 0)

msg = JointState()
msg.name = ["gimbal1", "gimbal2", "gimbal3", "gimbal4"]
msg.position = [radian, radian, radian, radian]

time.sleep(0.6)
pub.publish(msg)
