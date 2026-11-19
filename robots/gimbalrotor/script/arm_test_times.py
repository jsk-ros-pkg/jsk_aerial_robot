#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

rospy.init_node("gimbal_test_sequence")

pub = rospy.Publisher("/gimbalrotor/joints_ctrl", JointState, queue_size=1)

# 正の整数（デフォルト1）
test_times = rospy.get_param("~test_times", 1)

if test_times <= 0:
    rospy.logerr("~test_times must be a positive integer")
    exit(1)

msg = JointState()
msg.name = [
        "arm_joint1_joint1",
        "arm_joint2_joint1",
        "arm_joint3_joint1",
        "arm_joint4_joint1",
]

# 繰り返すシーケンス
sequence = [0.0, 1.00, 0.0, -1.00]

# publisher安定待ち
time.sleep(0.5)

for _ in range(test_times):
    for value in sequence:
        msg.position = [value, 0, 0, 0]
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo(f"Published: {value}")
        time.sleep(2.0)
    msg.position = [0, 0, 0, 0]
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)
    rospy.loginfo(f"Published: {value}")
    time.sleep(1.0)

