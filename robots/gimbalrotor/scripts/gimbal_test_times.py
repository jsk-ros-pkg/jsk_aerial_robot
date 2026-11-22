#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

rospy.init_node("gimbal_test_sequence")

pub = rospy.Publisher("/gimbalrotor/gimbals_ctrl", JointState, queue_size=1)

# 正の整数（デフォルト1）
test_times = rospy.get_param("~test_times", 1)

if test_times <= 0:
    rospy.logerr("~test_times must be a positive integer")
    exit(1)

msg = JointState()
msg.name = ["gimbal1", "gimbal2", "gimbal3", "gimbal4"]

# 繰り返すシーケンス
sequence = [0.0, -0.35, 0.0, 0.35]

# publisher安定待ち
time.sleep(0.5)

for _ in range(test_times):
    for value in sequence:
        msg.position = [value, value, value, value]
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rospy.loginfo(f"Published: {value}")
        time.sleep(1.0)
