#!/usr/bin/env python

import sys
import time
import rospy
import math
from spinal.msg import ServoControlCmd

rospy.init_node("ethernet_load_test")

joint_num = rospy.get_param("~joint_num", 14)
rate = rospy.get_param("~rate", 500) # 500Hz

joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "/target_servo_sub")
pub = rospy.Publisher(joint_control_topic_name, ServoControlCmd, queue_size=10)

joint_msg = ServoControlCmd()
joint_msg.index  = [1] * joint_num
joint_msg.angles = [1] * joint_num

loop_rate = rospy.Rate(rate)

cnt = 0

time.sleep(1.0)

while not rospy.is_shutdown():

    for i in range(4):
        joint_msg.index[i] = (cnt >> (8 * i)) & 0xff
    pub.publish(joint_msg)
    loop_rate.sleep()
    cnt +=1

