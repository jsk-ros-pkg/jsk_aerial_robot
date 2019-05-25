#!/usr/bin/env python

import sys
import time
import rospy
import math
from spinal.msg import ServoControlCmd

rospy.init_node("gimbal_control_test")

duration = rospy.get_param("~duration", 0.02) #20Hz
freq = rospy.get_param("~freq", 1) #hz
amp = rospy.get_param("~amp", 100) #rad

gimbal_control_topic_name = rospy.get_param("~gimbal_control_topic_name", "/servo/target_states")
pub = rospy.Publisher(gimbal_control_topic_name, ServoControlCmd, queue_size=10)

time.sleep(0.5)

gimbal = ServoControlCmd()
gimbal.index = [2,3]
gimbal.angles = [2047,2047]

pub.publish(gimbal)

time.sleep(0.5)

start_time = rospy.get_time()
while not rospy.is_shutdown():
    time_diff = start_time - rospy.get_time()
    gimbal.angles[0] = 2047 + amp * math.cos(2* math.pi * freq * time_diff)
    gimbal.angles[1] = 2047 + amp * math.sin(2* math.pi * 2 * freq * time_diff)

    pub.publish(gimbal)
    time.sleep(duration)
