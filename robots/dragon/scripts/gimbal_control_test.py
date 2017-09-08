#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState

rospy.init_node("gimbal_control_test")

link_num = rospy.get_param("~link_num", 4)
duration = rospy.get_param("~duration", 0.05) #20Hz

gimbal_control_topic_name = rospy.get_param("~gimbal_control_topic_name", "/dragon/gimbals_ctrl")
pub = rospy.Publisher(gimbal_control_topic_name, JointState, queue_size=10)

gimbal = JointState()
gimbal.position = []

# gimbal
for i in range(1, link_num + 1):
    gimbal.position.append(0)
    gimbal.position.append(0)

gimbal.position[0] = 0.6;
pub.publish(gimbal)

time.sleep(0.5)

start_time = rospy.get_time()
while not rospy.is_shutdown():
    time_diff = start_time - rospy.get_time()
    gimbal.position[0] = 0.6 * math.cos(8 * time_diff / math.pi / 2 )
    gimbal.position[1] = 0.6 * math.sin(8 * time_diff / math.pi / 2 )

    pub.publish(gimbal)
    time.sleep(duration)
