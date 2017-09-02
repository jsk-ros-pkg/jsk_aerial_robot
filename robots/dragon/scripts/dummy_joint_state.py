#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState

rospy.init_node("dragon_dummy_joint_state")

link_num = rospy.get_param("~link_num", 4)
duration = rospy.get_param("~duration", 0.05) #20Hz

joint_control_topic_name = rospy.get_param("~joint_state_topic_name", "/dragon/joint_states")
pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=10)

joint = JointState()
joint.name = []
joint.position = []

# gimbal
for i in range(1, link_num + 1):
    joint.name.append("gimbal" + str(i)  + "_pitch")
    joint.name.append("gimbal" + str(i)  + "_roll")
    joint.position.append(0)
    joint.position.append(0)

# joint
for i in range(1, link_num):
    joint.name.append("joint" + str(i)  + "_pitch")
    joint.name.append("joint" + str(i)  + "_yaw")
    joint.position.append(0)
    if i == 1:
        joint.position.append(1.57)
    if i == 2:
        joint.position.append(0)
    if i == 3:
        joint.position.append(0)

while not rospy.is_shutdown():
    joint.header.stamp = rospy.get_rostime()
    pub.publish(joint)
    time.sleep(duration)
