#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState

if __name__ == "__main__":

    rospy.init_node("hydrus_demo")

    link_num = rospy.get_param("~link_num", 4)
    joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "joints_ctrl")
    pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=10)

    time.sleep(5)
    joint = JointState()
    joint.position = []
    for i in range(0, link_num - 1):
        joint.position.append(math.pi * (link_num -2) / link_num)
    while not rospy.is_shutdown():

        for i in range(0, link_num - 1):
            joint.position[i] = -joint.position[i]
            pub.publish(joint)
            print joint.position
            time.sleep(5)
