#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import time

if __name__=="__main__":

    rospy.init_node('joint_state_pub')
    joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    time.sleep(5)

    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state_pub.publish(joint_state)
