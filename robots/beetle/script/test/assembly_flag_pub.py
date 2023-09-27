#!/usr/bin/env python

import rospy
import time
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool

if __name__=="__main__":
    rospy.init_node("assembly_flag_pub")
    r = rospy.Rate(1)
    flag_pub = rospy.Publisher('/beetle1/assembly_flag', KeyValue, queue_size = 1)
    docking_pub = rospy.Publisher('/beetle1/docking_cmd', Bool, queue_size = 1)

    flag_msg = KeyValue()
    docking_msg = Bool()
    time.sleep(0.5)

    flag_msg.key = '1'
    flag_msg.value = '1'
    flag_pub.publish(flag_msg)
    time.sleep(0.5)

    flag_msg.key = '2'
    flag_msg.value = '1'
    flag_pub.publish(flag_msg)
    time.sleep(0.5)

    docking_msg.data = True
    docking_pub.publish(docking_msg)

