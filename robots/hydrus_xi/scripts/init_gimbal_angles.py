#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math

class InitGimbalAngle:
    def __init__(self):
        rospy.init_node('init_gimbal_angles')
        self.pub = rospy.Publisher('/hydrus_xi/gimbals_ctrl', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self.callback)
        self.complete = False
        #self.angle = [2.6983, -1.3321, 2.7187, -1.3223, 2.7143, -1.3094]
        self.angle = [0.9491179081867179, -0.35778419036538456, 0.9463033377709107, -0.36272130661470514, 0.950201787642569, -0.3565103549241811]

    def callback(self, msg):
        pub_msg = JointState()
        pub_msg.position = self.angle
        self.complete = True
        if not rospy.is_shutdown():
            self.pub.publish(pub_msg)

if __name__=='__main__':
    node = InitGimbalAngle()

    while not rospy.is_shutdown():
        if node.complete:
            rospy.sleep(10)
            rospy.signal_shutdown('init gimbal angles complete')
