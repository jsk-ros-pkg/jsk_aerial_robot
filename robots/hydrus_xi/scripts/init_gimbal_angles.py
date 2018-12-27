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
        self.angle = [math.pi + -0.443246, 1.80942 - math.pi, -0.422855 + math.pi, 1.81924 - math.pi, -0.427232 + math.pi, 1.83216 - math.pi]

    def callback(self, msg):
        pub_msg = JointState()
        pub_msg.position = self.angle
        self.complete = True
        self.pub.publish(pub_msg)

if __name__=='__main__':
    node = InitGimbalAngle()

    while not rospy.is_shutdown():
        if node.complete:
            rospy.sleep(1)
            rospy.signal_shutdown('init gimbal angles complete')
