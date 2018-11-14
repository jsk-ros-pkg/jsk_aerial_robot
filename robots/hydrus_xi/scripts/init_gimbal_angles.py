#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math

class InitGimbalAngle:
    def __init__(self):
        rospy.init_node('init_gimbal_angles')
        self.pub = rospy.Publisher('/hydrus_xi/gimbals_ctrl', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self.callback)
        self.gimbal_num = rospy.get_param('/hydrus_xi/gimbal_num')
        self.complete = False


    def callback(self, msg):
        pub_msg = JointState()
        angles = []
        for i in range(self.gimbal_num):
            if i % 2 == 0:
                angles.append(0.0)
            else:
                angles.append(math.pi)
        pub_msg.position = angles
        self.complete = True
        self.pub.publish(pub_msg)

if __name__=='__main__':
    node = InitGimbalAngle()

    while not rospy.is_shutdown():
        if node.complete:
            rospy.sleep(1)
            rospy.signal_shutdown('init gimbal angles complete')
