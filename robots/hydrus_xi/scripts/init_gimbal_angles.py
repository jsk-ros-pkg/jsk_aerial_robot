#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import math

class InitGimbalAngle:
    def __init__(self):
        rospy.init_node('init_gimbal_angles')

        self.init_angles = {}
        try:
            index = 1
            while True:
                angle = rospy.get_param('/hydrus_xi/servo_controller/gimbals/controller' + str(index) + '/simulation/init_value')
                name = rospy.get_param('/hydrus_xi/servo_controller/gimbals/controller' + str(index) + '/name')
                index += 1
                self.init_angles[name] = angle
        except:
            pass

        self.pub = rospy.Publisher('/hydrus_xi/gimbals_ctrl', JointState, queue_size=1)
        self.sub = rospy.Subscriber('/hydrus_xi/joint_states', JointState, self.callback)
        self.complete = False

    def callback(self, msg):
        pub_msg = JointState()
        pub_msg.name = self.init_angles.keys()
        pub_msg.position = self.init_angles.values()
        self.complete = True
        if not rospy.is_shutdown():
            self.pub.publish(pub_msg)

if __name__=='__main__':
    node = InitGimbalAngle()

    while not rospy.is_shutdown():
        if node.complete:
            rospy.sleep(10)
            rospy.signal_shutdown('init gimbal angles complete')
