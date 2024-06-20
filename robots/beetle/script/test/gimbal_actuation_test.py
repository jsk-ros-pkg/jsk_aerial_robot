#!/usr/bin/env python

from spinal.msg import ServoControlCmd
from sensor_msgs.msg import JointState

import rospy
import numpy as np
import math
import time

class GimbalTest():
    def __init__(self):

        self.KONDO_SERVO_POSITION_MIN = 3500
        self.KONDO_SERVO_POSITION_MAX = 11500
        self.KONDO_SERVO_ANGLE_MIN = -2.36
        self.KONDO_SERVO_ANGLE_MAX = 2.36

        self.test_angle_max = 1.0

        self.real_machine = True
        
        self.kondo_pub = rospy.Publisher('/beetle1/kondo_servo_cmd', ServoControlCmd, queue_size=1)
        self.gimbal_pub = rospy.Publisher('/beetle1/gimbals_ctrl', JointState, queue_size=1)
        self.nav_rate = rospy.Rate(40)
        
        self.servo_cmd_msg = ServoControlCmd()
        self.servo_cmd_msg.index = [1,2,3,4]

        self.joint_state_cmd_msg = JointState()
        self.joint_state_cmd_msg.name = ['gimbal1', 'gimbal2', 'gimbal3', 'gimbal4']

        self.t = 0
        
        time.sleep(0.5)

    def rad2KondoPosConv(self,angle):
        kondo_pos = int((self.KONDO_SERVO_POSITION_MAX-self.KONDO_SERVO_POSITION_MIN)*(-angle - self.KONDO_SERVO_ANGLE_MIN)/(self.KONDO_SERVO_ANGLE_MAX - self.KONDO_SERVO_ANGLE_MIN) + self.KONDO_SERVO_POSITION_MIN)
        return kondo_pos

    def main(self):
        while not rospy.is_shutdown():
            self.t = self.t + 0.04
            target_angle = self.test_angle_max * math.sin(self.t)
            target_kondo_pos = self.rad2KondoPosConv(target_angle)
            self.servo_cmd_msg.angles = [target_kondo_pos,target_kondo_pos,target_kondo_pos,target_kondo_pos]
            self.joint_state_cmd_msg.position = [target_angle,target_angle,target_angle,target_angle]

            if self.real_machine:
                self.kondo_pub.publish(self.servo_cmd_msg)
            else:
                self.gimbal_pub.publish(self.joint_state_cmd_msg)
            self.nav_rate.sleep()

if __name__ == "__main__":

  rospy.init_node("gimbale_test")

  gimbal_test = GimbalTest()
  gimbal_test.main()



