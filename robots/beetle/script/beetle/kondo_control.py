#!/usr/bin/env python

from spinal.msg import ServoControlCmd
from sensor_msgs.msg import JointState

import rospy
import numpy as np
import math
import time

class KondoControl():
    def __init__(self,robot_name,robot_id,servo_id,real_machine):

        self.KONDO_SERVO_POSITION_MIN = 3500
        self.KONDO_SERVO_POSITION_MAX = 11500
        self.KONDO_SERVO_ANGLE_MIN = -2.36
        self.KONDO_SERVO_ANGLE_MAX = 2.36
        self.real_machine = real_machine
        self.robot_name = robot_name
        self.robot_id = robot_id
        self.servo_id = servo_id
        self.servo_name = 'gimbal' + str(robot_id)
        
        self.kondo_pub = rospy.Publisher('/' + self.robot_name +'/kondo_servo_cmd', ServoControlCmd, queue_size=1)
        self.gimbal_pub = rospy.Publisher('/' + self.robot_name + '/gimbals_ctrl', JointState, queue_size=1)
        
        self.servo_cmd_msg = ServoControlCmd()
        self.servo_cmd_msg.index = [self.servo_id]

        self.joint_state_cmd_msg = JointState()
        self.joint_state_cmd_msg.name = [self.servo_name]
        
        time.sleep(0.5)

    def rad2KondoPosConv(self,angle):
        kondo_pos = int((self.KONDO_SERVO_POSITION_MAX-self.KONDO_SERVO_POSITION_MIN)*(-angle - self.KONDO_SERVO_ANGLE_MIN)/(self.KONDO_SERVO_ANGLE_MAX - self.KONDO_SERVO_ANGLE_MIN) + self.KONDO_SERVO_POSITION_MIN)
        return kondo_pos

    def sendTargetAngle(self,target_angle):
        self.servo_cmd_msg.angles = [target_angle]
        self.joint_state_cmd_msg.position = [target_angle]
        if self.real_machine:
            self.kondo_pub.publish(self.servo_cmd_msg)
        else:
            self.gimbal_pub.publish(self.joint_state_cmd_msg)


