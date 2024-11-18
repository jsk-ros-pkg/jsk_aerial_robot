#!/usr/bin/env python

from spinal.msg import ServoControlCmd, ServoTorqueCmd
from sensor_msgs.msg import JointState

import rospy
import numpy as np
import math
import time

class DynamixelControl():
    def __init__(self,robot_name,robot_id,servo_id,real_machine):

        self.DYNAMIXEL_SERVO_POSITION_MIN = 0
        self.DYNAMIXEL_SERVO_POSITION_MAX = 4096
        self.DYNAMIXEL_SERVO_ANGLE_MIN = 0
        self.DYNAMIXEL_SERVO_ANGLE_MAX = 2*math.pi
        self.real_machine = real_machine
        self.robot_name = robot_name
        self.robot_id = robot_id
        self.servo_id = servo_id
        self.servo_name = 'gimbal' + str(robot_id)
        
        self.dx_pos_pub = rospy.Publisher('/' + self.robot_name +'/servo/target_states', ServoControlCmd, queue_size=1)
        self.dx_torque_pub = rospy.Publisher('/' + self.robot_name +'/servo/torque_enable', ServoTorqueCmd, queue_size=1)
        self.gimbal_pub = rospy.Publisher('/' + self.robot_name + '/gimbals_ctrl', JointState, queue_size=1)
        
        self.servo_pos_msg = ServoControlCmd()
        self.servo_pos_msg.index = [self.servo_id]

        self.servo_torque_msg = ServoTorqueCmd()
        self.servo_torque_msg.index = [self.servo_id]        

        self.joint_state_cmd_msg = JointState()
        self.joint_state_cmd_msg.name = [self.servo_name]
        
        time.sleep(0.5)

    def rad2DynamixelPosConv(self,angle):
        dynamixel_pos = int((self.DYNAMIXEL_SERVO_POSITION_MAX-self.DYNAMIXEL_SERVO_POSITION_MIN)*(-angle - self.DYNAMIXEL_SERVO_ANGLE_MIN)/(self.DYNAMIXEL_SERVO_ANGLE_MAX - self.DYNAMIXEL_SERVO_ANGLE_MIN) + self.DYNAMIXEL_SERVO_POSITION_MIN)
        return dynamixel_pos

    def sendTargetAngle(self,target_angle):
        self.servo_pos_msg.angles = [target_angle]
        self.joint_state_cmd_msg.position = [target_angle]
        if self.real_machine:
            self.dx_pos_pub.publish(self.servo_pos_msg)
        else:
            self.gimbal_pub.publish(self.joint_state_cmd_msg)

    def torqueEnable(self,torque_enable):
        self.servo_torque_msg.torque_enable = [torque_enable]
        self.dx_torque_pub.publish(self.servo_torque_msg)


            
