#!/usr/bin/env python

from __future__ import print_function # for print function in python2

import time

# import sys, select, termios, tty

import numpy as np
import rospy
import math

from spinal.msg import ServoControlCmd
from spinal.msg import ServoStates
from spinal.msg import ServoTorqueCmd
from spinal.msg import ServoTorqueStates
from spinal.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop():
    def __init__(self):

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.vel_rate = rospy.get_param('~vel_rate', 250.0)
        self.max_val = rospy.get_param('~max_val', 250)
        self.max_turn_angle_val = rospy.get_param('~max_turn_angle', 1024)
        self.keep_vel = rospy.get_param('~keep_vel', [0.0, 0.0])
        self.servo_equ_angles = rospy.get_param('~servos_angle', [0, 0, 0])
        self.servo_0_load = rospy.get_param('~servo_0_load', 0)
        self.servo_torque_enable = rospy.get_param('~servo_enable', [0, 0, 0])
        self.servo_cmd = rospy.get_param('~servo_cmd', [0, 0, 0])
        self.turning = rospy.get_param('~turning', False)
        self.imu_angle = rospy.get_param('imu_angle', 0.0)
        self.tar_angle = rospy.get_param('~tar_angle', 0.0)
        self.tar_angle_ref = rospy.get_param('~tar_angle_ref', 0.0)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twistCallback)
        self.servo_pos_sub = rospy.Subscriber('/servo/states', ServoStates, self._servoStateCallback)
        self.servo_torque_sub = rospy.Subscriber('/servo/torque_states', ServoTorqueStates, self._servoTorqueStateCallback)

        self.cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)
        # self.torque_pub = rospy.Publisher('/servo/torque_enable', ServoTorqueCmd, queue_size=1)


    def _servoStateCallback(self, msg):
        self.servo_equ_angles = [msg.servos[0].angle % 4096, msg.servos[1].angle % 4096, msg.servos[2].angle]
        self.servo_0_load = msg.servos[0].load

    def _servoTorqueStateCallback(self, msg):
        self.servo_torque_enable = [msg.torque_enable[0], msg.torque_enable[1], msg.torque_enable[2]]

    def pos_control(self, servo_no, tar, rate):
        pos = self.servo_equ_angles[servo_no]
        if math.fabs(tar - pos) > 2048:
            if tar-pos < 0:
                tar += 4096
            else:
                tar -= 4096
        if tar-pos < 0:
            direction = -1
        else:
            direction = 1

        kp = 2.8
        if math.fabs(tar-pos) > 700:
            return direction * rate*self.max_val
        if math.fabs(tar-pos) > 50:
            # if (tar-pos)/2048 * kp >= 1:
            #     return direction*self.max_val
            return (tar-pos)/2048 * kp * rate*self.max_val
        else:
            return 0.0

    def servo_sync(self):
        self.servo_cmd[0] = self.pos_control(0, 2048, 1)
        self.servo_cmd[1] = self.pos_control(1, 2048, 1)
        # self.servo_cmd[2] = self.pos_control(2, 2048, 1)

    def left_pose(self):
        # self.servo_cmd[0] = self.pos_control(0, 1024, 1)
        self.servo_cmd[1] = self.pos_control(1, 3072, 1)

    def right_pose(self):
        # self.servo_cmd[0] = self.pos_control(0, 3072, 1)
        self.servo_cmd[1] = self.pos_control(1, 1024, 1)

    # def reboot_servo_2(self):
    #     msg = ServoTorqueCmd()
    #     msg.index = [2]
    #     msg.torque_enable = [0]
    #     self.torque_pub
    #     time.sleep(2)
    #     msg.torque_enable

    def _joyCallback(self, msg):

        # forward
        forward_vel = msg.axes[1]
        if math.fabs(forward_vel) < self.joy_dead_zone:
            forward_vel = 0

        forward_val = forward_vel * self.vel_rate
        if forward_val > self.max_val:
            forward_val = self.max_val
        if forward_val < -self.max_val:
            forward_val = -self.max_val

        self.servo_cmd[1] = forward_val
        self.servo_cmd[0] = -forward_val
        self.servo_cmd[2] = 0

        # only control one servo with L2 & R2 & square
        if msg.buttons[6] == 1: #L2
            vel_0 = -0.5 * (msg.axes[3] - 1.0)
            if math.fabs(vel_0) > self.joy_dead_zone:
                if msg.buttons[0] == 1:
                    self.servo_cmd[0] = -vel_0 * self.vel_rate
                else:
                    self.servo_cmd[1] = vel_0 * self.vel_rate
        if msg.buttons[7] == 1: #R2
            vel_0 = 0.5 * (msg.axes[4] - 1.0)
            if math.fabs(vel_0) > self.joy_dead_zone:
                if msg.buttons[0] == 1:
                    self.servo_cmd[0] = -vel_0 * self.vel_rate
                else:
                    self.servo_cmd[1] = vel_0 * self.vel_rate

        # pose fish to left or right
        if msg.axes[9] == 1.0:
            self.left_pose()
        if msg.axes[9] == -1.0:
            self.right_pose()

        # sync the two servos to middle position
        if msg.axes[10] != 0.0: # front or back
            self.servo_sync()

        # lock on and off
        if msg.buttons[2] == 1: #lock
            self.servo_cmd[2] = self.pos_control(2, 2048+600, 1)
        if msg.buttons[1] == 1: #unlock
            # time.sleep(2)
            self.servo_cmd[2] = self.pos_control(2, 2048, 1)
            self.servo_cmd[1] = self.pos_control(1, 0, 1)

        # spring charge
        if msg.buttons[5] == 1:
            if self.servo_0_load < 700:
                self.servo_cmd[0] = -self.max_val

        # motor_vel = np.array([int(forward_vel), int(forward_val)], dtype=np.int16)
        msg_0 = ServoControlCmd()
        msg_0.index = [0,1,2]
        msg_0.cmd.append(int(self.servo_cmd[0]))
        msg_0.cmd.append(int(self.servo_cmd[1]))
        msg_0.cmd.append(int(self.servo_cmd[2]))
        self.cmd_pub.publish(msg_0)


    def _twistCallback(self, msg):

        vel = msg.linear.x
        val = vel * self.vel_rate
        if vel > self.max_val:
            val = self.max_val
        if vel < -self.max_val:
            val = -self.max_val
        msg = ServoControlCmd()
        msg.index = [0]
        msg.cmd.append(int(val))

        self.cmd_pub.publish(msg)



if __name__=="__main__":

    rospy.init_node("teleop")

    teleop_node = Teleop()

    rospy.spin()
