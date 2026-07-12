#!/usr/bin/env python

from __future__ import print_function # for print function in python2

import numpy as np
# import sys, select, termios, tty

import rospy
import math

from spinal.msg import ServoControlCmd, ServoStates, ServoExtendedStates, ServoExtendedCmd, ServoExtendedCmds
from spinal.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop():
    def __init__(self):

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.max_val = rospy.get_param('~max_val', 250)
        self.servo_equ_angles = rospy.get_param('~servos_angle', 0)
        self.servo_cmd = rospy.get_param('~servo_cmd', [0, 0]) # 0 for servo, 1 for motor
        self.motor_equ_angles = rospy.get_param('~motor_angle', 0.0)
        self.motor_angle_raw = rospy.get_param('~motor_angle_raw', 0.0)
        self.motor_angle_ref = rospy.get_param('~motor_angle_ref', 2048+720)
        self.imu_angle = rospy.get_param('imu_angle', 0.0)
        self.imu_angle_raw = rospy.get_param('imu_angle_raw', 0.0)
        self.tar_angle_ref = rospy.get_param('~tar_angle_ref', 0.0)
        self.servo_axis = 0.0
        self.motor_acc = 0.0
        self.motor_acc_rate = rospy.get_param('~motor_acc_rate', 1.0)
        self.auto_control_mode = False
        self.auto_pattern = 'stop'
        self.auto_button_prev = 0
        self.turn_button_prev = 0
        self.fast_turn_tar_angle = 2048.0
        self.auto_start_time = rospy.Time.now()

        self.servo_cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)
        self.motor_cmd_pub = rospy.Publisher('/servo/extended_cmds', ServoExtendedCmds, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twistCallback)
        self.servo_pos_sub = rospy.Subscriber('/servo/states', ServoStates, self._servoStateCallback)
        self.motor_state_sub = rospy.Subscriber('/servo/extended_states', ServoExtendedStates, self._motorStateCallback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self._imuCallback)

        self.control_timer = rospy.Timer(rospy.Duration(0.02), self._controlLoop)


    def _servoStateCallback(self, msg):
        self.servo_equ_angles = msg.servos[0].angle % 4096

    def _imuCallback(self, msg):
        x = msg.quaternion[0]
        y = msg.quaternion[1]
        z = msg.quaternion[2]
        w = msg.quaternion[3]
        self.imu_angle_raw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        self.imu_angle = self.normalize_angle(self.imu_angle_raw - self.tar_angle_ref)

    def _motorStateCallback(self, msg):
        self.motor_angle_raw = msg.servos[0].angle % 4096
        self.motor_equ_angles = self.normalize_servo_angle(self.motor_angle_raw - self.motor_angle_ref + 2048.0)

    def initialize(self):
        self.tar_angle_ref = self.imu_angle_raw
        self.motor_angle_ref = self.motor_angle_raw
        self.motor_equ_angles = 2048.0
        rospy.loginfo("initialize fish: imu ref %s, motor ref %s", self.tar_angle_ref, self.motor_angle_ref)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def normalize_servo_angle(self, angle):
        while angle >= 4096:
            angle -= 4096
        while angle < 0:
            angle += 4096
        return angle

    def pos_control(self, servo_no, tar, rate):
        if servo_no == 1:
            pos = self.motor_equ_angles
            val = 2 * np.pi
            kp = 2.8
            deadzone = 50
        else:
            pos = self.servo_equ_angles
            val = self.max_val
            kp = 2.8
            deadzone = 50
        if math.fabs(tar - pos) > 2048:
            if tar-pos < 0:
                tar += 4096
            else:
                tar -= 4096
        if tar-pos < 0:
            direction = -1
        else:
            direction = 1


        if math.fabs(tar-pos) > 700:
            cmd = direction * rate * val
        elif math.fabs(tar-pos) > deadzone:
            cmd = (tar-pos)/2048 * kp * val
        else:
            cmd = 0.0
        self.servo_cmd[servo_no] = cmd

    def start_auto(self):
        self.auto_control_mode = True
        self.auto_pattern = 'fast_start'
        self.auto_start_time = rospy.Time.now()
        rospy.loginfo("start auto mode")

    def start_fast_turn(self, tar_angle):
        self.auto_control_mode = True
        self.auto_pattern = 'fast_turn'
        self.fast_turn_tar_angle = tar_angle
        self.auto_start_time = rospy.Time.now()
        rospy.loginfo("start fast turn")

    def start_cruise(self):
        self.auto_control_mode = False
        if not self.auto_pattern == 'cruise':
            rospy.loginfo("start cruise")
        self.auto_pattern = 'cruise'
        self.servo_cmd[1] = 4.0 * math.pi

    def stop_auto(self):
        self.auto_control_mode = False
        if not self.auto_pattern == 'stop':
            rospy.loginfo("motor stop.")
        self.auto_pattern = 'stop'
        self.stop_control()

    def manual_control(self):
        servo_tar = 2048.0 - self.servo_axis * 1024.0
        if servo_tar > 3072.0:
            servo_tar = 3072.0
        if servo_tar < 1024.0:
            servo_tar = 1024.0

        self.pos_control(0, servo_tar, 1.0)
        self.servo_cmd[1] += self.motor_acc * self.motor_acc_rate
        if self.servo_cmd[1] > self.max_val:
            self.servo_cmd[1] = self.max_val
        if self.servo_cmd[1] < -self.max_val:
            self.servo_cmd[1] = -self.max_val

    def stop_control(self):
        self.pos_control(0, 2048, 1.0)
        self.pos_control(1, 2048,1.0)

    def fast_start(self):
        t = (rospy.Time.now() - self.auto_start_time).to_sec()

        if t < 1.0:
            self.servo_cmd[0] = 250
            self.servo_cmd[1] = 2.0 * math.pi
        else:
            self.start_cruise()

    def fast_turn(self):
        t = (rospy.Time.now() - self.auto_start_time).to_sec()

        if t < 0.75:
            self.pos_control(0, 2048.0, 0.33)
            self.pos_control(1, 2048.0, 0.33)
        elif t < 1.0:
            self.pos_control(0, self.fast_turn_tar_angle, 1.0)
            self.pos_control(1, self.fast_turn_tar_angle, 3.0)
        else:
            self.start_cruise()

    def cruise(self):
        self.manual_control()

    def auto_control(self):
        if self.auto_pattern == 'fast_start':
            self.fast_start()
        elif self.auto_pattern == 'fast_turn':
            self.fast_turn()
        else:
            self.auto_control_mode = False
            self.servo_cmd[0] = 0
            self.servo_cmd[1] = 0

    def publish_cmd(self):
        servo_msg = ServoControlCmd()
        servo_msg.index = [0]
        servo_msg.cmd.append(int(self.servo_cmd[0]))
        self.servo_cmd_pub.publish(servo_msg)

        motor = ServoExtendedCmd()
        motor.index = 0
        motor.mode = 1
        motor.cmd = -self.servo_cmd[1]

        motor_msg = ServoExtendedCmds()
        motor_msg.stamp.secs = 0
        motor_msg.stamp.nsecs = 0
        motor_msg.servos.append(motor)
        self.motor_cmd_pub.publish(motor_msg)

    def _controlLoop(self, event):
        if self.auto_control_mode:
            self.auto_control()
            self.publish_cmd()

    def _joyCallback(self, msg):

        if msg.buttons[1] ==1:
            self.stop_auto()
            self.publish_cmd()
            return

        if self.auto_control_mode:
            return

        if msg.buttons[0] == 1:
            self.start_cruise()

        if msg.buttons[2] == 1:
            self.initialize()

        # servo
        self.servo_axis = msg.axes[2]
        if math.fabs(self.servo_axis) < self.joy_dead_zone:
            self.servo_axis = 0

        # auto trajectory
        if msg.buttons[3] == 1 and self.auto_button_prev == 0:
            self.start_auto()
            self.auto_button_prev = msg.buttons[3]
            return
        self.auto_button_prev = msg.buttons[3]

        if msg.buttons[5] == 1:
            turn_tar = 2048.0 - self.servo_axis * 1024.0
            if turn_tar > 3072.0:
                turn_tar = 3072.0
            if turn_tar < 1024.0:
                turn_tar = 1024.0

            if msg.buttons[4] == 1 and self.turn_button_prev == 0:
                if self.servo_axis > 0:
                    self.start_fast_turn(1024.0)
                    self.turn_button_prev = msg.buttons[4]
                    return
                elif self.servo_axis < 0:
                    self.start_fast_turn(3072.0)
                    self.turn_button_prev = msg.buttons[4]
                    return
            self.turn_button_prev = msg.buttons[4]
            self.pos_control(0, turn_tar, 1.0)
            self.pos_control(1, turn_tar, 1.0)
            self.publish_cmd()
            return
        if msg.buttons[4] == 0:
            self.turn_button_prev = 0

        # motor acceleration and deceleration
        if msg.buttons[6] == 1 or msg.buttons[7] == 1:
            self.motor_acc = 0.5 * (msg.axes[3] - 1.0) - 0.5 * (msg.axes[4] - 1.0)
            if math.fabs(self.motor_acc) < self.joy_dead_zone:
                self.motor_acc = 0
            rospy.loginfo("accelerate, acc = %s", self.motor_acc)
        else:
            self.motor_acc = 0

        if self.auto_pattern == 'stop':
            self.stop_control()
            self.publish_cmd()
        elif self.auto_pattern == 'cruise':
            self.cruise()
            self.publish_cmd()


    def _twistCallback(self, msg):

        if self.auto_pattern != 'stop':
            self.servo_cmd[1] = msg.linear.x
            self.publish_cmd()



if __name__=="__main__":

    rospy.init_node("teleop")

    teleop_node = Teleop()

    rospy.spin()
