#!/usr/bin/env python3

import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
from aerial_robot_msgs.msg import ControlInput
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from spinal.msg import FourAxisCommand, TorqueAllocationMatrixInv
from pid import PI_D
from mujoco_navigation import mujocoNavigator

class mujocoFlightController:
    def __init__(self):
        self.takeoff_flag = False
        self.flight_config = None
        self.navigator = mujocoNavigator()
        self.target_pos = np.zeros(3)
        self.target_rpy = np.zeros(3)

        x_pid_param = rospy.get_param("~x_pid_param")
        y_pid_param = rospy.get_param("~y_pid_param")
        z_pid_param = rospy.get_param("~z_pid_param")
        roll_pid_param = rospy.get_param("~roll_pid_param")
        pitch_pid_param = rospy.get_param("~pitch_pid_param")
        yaw_pid_param = rospy.get_param("~yaw_pid_param")
        self.x_pid = PI_D(*x_pid_param)
        self.y_pid = PI_D(*y_pid_param)
        self.z_pid = PI_D(*z_pid_param)
        self.roll_pid = PI_D(*roll_pid_param)
        self.pitch_pid = PI_D(*pitch_pid_param)
        self.yaw_pid = PI_D(*yaw_pid_param)

        # ros publisher
        self.control_input = ControlInput()
        self.ctrl_input_pub = rospy.Publisher("ctrl_input", ControlInput, queue_size=1)
        self.ctrl_input_pus = rospy.Publisher("hoge", ControlInput, queue_size=1)

        # ros subscriber
        self.odom = PoseStamped()
        self.twist = TwistStamped()
        self.mocap_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)
        self.twist_sub = rospy.Subscriber("twist", TwistStamped, self.twistCallback)

        # ros timer
        self.prev_time = rospy.get_time()
        timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)

    def setNavigator(self, navigator):
        self.navigator = navigator

    def takeoff(self):
        self.takeoff_flag = True

    def flightConfigCallback(self, msg):
        pass

    def mocapCallback(self, msg):
        self.odom = msg

    def twistCallback(self, msg):
        self.twist = msg

    def timerCallback(self, event):
        now_time = rospy.get_time()
        if not self.takeoff_flag:
            return
        else:
            current_rpy = tf.transformations.euler_from_quaternion([self.odom.pose.orientation.x, self.odom.pose.orientation.y, self.odom.pose.orientation.z, self.odom.pose.orientation.w])
            target_force_w = np.zeros(4)
            target_force_w[0] = self.x_pid(self.odom.pose.position.x, self.twist.twist.linear.x, dt=now_time-self.prev_time)
            target_force_w[1] = self.y_pid(self.odom.pose.position.y, self.twist.twist.linear.y, dt=now_time-self.prev_time)
            target_force_w[2] = self.z_pid(self.odom.pose.position.z, self.twist.twist.linear.z, dt=now_time-self.prev_time)
            rot_mat = tf.transformations.rotation_matrix(-current_rpy[2], [0, 0, 1])
            target_force_b = rot_mat.dot(target_force_w)
            self.roll_pid.setpoint = -target_force_b[1]
            self.pitch_pid.setpoint = target_force_b[0]

            control_outputs = np.zeros(4)
            control_outputs[0] = target_force_b[2]
            control_outputs[1] = self.roll_pid(current_rpy[0], self.odom.twist.twist.angular.x, dt=self.dt)
            control_outputs[2] = self.pitch_pid(current_rpy[1], self.odom.twist.twist.angular.y, dt=self.dt)
            control_outputs[3] = self.yaw_pid(current_rpy[2], self.odom.twist.twist.angular.z, dt=self.dt)

        self.prev_time = now_time


if __name__ == '__main__':
    rospy.init_node("mujoco_flight_controller" , anonymous=True)
    node = mujocoFlightController()
    rospy.spin()
