#!/usr/bin/env python3

import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
from aerial_robot_msgs.msg import ControlInput, WrenchAllocationMatrix, PoseControlPid
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
from spinal.msg import FourAxisCommand, TorqueAllocationMatrixInv, FlightConfigCmd
from pid import PI_D
import tf

class mujocoFlightController:
    def __init__(self):
        self.takeoff_flag = False
        self.flight_config = None
        self.target_pos = np.zeros(3)
        self.target_rpy = np.zeros(3)
        self.rotor_names = ['rotor1_thrust', 'rotor2_thrust', 'rotor3_thrust', 'rotor4_thrust']

        # controllers
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

        self.torque_allocation_mat = np.zeros((4, 4))
        self.torque_allocation_mat_inv = np.zeros((4, 4))

        # ros publisher
        self.control_input = ControlInput()
        self.ctrl_input_pub = rospy.Publisher("ctrl_input", ControlInput, queue_size=1)
        self.pid_pub = rospy.Publisher("pid", PoseControlPid, queue_size=1)

        # ros subscriber
        self.pose = PoseStamped()
        self.twist = TwistStamped()
        self.wrench_allocation_mat =  WrenchAllocationMatrix()
        self.torque_allocation_matrix_inv = TorqueAllocationMatrixInv()
        self.four_axis_command = FourAxisCommand()

        self.mocap_sub = rospy.Subscriber("mocap/pose", PoseStamped, self.mocapCallback)
        self.twist_sub = rospy.Subscriber("twist", TwistStamped, self.twistCallback)
        self.wrench_allocation_mat_sub = rospy.Subscriber("wrench_allocation_mat", WrenchAllocationMatrix, self.wrenchAllocationMatrixCallback)
        self.torque_allocation_matrix_inv_sub = rospy.Subscriber("torque_allocation_matrix_inv", TorqueAllocationMatrixInv, self.torqueAllocationMatrixInvCallback)
        self.four_axis_commnad_sub = rospy.Subscriber("four_axes/command", FourAxisCommand, self.fourAxisCommandCallback)

        self.takeoff_sub = rospy.Subscriber("teleop_command/takeoff", Empty, self.takeoffCallback)
        self.halt_sub = rospy.Subscriber("teleop_command/halt", Empty, self.haltCallback)

        # ros param
        self.takeoff_height = rospy.get_param("~takeoff_height")

        # ros timer
        self.prev_time = rospy.get_time()
        timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)

    def takeoffCallback(self, msg):
        self.takeoff_flag = True
        rpy = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])
        self.target_rpy[2] = rpy[2]
        self.x_pid.setpoint = self.pose.pose.position.x
        self.y_pid.setpoint = self.pose.pose.position.y
        self.z_pid.setpoint = self.takeoff_height
        self.roll_pid.setpoint = 0.0
        self.pitch_pid.setpoint = 0.0
        self.yaw_pid.setpoint = rpy[2]
        print("xyz=[", self.pose.pose.position.x, self.pose.pose.position.y, self.takeoff_height, "], yaw=", rpy[2])

    def haltCallback(self, msg):
        self.takeoff_flag = False
        self.control_input.name = self.rotor_names
        self.control_input.input = [0] * len(self.rotor_names)
        self.ctrl_input_pub.publish(self.control_input)
        self.x_pid.reset()
        self.y_pid.reset()
        self.z_pid.reset()
        self.roll_pid.reset()
        self.pitch_pid.reset()
        self.yaw_pid.reset()

    def mocapCallback(self, msg):
        self.pose = msg

    def twistCallback(self, msg):
        self.twist = msg

    def wrenchAllocationMatrixCallback(self, msg):
        self.wrench_allocation_mat = msg
        self.torque_allocation_mat[0] = msg.f_z
        self.torque_allocation_mat[1] = msg.t_x
        self.torque_allocation_mat[2] = msg.t_y
        self.torque_allocation_mat[3] = msg.t_z
        self.torque_allocation_mat_inv = np.linalg.pinv(self.torque_allocation_mat)

    def torqueAllocationMatrixInvCallback(self, msg):
        self.torque_allocation_matrix_inv = msg

    def fourAxisCommandCallback(self, msg):
        self.four_axis_command = msg

    def timerCallback(self, event):
        now_time = rospy.get_time()
        self.dt = now_time - self.prev_time
        if not self.takeoff_flag:
            self.prev_time = now_time
            return
        else:
            current_rpy = tf.transformations.euler_from_quaternion([self.pose.pose.orientation.x, self.pose.pose.orientation.y, self.pose.pose.orientation.z, self.pose.pose.orientation.w])
            target_force_w = np.zeros(4)
            target_force_w[0] = self.x_pid(self.pose.pose.position.x, self.twist.twist.linear.x, dt=self.dt)
            target_force_w[1] = self.y_pid(self.pose.pose.position.y, self.twist.twist.linear.y, dt=self.dt)
            target_force_w[2] = self.z_pid(self.pose.pose.position.z, self.twist.twist.linear.z, dt=self.dt)

            cog_rot_mat_from_world = tf.transformations.rotation_matrix(-current_rpy[2], [0, 0, 1])  # (4, 4)
            target_force_b = np.dot(cog_rot_mat_from_world, target_force_w)

            self.roll_pid.setpoint = -target_force_b[1]
            self.pitch_pid.setpoint = target_force_b[0]

            yaw_err = self.yaw_pid.setpoint - current_rpy[2]
            if yaw_err > np.pi:
                self.yaw_pid.setpoint = self.yaw_pid.setpoint - 2 * np.pi
            elif yaw_err < -np.pi:
                self.yaw_pid.setpoint = self.yaw_pid.setpoint + 2 * np.pi

            control_outputs = np.zeros(4)
            control_outputs[0] = target_force_b[2]
            control_outputs[1] = self.roll_pid(current_rpy[0], self.twist.twist.angular.x, dt=self.dt)
            control_outputs[2] = self.pitch_pid(current_rpy[1], self.twist.twist.angular.y, dt=self.dt)
            control_outputs[3] = self.yaw_pid(current_rpy[2], self.twist.twist.angular.z, dt=self.dt)

            rotor_input = np.dot(self.torque_allocation_mat_inv, control_outputs)

            self.control_input.name = self.rotor_names
            self.control_input.input = rotor_input
            self.ctrl_input_pub.publish(self.control_input)

            pid = PoseControlPid()
            pid.x = self.x_pid.getDebugMsg()
            pid.y = self.y_pid.getDebugMsg()
            pid.z = self.z_pid.getDebugMsg()
            pid.roll = self.roll_pid.getDebugMsg()
            pid.pitch = self.pitch_pid.getDebugMsg()
            pid.yaw = self.yaw_pid.getDebugMsg()
            self.pid_pub.publish(pid)

            self.prev_time = now_time

if __name__ == '__main__':
    rospy.init_node("mujoco_flight_controller" , anonymous=True)
    node = mujocoFlightController()
    rospy.spin()
