#!/usr/bin/env python
'''
 Created by li-jinjie on 24-12-15.
'''

import numpy as np
from mhe_wrench_est_acc_mom import MHEWrenchEstAccMom
import rospy
from spinal.msg import Imu, ESCTelemetry, ESCTelemetryArray, FourAxisCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped

from tilt_qd_servo_thrust_dist import XrUrConverter

import phys_param_beetle_omni as phys_omni


class MHEWrenchEstAccMomNode:
    def __init__(self, robot_name):
        """
        Initializes the ROS node for kinematics-based Moving Horizon Estimation.

        :param robot_name: Name of the robot (e.g., "beetle1").
        """
        self.robot_name = robot_name

        rospy.init_node(f'{self.robot_name}_mhe_node', anonymous=True)

        # MHE
        mhe = MHEWrenchEstAccMom()
        self.mhe_solver = mhe.get_ocp_solver()

        self.mhe_nm = 6  # number of the measurements
        self.mhe_nx = self.mhe_solver.acados_ocp.dims.nx
        self.mhe_nu = self.mhe_solver.acados_ocp.dims.nu
        self.mhe_np = self.mhe_solver.acados_ocp.dims.np
        self.mhe_N = self.mhe_solver.N

        # initialize MHE solver
        self.x0_bar = np.zeros(self.mhe_nx)
        for stage in range(self.mhe_N + 1):
            self.mhe_solver.set(stage, "x", self.x0_bar)

        self.mhe_yref_0 = np.zeros(self.mhe_nm + self.mhe_nu + self.mhe_nx)
        self.mhe_yref_list = np.zeros((self.mhe_N, self.mhe_nm + self.mhe_nu))
        self.mhe_p_list = np.zeros((self.mhe_N + 1, self.mhe_np))

        # Publishers
        self.est_dist_wrench_publisher = rospy.Publisher(f"/{self.robot_name}/mhe/dist_wrench", WrenchStamped,
                                                         queue_size=10)

        # Others
        self.alloc_mat = XrUrConverter()._get_alloc_mat()

        # Subscribers -- always initialize subscribers last
        imu_topic = f"/{self.robot_name}/imu"
        self.imu_subscriber = rospy.Subscriber(imu_topic, Imu, self.imu_callback)
        self.latest_imu_data = None

        joint_states_topic = f"/{self.robot_name}/joint_states"
        self.joint_states_subscriber = rospy.Subscriber(joint_states_topic, JointState, self.joint_states_callback)
        self.latest_joint_states = None

        esc_telem_topic = f"/{self.robot_name}/esc_telem"
        self.esc_telem_subscriber = rospy.Subscriber(esc_telem_topic, ESCTelemetryArray, self.esc_telem_callback)
        self.latest_esc_telem = None

        rospy.loginfo(f"MHE Node initialized for {self.robot_name}. Subscribing to:")
        rospy.loginfo(f" - {imu_topic}")
        rospy.loginfo(f" - {joint_states_topic}")
        rospy.loginfo(f" - {esc_telem_topic}")

        # Timer
        self.dist_est_timer = rospy.Timer(rospy.Duration(0.01), self.dist_est_timer_callback)

    def dist_est_timer_callback(self, event):

        if self.latest_imu_data is None or self.latest_esc_telem is None or self.latest_joint_states is None:
            return

        # Calculate wrench_u_sensor_b
        ft1 = (self.latest_esc_telem.esc_telemetry_1.rpm * 0.001) ** 2 / 7.7991
        ft2 = (self.latest_esc_telem.esc_telemetry_2.rpm * 0.001) ** 2 / 7.7991
        ft3 = (self.latest_esc_telem.esc_telemetry_3.rpm * 0.001) ** 2 / 7.7991
        ft4 = (self.latest_esc_telem.esc_telemetry_4.rpm * 0.001) ** 2 / 7.7991
        ft_sensor = np.array([ft1, ft2, ft3, ft4])

        a1 = self.latest_joint_states.position[0]
        a2 = self.latest_joint_states.position[1]
        a3 = self.latest_joint_states.position[2]
        a4 = self.latest_joint_states.position[3]
        a_sensor = np.array([a1, a2, a3, a4])

        z_sensor = np.zeros(8)
        z_sensor[0] = ft_sensor[0] * np.sin(a_sensor[0])
        z_sensor[1] = ft_sensor[0] * np.cos(a_sensor[0])
        z_sensor[2] = ft_sensor[1] * np.sin(a_sensor[1])
        z_sensor[3] = ft_sensor[1] * np.cos(a_sensor[1])
        z_sensor[4] = ft_sensor[2] * np.sin(a_sensor[2])
        z_sensor[5] = ft_sensor[2] * np.cos(a_sensor[2])
        z_sensor[6] = ft_sensor[3] * np.sin(a_sensor[3])
        z_sensor[7] = ft_sensor[3] * np.cos(a_sensor[3])

        wrench_u_sensor_b = np.dot(self.alloc_mat, z_sensor)

        # Calculate wrench_u_imu_b
        sf_b_imu = np.array(
            [self.latest_imu_data.acc_data[0], self.latest_imu_data.acc_data[1], self.latest_imu_data.acc_data[2]])
        w_imu = np.array(
            [self.latest_imu_data.gyro_data[0], self.latest_imu_data.gyro_data[1], self.latest_imu_data.gyro_data[2]])
        force_u_imu_b = np.zeros(3)
        force_u_imu_b[0:3] = phys_omni.mass * sf_b_imu

        # step 1: shift u_list
        self.mhe_p_list[:-1, :] = self.mhe_p_list[1:, :]
        self.mhe_p_list[-1, 0:3] = wrench_u_sensor_b[3:6]  # tau_u_g

        # step 2: shift yref_list
        self.mhe_yref_0[:self.mhe_nm + self.mhe_nu] = self.mhe_yref_list[0, :self.mhe_nm + self.mhe_nu]
        self.mhe_yref_0[self.mhe_nm + self.mhe_nu:] = self.x0_bar

        self.mhe_yref_list[:-1, :] = self.mhe_yref_list[1:, :]

        self.mhe_yref_list[-1, 0:3] = (
                force_u_imu_b[0:3] - wrench_u_sensor_b[0:3])  # f_d_w  TODO: convert from b to w frames
        self.mhe_yref_list[-1, 3:6] = w_imu  # omega_g, from sensor

        # step 3: fill yref and p
        self.mhe_solver.set(0, "yref", self.mhe_yref_0)
        self.mhe_solver.set(0, "p", self.mhe_p_list[0, :])

        for stage in range(1, self.mhe_N):
            self.mhe_solver.set(stage, "yref", self.mhe_yref_list[stage - 1, :])
            self.mhe_solver.set(stage, "p", self.mhe_p_list[stage, :])

        self.mhe_solver.set(self.mhe_N, "yref", self.mhe_yref_list[self.mhe_N - 1, :self.mhe_nm])
        self.mhe_solver.set(self.mhe_N, "p", self.mhe_p_list[self.mhe_N, :])

        # step 4: solve
        self.mhe_solver.solve()

        # step 5: update x0_bar
        self.x0_bar = self.mhe_solver.get(1, "x")

        # step 6: update estimated states
        mhe_x = self.mhe_solver.get(self.mhe_N, "x")

        disturb_estimated = WrenchStamped()
        disturb_estimated.wrench.force.x = mhe_x[3]
        disturb_estimated.wrench.force.y = mhe_x[4]
        disturb_estimated.wrench.force.z = mhe_x[5]
        disturb_estimated.wrench.torque.x = mhe_x[6]
        disturb_estimated.wrench.torque.y = mhe_x[7]
        disturb_estimated.wrench.torque.z = mhe_x[8]

        # step 7: publish estimated disturbance
        self.est_dist_wrench_publisher.publish(disturb_estimated)

    def imu_callback(self, imu_data: Imu):
        """
        Callback function for the IMU topic.

        :param imu_data: Incoming IMU data (sensor_msgs/Imu).
        """
        self.latest_imu_data = imu_data

    def joint_states_callback(self, joint_states: JointState):
        """
        Callback function for the joint states topic.

        :param joint_states: Incoming joint states data (sensor_msgs/JointState).
        """
        self.latest_joint_states = joint_states

    def esc_telem_callback(self, esc_telem: ESCTelemetryArray):
        """
        Callback function for the ESC telemetry topic.

        :param esc_telem: Incoming ESC telemetry data (spinal.msg.ESCTelemetryArray).
        """
        self.latest_esc_telem = esc_telem

    def run(self):
        """
        Keeps the ROS node running.
        """
        rospy.spin()


if __name__ == '__main__':
    try:
        # Fetch robot name from ROS parameter server or use default
        robot_name = rospy.get_param("~robot_name", "beetle1")
        node = MHEWrenchEstAccMomNode(robot_name)
        node.run()
    except rospy.ROSInterruptException:
        pass
