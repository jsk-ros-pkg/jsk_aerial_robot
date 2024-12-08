#!/usr/bin/env python
'''
 Created by li-jinjie on 24-12-8.
'''

import numpy as np
from mhe_kinematics import MHEKinematics
import rospy
from spinal.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped


class MHEKinematicsNode:
    def __init__(self, robot_name):
        """
        Initializes the ROS node for kinematics-based Moving Horizon Estimation.

        :param robot_name: Name of the robot (e.g., "beetle1").
        """
        self.robot_name = robot_name

        rospy.init_node(f'{self.robot_name}_mhe_node', anonymous=True)

        # MHE
        mhe = MHEKinematics()
        self.mhe_solver = mhe.get_ocp_solver()

        self.mhe_nm = 13  # number of the measurements
        self.mhe_nx = self.mhe_solver.acados_ocp.dims.nx
        self.mhe_nu = self.mhe_solver.acados_ocp.dims.nu
        self.mhe_np = self.mhe_solver.acados_ocp.dims.np
        self.mhe_N = self.mhe_solver.N

        # initialize MHE solver
        x0_bar = np.zeros(self.mhe_nx)
        for stage in range(self.mhe_N + 1):
            self.mhe_solver.set(stage, "x", x0_bar)

        self.mhe_yref_0 = np.zeros(self.mhe_nm + self.mhe_nu + self.mhe_nx)
        self.mhe_yref_list = np.zeros((self.mhe_N, self.mhe_nm + self.mhe_nu))
        self.mhe_p_list = np.zeros((self.mhe_N, self.mhe_np))

        # Subscribers
        imu_topic = f"/{self.robot_name}/imu"
        self.imu_subscriber = rospy.Subscriber(imu_topic, Imu, self.imu_callback)
        self.latest_imu_data = None

        mocap_topic = f"/{self.robot_name}/mocap/pose"
        self.mocap_subscriber = rospy.Subscriber(mocap_topic, PoseStamped, self.mocap_callback)
        self.latest_mocap_pose = None

        rospy.loginfo(f"MHE Node initialized for {self.robot_name}. Subscribing to:")
        rospy.loginfo(f" - {imu_topic}")
        rospy.loginfo(f" - {mocap_topic}")

        # Publishers
        self.est_pose_publisher = rospy.Publisher(f"/{self.robot_name}/mhe/pose", PoseStamped, queue_size=10)
        self.est_twist_publisher = rospy.Publisher(f"/{self.robot_name}/mhe/twist", TwistStamped, queue_size=10)
        self.est_acc_publisher = rospy.Publisher(f"/{self.robot_name}/mhe/acc", AccelStamped, queue_size=10)

    def imu_callback(self, imu_data):
        """
        Callback function for the IMU topic.

        :param imu_data: Incoming IMU data (sensor_msgs/Imu).
        """
        # Placeholder for IMU data processing
        self.latest_imu_data = imu_data

    def mocap_callback(self, pose_data):
        """
        Callback function for the Mocap Pose topic.

        :param pose_data: Incoming Mocap Pose data (geometry_msgs/PoseStamped).
        """
        self.latest_mocap_pose = pose_data

    def run(self):
        """
        Keeps the ROS node running.
        """
        rospy.spin()


if __name__ == '__main__':
    try:
        # Fetch robot name from ROS parameter server or use default
        robot_name = rospy.get_param("~robot_name", "beetle1")
        node = MHEKinematicsNode(robot_name)
        node.run()
    except rospy.ROSInterruptException:
        pass
