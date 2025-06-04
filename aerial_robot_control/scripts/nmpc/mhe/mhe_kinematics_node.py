#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped, AccelStamped
from spinal.msg import Imu
from mhe_kinematics import MHEKinematics

# TODO why run ros node in python? -> convert this file to cpp and move out of 'scripts/' folder. 

class MHEKinematicsNode:
    def __init__(self, robot_name):
        """
        Initializes the ROS node for kinematics-based Moving Horizon Estimation (MHE).

        :param robot_name: Name of the robot (e.g., "beetle1").
        """
        self.robot_name = robot_name

        rospy.init_node(f'{self.robot_name}_mhe_node', anonymous=True)

        # Create MHE object and its solver
        mhe = MHEKinematics()
        self.mhe_solver = mhe.get_ocp_solver()

        self.mhe_nm = 13    # Number of measurements
        self.mhe_nx = self.mhe_solver.acados_ocp.dims.nx
        self.mhe_nu = self.mhe_solver.acados_ocp.dims.nu
        self.mhe_np = self.mhe_solver.acados_ocp.dims.np
        self.mhe_N = self.mhe_solver.N

        # Initialize parameter list
        self.mhe_p_list = np.zeros((self.mhe_N, self.mhe_np))
        self.mhe_p_list[:, 0] = 1.0     # Initialize quaternion

        # Initialize state estimation at t=0 for MHE solver
        self.x0_bar = np.zeros(self.mhe_nx) 
        self.x0_bar[9] = 1.0            # Initialize quaternion
        for stage in range(self.mhe_N + 1):     # Prediction horizon is until the Nth time Step, therefore there are N+1 time Steps when including initial time Step t0
            self.mhe_solver.set(stage, "x", self.x0_bar)

        # Initialize reference
        self.mhe_yref_0 = np.zeros(self.mhe_nm + self.mhe_nu + self.mhe_nx)
        self.mhe_yref_0[6] = 1.0        # Initialize quaternion
        self.mhe_yref_list = np.zeros((self.mhe_N, self.mhe_nm + self.mhe_nu))
        self.mhe_yref_list[:, 6] = 1.0  # Initialize quaternion

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

    def imu_callback(self, imu_data: Imu):
        """
        Callback function for the IMU topic.

        :param imu_data: Incoming IMU data (sensor_msgs/Imu).
        """
        self.latest_imu_data = imu_data

        if self.latest_mocap_pose is None:
            rospy.logwarn("No mocap pose received from MHE.")
            return
        
        # Get state information
        pos_meas = self.latest_mocap_pose.pose.position
        acc_meas = self.latest_imu_data.acc_data
        quat_meas = self.latest_mocap_pose.pose.orientation
        omega_meas = self.latest_imu_data.gyro_data

        # Step 1: Shift and update parameter list
        self.mhe_p_list[:-1, :] = self.mhe_p_list[1:, :]    # Overwrite rows 0 to n-1 with rows from 1 to n
        self.mhe_p_list[-1, :] = np.array([quat_meas.w, quat_meas.x, quat_meas.y, quat_meas.z])     # Update quartenion parameters to nth row

        # Step 2: Shift and update yref_list
        self.mhe_yref_0[:self.mhe_nm + self.mhe_nu] = self.mhe_yref_list[0, :self.mhe_nm + self.mhe_nu]
        self.mhe_yref_0[self.mhe_nm + self.mhe_nu:] = self.x0_bar

        self.mhe_yref_list[:-1, :] = self.mhe_yref_list[1:, :]

        self.mhe_yref_list[-1, 0:3] = np.array([pos_meas.x, pos_meas.y, pos_meas.z])                    # position, from Mocap
        self.mhe_yref_list[-1, 3:6] = np.array([acc_meas[0], acc_meas[1], acc_meas[2]])                 # acc, from IMU
        self.mhe_yref_list[-1, 6:10] = np.array([quat_meas.w, quat_meas.x, quat_meas.y, quat_meas.z])   # quaternion angles, from Mocap
        self.mhe_yref_list[-1, 10:13] = np.array([omega_meas[0], omega_meas[1], omega_meas[2]])         # omega, from IMU

        # Step 3: Set parameters and yref
        self.mhe_solver.set(0, "p", self.mhe_p_list[0, :])
        self.mhe_solver.set(0, "yref", self.mhe_yref_0)

        for stage in range(1, self.mhe_N):
            self.mhe_solver.set(stage, "p", self.mhe_p_list[stage, :])
            self.mhe_solver.set(stage, "yref", self.mhe_yref_list[stage - 1, :])

        self.mhe_solver.set(self.mhe_N, "yref", self.mhe_yref_list[self.mhe_N - 1, :self.mhe_nm])

        # Step 4: Solve OCP
        self.mhe_solver.solve()

        # Step 5: Update initial state estimate x0_bar for next yref update
        # TODO move before yref_list update??!?!
        self.x0_bar = self.mhe_solver.get(0, "x")

        # Step 6: Get last estimated state and input
        mhe_state = self.mhe_solver.get(self.mhe_N, "x")
        mhe_u = self.mhe_solver.get(self.mhe_N - 1, "u")

        # Step 7.1: Publish last estimated pose
        est_pose = PoseStamped()
        est_pose.header.stamp = rospy.Time.now()
        est_pose.pose.position.x = mhe_state[0]
        est_pose.pose.position.y = mhe_state[1]
        est_pose.pose.position.z = mhe_state[2]
        est_pose.pose.orientation.w = mhe_state[9]
        est_pose.pose.orientation.x = mhe_state[10]
        est_pose.pose.orientation.y = mhe_state[11]
        est_pose.pose.orientation.z = mhe_state[12]
        self.est_pose_publisher.publish(est_pose)

        # Step 7.2: Publish last estimated twist
        est_twist = TwistStamped()
        est_twist.header.stamp = rospy.Time.now()
        est_twist.twist.linear.x = mhe_state[3]
        est_twist.twist.linear.y = mhe_state[4]
        est_twist.twist.linear.z = mhe_state[5]
        est_twist.twist.angular.x = mhe_state[13]
        est_twist.twist.angular.y = mhe_state[14]
        est_twist.twist.angular.z = mhe_state[15]
        self.est_twist_publisher.publish(est_twist)

        # Step 7.3: Publish last estimated acceleration

        est_acc = AccelStamped()
        est_acc.header.stamp = rospy.Time.now()
        est_acc.accel.linear.x = mhe_state[6]
        est_acc.accel.linear.y = mhe_state[7]
        est_acc.accel.linear.z = mhe_state[8]
        est_acc.accel.angular.x = mhe_u[3]
        est_acc.accel.angular.y = mhe_u[4]
        est_acc.accel.angular.z = mhe_u[5]
        self.est_acc_publisher.publish(est_acc)

    def mocap_callback(self, pose_data: PoseStamped):
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
