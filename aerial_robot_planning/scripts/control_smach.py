#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : control_smach.py
@Author  : Li_Jia_Xuan
@Date    : 2024-11-21 16:30
@Software: PyCharm
"""

import os
import sys
import yaml
import smach
import rospkg
import signal
import argparse
import smach_ros

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = param_dict["controller"]["nmpc"]


import time
import math
import rospy
import socket
import rosgraph
import sys,termios
import numpy as np
from std_msgs.msg import UInt8
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import tf.transformations as tft
import scipy.spatial.transform as trans
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3,PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class HandPosition:
    def __init__(self, topic_name="/hand/mocap/pose"):
        self.hand_position = PoseStamped()
        self.topic_name = topic_name
        self.subscriber = rospy.Subscriber(
            self.topic_name, PoseStamped, self.hand_position_callback
        )
        rospy.loginfo(f"Subscribed to {self.topic_name}")

    def hand_position_callback(self, msg: PoseStamped):
        self.hand_position = msg

    def get_position(self):
        return self.hand_position

class ArmPosition:
    def __init__(self):
        self.arm_position = PoseStamped()
        self.arm_pose_sub = rospy.Subscriber(
            "/arm/mocap/pose", PoseStamped, self.arm_position_callback
        )
        rospy.loginfo("Subscribed to /arm/mocap/pose")

    def arm_position_callback(self, msg: PoseStamped):
        self.arm_position = msg

    def get_position(self):
        return self.arm_position

class DronePosition:
    def __init__(self, robot_name: str) -> None:
        self.robot_name = robot_name
        self.drone_position = Odometry()
        self.drone_pose_sub = rospy.Subscriber(
            f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback, queue_size=1
        )
        rospy.loginfo(f"Subscribed to {robot_name}/uav/cog/odom")

    def sub_odom_callback(self, msg: Odometry):
        self.drone_position = msg

    def get_position(self):
        return self.drone_position

class ControlMode:
    def __init__(self):
        self.control_mode = UInt8()

        self.control_mode_sub = rospy.Subscriber('hand/control_mode', UInt8, self.callback_control_mode)

    def callback_control_mode(self, data):
        self.control_mode = data.data

# Define the Idle state / 定义“待起飞状态”
class Idle(smach.State):
    def __init__(self,hand, arm, start_pub, takeoff_pub):
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['Takeoff','Idle'])
        self.hand = hand

        self.arm = arm

        self.start_pub = start_pub

        self.takeoff_pub = takeoff_pub

        self.time_last_up = None

        self.first = 1

        # 当超过阈值，并达到了一定的时间长，就进行起飞
        self.direction_up_threshold = 0.3
        self.direction_hold_time = 1

    def execute(self, userdata):
        if self.first == 1:
            print("Current state: Idle / 当前状态: 待起飞状态")
            print("正在等待起飞指令")
            self.first = 2
        # rospy.loginfo("Current state: Idle / 当前状态: 待起飞状态")
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            z_difference = self.hand.hand_position.pose.position.z - self.arm.arm_position.pose.position.z

            sys.stdout.write(f"\r当前手-肩距离差 = {z_difference:.4f}，阈值为 = {self.direction_up_threshold:.2f}")

            sys.stdout.flush()

            if z_difference > self.direction_up_threshold:
                if self.time_last_up is None:
                    self.time_last_up = rospy.get_time()
                elif rospy.get_time() - self.time_last_up >= self.direction_hold_time:
                    self.start_pub.publish(Empty())
                    time.sleep(0.1)
                    self.takeoff_pub.publish(Empty())
                    return 'Takeoff'
            else:
                self.time_last_up = None
                return 'Idle'

# Define the Takeoff state / 定义“起飞状态”
class Takeoff(smach.State):
    def __init__(self, drone):
        smach.State.__init__(self, outcomes=['Lock','Takeoff'])

        self.drone = drone

        self.last_position = None

        self.stability_threshold = 0.02  # 稳定阈值（单位：米）

        self.stability_time = 1  # 位置变化稳定的时间（单位：秒）

        self.last_stability_time = None

        self.first = 1
    def execute(self, userdata):

        if self.first == 1:
            print("")
            print("Current state: Takeoff / 当前状态: 起飞状态")
            print("正在进行稳定判断")
            self.first = 2

        current_position = self.drone.drone_position.pose.pose.position
        if self.last_position is None:
            self.last_position = current_position
            return 'Takeoff'

        distance = ((current_position.x - self.last_position.x) ** 2 +
                    (current_position.y - self.last_position.y) ** 2 +
                    (current_position.z - self.last_position.z) ** 2) ** 0.5

        sys.stdout.write(f"\r无人机当前位置波动数值为 = {distance:.4f}，阈值为 = {self.stability_threshold:.4f}")
        sys.stdout.flush()

        if distance < self.stability_threshold:
            if self.last_stability_time is None:
                self.last_stability_time = rospy.get_time()
                return 'Takeoff'
            elif rospy.get_time() - self.last_stability_time > self.stability_time:
                rospy.loginfo("Drone position is stable.")
                print("Drone position is stable.")
                return 'Lock'
            else:
                return 'Takeoff'
        else:
            self.last_position = current_position
            self.last_stability_time = None
            return 'Takeoff'

class Lock(smach.State):
    def __init__(self,hand,uav_odom):
        smach.State.__init__(self, outcomes=['Unlock','Lock'])

        self.hand = hand

        self.uav_odom = uav_odom

        self.time_last_x = None

        self.threshold = 8

        self.direction_hold_time = 1

        self.first = 1

    def execute(self, userdata):
        if self.first == 1:
            print("Current state: Lock / 当前状态: 锁定状态")
            print("正在进行解锁判断，请将手掌姿态对准无人机手姿态")
            self.first = 2

        drone_orientation = [
            self.uav_odom.drone_position.pose.pose.orientation.x,
            self.uav_odom.drone_position.pose.pose.orientation.y,
            self.uav_odom.drone_position.pose.pose.orientation.z,
            self.uav_odom.drone_position.pose.pose.orientation.w
        ]
        hand_orientation = [
            self.hand.hand_position.pose.orientation.x,
            self.hand.hand_position.pose.orientation.y,
            self.hand.hand_position.pose.orientation.z,
            self.hand.hand_position.pose.orientation.w
        ]

        q_drone_inv = tft.quaternion_inverse(drone_orientation)
        q_relative = tft.quaternion_multiply(hand_orientation, q_drone_inv)
        euler_angles = np.degrees(tft.euler_from_quaternion(q_relative))

        sys.stdout.write(
            f"\r无人机 {drone_orientation[0]:6.1f}, {drone_orientation[1]:6.1f}, {drone_orientation[2]:6.1f}, {drone_orientation[3]:6.1f} "
            f"手部 {hand_orientation[0]:6.1f}, {hand_orientation[1]:6.1f}, {hand_orientation[2]:6.1f}, {hand_orientation[3]:6.1f} "
            f"当前的各轴偏差值为 = Roll: {euler_angles[0]:6.1f}, Pitch: {euler_angles[1]:6.1f}, Yaw: {euler_angles[2]:6.1f}"
        )
        sys.stdout.flush()

        if abs(euler_angles[0]) < self.threshold and abs(euler_angles[1]) < self.threshold and abs(euler_angles[2]) < self.threshold:
            if self.time_last_x is None:
                self.time_last_x = rospy.get_time()
                return 'Lock'
            elif rospy.get_time() - self.time_last_x > self.direction_hold_time:
                print("")
                print("Current state: Unlock / 当前状态: 解锁状态")
                time.sleep(0.3)
                return 'Unlock'
            else:
                return 'Lock'
        else:
            self.time_last_x = None
            return 'Lock'

# Define the Unlock state / 定义“解锁状态”
class Unlock(smach.State):
    def __init__(self):
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['CylDirectionFlight'])

    def execute(self, userdata):
        # Log the current state / 打印当前状态日志
        print("Current state: CylDirectionFlight / 当前状态: 极坐标系")
        time.sleep(0.5)
        # Transition to the next state / 转移到下一状态
        return 'CylDirectionFlight'

# Define the DirectionFlight state / 定义“方向飞行状态”
class CylDirectionFlight(smach.State):
    def __init__(self, robot_name: str ,pub_ref_traj,namespace,node_name,hand,arm,uav_odom,control_mode) -> None:

        smach.State.__init__(self, outcomes=['Motion1to1', 'Land','CylDirectionFlight'])

        self.robot_name = robot_name
        self.hand = hand
        self.arm = arm
        self.uav_odom = uav_odom
        self.namespace = namespace
        self.node_name = node_name
        self.control_mode = control_mode

        # initialize vel_twist and acc_twist
        r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
        r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0
        self.vel_twist = Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate))
        self.acc_twist = Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc))

        self.pub_ref_traj = pub_ref_traj

        # nmpc and robot related
        self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])


        self.start_time = rospy.Time.now().to_sec()

        # Frequency
        self.ts_pt_pub = 0.1  # [s]  callback

        self.expected_ad_distance_mini = 0.5

        self.expected_ad_distance = 0.6

        self.timer = None

    def _callback_ps_tb(self, event):  # 添加 event 参数
        """Callback function to compute robot's x-axis direction vector from PoseStamped."""
        if self.hand.hand_position.pose.position.x == 0.0:
            print("出现空白数据")
        else:
            # 1.calculate the direction vector of movement
            ah_direction = [
                self.hand.hand_position.pose.position.x - self.arm.arm_position.pose.position.x,
                self.hand.hand_position.pose.position.y - self.arm.arm_position.pose.position.y,
                self.hand.hand_position.pose.position.z - self.arm.arm_position.pose.position.z
            ]

            ah_distance = math.sqrt(ah_direction[0] ** 2 + ah_direction[1] ** 2 + ah_direction[2] ** 2)

            ah_unit_vector = [
                ah_direction[0] / ah_distance,
                ah_direction[1] / ah_distance,
                ah_direction[2] / ah_distance
            ]

            ad_direction = [
                self.uav_odom.drone_position.pose.pose.position.x - self.arm.arm_position.pose.position.x,
                self.uav_odom.drone_position.pose.pose.position.y - self.arm.arm_position.pose.position.y,
                self.uav_odom.drone_position.pose.pose.position.z - self.arm.arm_position.pose.position.z
            ]

            ad_distance = math.sqrt(ad_direction[0] ** 2 + ad_direction[1] ** 2 + ad_direction[2] ** 2)

            ad_unit_vector = [
                ad_direction[0] / ad_distance,
                ad_direction[1] / ad_distance,
                ad_direction[2] / ad_distance
            ]

            hand_orientation = [self.hand.hand_position.pose.orientation.x,
                                self.hand.hand_position.pose.orientation.y,
                                self.hand.hand_position.pose.orientation.z,
                                self.hand.hand_position.pose.orientation.w]

            if ah_distance > 0.4:
                self.expected_ad_distance = self.expected_ad_distance + (ah_distance - 0.4)*0.5

                move_position = [
                    ah_unit_vector[i] * self.expected_ad_distance + getattr(self.arm.arm_position.pose.position, axis) for
                    i, axis in
                    enumerate(['x', 'y', 'z'])
                ]
                sys.stdout.write(
                    f"\r增加长度"
                )
                sys.stdout.flush()
                self._pub_ref_traj(move_position, hand_orientation)

                # move_position = [self.arm.arm_position.pose.position.x,self.arm.arm_position.pose.position.y,self.arm.arm_position.pose.position.z]
                # self._pub_ref_traj(move_position, hand_orientation)

            elif 0.4 > ah_distance > 0.2:
                sys.stdout.write(
                    f"\r原地不动"
                )
                sys.stdout.flush()
            else:
                self.expected_ad_distance = self.expected_ad_distance - (0.2 - ah_distance) * 3
                if self.expected_ad_distance < self.expected_ad_distance_mini:
                    self.expected_ad_distance = self.expected_ad_distance_mini
                if ad_distance < (self.expected_ad_distance_mini + 0.1):
                    sys.stdout.write(
                        f"\r无人机已抵达极限位置,不再移动"
                    )
                    sys.stdout.flush()
                else:
                    move_position = [
                        ad_unit_vector[i] * self.expected_ad_distance + getattr(self.arm.arm_position.pose.position, axis)
                        for i, axis in
                        enumerate(['x', 'y', 'z'])
                    ]
                    print(f"减少长度")
                    self._pub_ref_traj(move_position, hand_orientation)

    def _pub_ref_traj(self, move_position, hand_orientation):
        # 2.Path Calculation
        multi_dof_joint_traj = MultiDOFJointTrajectory()
        t_has_started = rospy.Time.now().to_sec() - self.start_time
        if move_position[2] < 0.5:
            move_position[2] = 0.5
        print(f"目标位置{move_position}")
        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(translation=Vector3(*move_position), rotation=Quaternion(*hand_orientation)))
            traj_pt.velocities.append(self.vel_twist)
            traj_pt.accelerations.append(self.acc_twist)

            t_pred = i * 0.1
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        multi_dof_joint_traj.header.stamp = rospy.Time.now()
        multi_dof_joint_traj.header.frame_id = "map"

        self.pub_ref_traj.publish(multi_dof_joint_traj)

    def execute(self, userdata):
        if self.control_mode.control_mode == 0:
            # Start the timer only when switching to 'direction_flight' state
            if self.timer is None:

                self.timer = rospy.Timer(rospy.Duration(self.ts_pt_pub), self._callback_ps_tb)

            return 'CylDirectionFlight'

        elif self.control_mode.control_mode == 1:
            if self.timer is not None:
                self.timer.shutdown()
                self.timer = None

            print("Current state: Motion1to1 / 当前状态: 一比一映射")
            return 'Motion1to1'

        elif self.control_mode.control_mode == 2:
            if self.timer is not None:
                self.timer.shutdown()
                self.timer = None

            print("Current state: Land / 当前状态: 降落")
            return 'Land'

# Define the Motion1to1 state / 定义“1:1运动状态”
class Motion1to1(smach.State):
    def __init__(self,hand,arm,uav_odom,control_mode,pub_ref_traj) -> None:
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['CylDirectionFlight', 'Land','Motion1to1'])
        self.hand = hand
        self.arm = arm
        self.uav_odom = uav_odom
        self.pub_ref_traj = pub_ref_traj
        self.control_mode = control_mode

        self.initial_hand_position = None
        self.initial_drone_position = None

        self.position_change = None

        # nmpc and robot related
        self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

        self.start_time = rospy.Time.now().to_sec()

        # Frequency
        self.ts_pt_pub = 0.02  # [s]  callback

        # initialize vel_twist and acc_twist
        r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
        r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0
        self.vel_twist = Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate))
        self.acc_twist = Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc))

        self.timer = None

    def _call_back_ps_tb(self,event):

        if self.initial_hand_position is None:
            self.initial_hand_position = [
                self.hand.hand_position.pose.position.x,
                self.hand.hand_position.pose.position.y,
                self.hand.hand_position.pose.position.z
            ]
            self.initial_drone_position = [
                self.uav_odom.drone_position.pose.pose.position.x,
                self.uav_odom.drone_position.pose.pose.position.y,
                self.uav_odom.drone_position.pose.pose.position.z
            ]
        else :
            current_position = [
                self.hand.hand_position.pose.position.x,
                self.hand.hand_position.pose.position.y,
                self.hand.hand_position.pose.position.z
            ]

            self.position_change = [
                current_position[i] - self.initial_hand_position[i] for i in range(3)
            ]

            direction = [

                self.initial_drone_position[0] + self.position_change[0],
                self.initial_drone_position[1] + self.position_change[1],
                self.initial_drone_position[2] + self.position_change[2]
            ]

            hand_orientation = [self.hand.hand_position.pose.orientation.x,
                                self.hand.hand_position.pose.orientation.y,
                                self.hand.hand_position.pose.orientation.z,
                                self.hand.hand_position.pose.orientation.w]

            sys.stdout.write(
                f"\r目标地点 {direction[0]:6.1f}, {direction[1]:6.1f}, {direction[2]:6.1f} "
                f"目标姿势 {hand_orientation[0]:6.1f}, {hand_orientation[1]:6.1f}, {hand_orientation[2]:6.1f}, {hand_orientation[3]:6.1f} "

            )
            sys.stdout.flush()

            # 2.Path Calculation
            multi_dof_joint_traj = MultiDOFJointTrajectory()
            t_has_started = rospy.Time.now().to_sec() - self.start_time

            for i in range(self.N_nmpc + 1):
                traj_pt = MultiDOFJointTrajectoryPoint()
                traj_pt.transforms.append(
                    Transform(translation=Vector3(*direction), rotation=Quaternion(*hand_orientation)))
                traj_pt.velocities.append(self.vel_twist)
                traj_pt.accelerations.append(self.acc_twist)

                t_pred = i * 0.1
                t_cal = t_pred + t_has_started
                traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

                multi_dof_joint_traj.points.append(traj_pt)

            multi_dof_joint_traj.header.stamp = rospy.Time.now()
            multi_dof_joint_traj.header.frame_id = "map"

            self.pub_ref_traj.publish(multi_dof_joint_traj)

    def execute(self, userdata):
        if self.control_mode.control_mode == 1:
            if self.timer is None:
                self.timer = rospy.Timer(rospy.Duration(self.ts_pt_pub), self._call_back_ps_tb)
            return 'Motion1to1'

        elif self.control_mode.control_mode == 0:
            if self.timer is not None:
                self.initial_hand_position = None
                self.timer.shutdown()
                self.timer = None

            print("Current state: CylDirectionFlight / 当前状态: 极坐标系")
            return 'CylDirectionFlight'

        elif self.control_mode.control_mode == 2:
            if self.timer is not None:
                self.timer.shutdown()
                self.timer = None

            print("Current state: Land / 当前状态: 降落")
            return 'Land'

# Define the Land state / 定义“降落状态”
class Land(smach.State):
    def __init__(self, uav_odom, land_pub, pub_ref_traj):
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['End', 'Land'])
        self.uav_odom = uav_odom
        self.land_pub = land_pub
        self.pub_ref_traj = pub_ref_traj

        # nmpc and robot related
        self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

        self.start_time = rospy.Time.now().to_sec()

        # Frequency
        self.ts_pt_pub = 0.02  # [s]  callback

        # initialize vel_twist and acc_twist
        r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
        r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0
        self.vel_twist = Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate))
        self.acc_twist = Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc))

        self.timer = None

        self.condition_start_time = None

    def execute(self, userdata):

        target_orientation = [0, 0, 0, 1]

        # Path Calculation
        multi_dof_joint_traj = MultiDOFJointTrajectory()

        t_has_started = rospy.Time.now().to_sec() - self.start_time

        move_position = [self.uav_odom.drone_position.pose.pose.position.x,
                         self.uav_odom.drone_position.pose.pose.position.y,
                         self.uav_odom.drone_position.pose.pose.position.z]

        for i in range(self.N_nmpc + 1):
            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(
                Transform(translation=Vector3(*move_position), rotation=Quaternion(*target_orientation)))
            traj_pt.velocities.append(self.vel_twist)
            traj_pt.accelerations.append(self.acc_twist)
            t_pred = i * 0.1
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)
            multi_dof_joint_traj.points.append(traj_pt)
        multi_dof_joint_traj.header.stamp = rospy.Time.now()
        multi_dof_joint_traj.header.frame_id = "map"
        self.pub_ref_traj.publish(multi_dof_joint_traj)

        current_dragon_orientation = self.uav_odom.drone_position.pose.pose.orientation

        current_orientation_list = [current_dragon_orientation.x,
                                     current_dragon_orientation.y,
                                     current_dragon_orientation.z,
                                     current_dragon_orientation.w]

        difference_squared = math.sqrt(sum((c - t) ** 2 for c, t in zip(current_orientation_list, target_orientation)))

        if difference_squared < 0.25:

            if self.condition_start_time is None:

                self.condition_start_time = rospy.Time.now().to_sec()

            elif rospy.Time.now().to_sec() - self.condition_start_time > 3.0:

                self.land_pub.publish(Empty())

                print("Current state: End / 当前状态: 结束")

                time.sleep(2)

                return 'End'

        else:

            self.condition_start_time = None

        return 'Land'

def safety_shutdown(land_pub):
    rospy.loginfo("Shutting down safely...")
    time.sleep(2)
    land_pub.publish(Empty())  # 发布停止/降落命令
    rospy.sleep(1)  # 等待一段时间让命令生效

def handle_shutdown_signal(signum, frame):
    rospy.loginfo("Caught Ctrl+C signal, performing shutdown...")
    time.sleep(2)
    safety_shutdown(land_pub)  # 执行安全关机操作
    rospy.signal_shutdown("Shutting down")

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="MPC Point Trajectory Publisher Node")
    parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")
    args = parser.parse_args()


    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("MocapControl_node", anonymous=False, log_level=rospy.WARN)

    robot_ns = rospy.get_param("~robot_ns", "");


    if not robot_ns:
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()

        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        if len(teleop_topics) == 1:
            robot_ns = teleop_topics[0].split('/teleop')[0]

    ns = robot_ns + "/teleop_command"

    start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
    halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
    land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)

    namespace = rospy.get_namespace().rstrip("/")

    # Pub
    pub_ref_traj = rospy.Publisher(f"/{args.robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=5)

    # init the hand,arm,drone
    hand = HandPosition()
    arm = ArmPosition()
    drone = DronePosition(robot_name=args.robot_name)

    control_mode = ControlMode()

    # 创建状态机
    sm = smach.StateMachine(outcomes=['End'])

    with sm:
        smach.StateMachine.add('Idle', Idle(hand, arm, start_pub, takeoff_pub),
                               transitions={'Takeoff': 'Takeoff', 'Idle': 'Idle'})
        smach.StateMachine.add('Takeoff', Takeoff(drone),
                               transitions={'Lock': 'Lock', 'Takeoff': 'Takeoff'})
        smach.StateMachine.add('Lock', Lock(hand,drone),
                               transitions={'Unlock': 'Unlock', 'Lock': 'Lock'})
        smach.StateMachine.add('Unlock', Unlock(),
                               transitions={'CylDirectionFlight': 'CylDirectionFlight'})
        smach.StateMachine.add('CylDirectionFlight', CylDirectionFlight(
            robot_name = args.robot_name,
            pub_ref_traj = pub_ref_traj,
            namespace = namespace ,
            node_name = "MocapControl_node",
            hand = hand,
            arm = arm,
            uav_odom=drone,
            control_mode=control_mode
            ),
            transitions={
                'Motion1to1': 'Motion1to1',
                'Land': 'Land',
                'CylDirectionFlight': 'CylDirectionFlight'
            },
                               )

        smach.StateMachine.add('Motion1to1', Motion1to1(
            pub_ref_traj = pub_ref_traj,
            arm = arm,
            uav_odom=drone,
            hand = hand,
            control_mode = control_mode
            ) ,
            transitions={
                'Motion1to1': 'Motion1to1',
                'Land': 'Land',
                'CylDirectionFlight': 'CylDirectionFlight'
            },
                               )

        smach.StateMachine.add('Land', Land(
            land_pub = land_pub,
            uav_odom = drone,
            pub_ref_traj = pub_ref_traj,
            ) ,
            transitions={
                'End': 'End',
                'Land': 'Land'
            },
                               )
    signal.signal(signal.SIGINT, handle_shutdown_signal)

    try:
        outcome = sm.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROSInterruptException caught.")
        safety_shutdown(land_pub)
    except Exception as e:
        rospy.logerr(f"State machine execution failed due to: {e}")
        safety_shutdown(land_pub)