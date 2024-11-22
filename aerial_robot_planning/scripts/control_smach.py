#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@File    : control_smach.py
@Author  : Li_Jia_Xuan
@Date    : 2024-11-21 16:30
@Software: PyCharm
"""

import smach
import smach_ros
import sys
import os
import yaml
import rospkg
import argparse

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
import rosgraph
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import sys, select, termios, tty
import scipy.spatial.transform as trans
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3,PoseStamped,Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class HandPosition():
    def __init__(self):
        self.hand_position = PoseStamped()

        # Sub
        self.hand_pose_sub = rospy.Subscriber(
            "/hand/mocap/pose",
            PoseStamped,
            self.hand_position_callback,
            queue_size=1
        ) # 100HZ

    def hand_position_callback(self, msg: PoseStamped):
        self.hand_position = msg

class ArmPosition():
    def __init__(self):

        self.arm_position = PoseStamped()

         # Sub
        self.arm_pose_sub = rospy.Subscriber(
            "/arm/mocap/pose",
            PoseStamped,
            self.arm_position_callback,
            queue_size=1
        ) # 100HZ

    def arm_position_callback(self,msg: PoseStamped):
        self.arm_position = msg

class DronePosition():
    def __init__(self, robot_name: str) -> None:
        self.robot_name = robot_name

        self.drone_position = Odometry()
        self.drone_pose_sub = rospy.Subscriber(
            f"/{robot_name}/uav/cog/odom",
            Odometry,
            self.sub_odom_callback,
            queue_size = 1
        )

    def sub_odom_callback(self, msg: Odometry):
        self.drone_position = msg

# Define the Idle state / 定义“待起飞状态”
class Idle(smach.State):
    def __init__(self,hand_position, arm_position, start_pub, takeoff_pub):
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['takeoff'])
        self.hand_position = hand_position
        self.arm_position = arm_position
        self.start_pub = start_pub
        self.takeoff_pub = takeoff_pub
        self.time_last_up = None

        #todo
        # 当超过阈值，并达到了一定的时间长，就进行起飞
        self.direction_up_threshold = 0.25
        self.direction_hold_time = 5

    def execute(self, userdata):

        # Log the current state / 打印当前状态日志

        rospy.loginfo("Current state: Idle / 当前状态: 待起飞状态")
        z_difference = self.hand_position.pose.position.z - self.arm_position.pose.position.z
        if z_difference > self.direction_up_threshold:

            if self.time_last_up is None:
                self.time_last_up = rospy.get_time()
            elif rospy.get_time() - self.time_last_up >= self.direction_hold_time:
                self.start_pub.publish(Empty())
                time.sleep(0.1)
                self.takeoff_pub.publish(Empty())
                return 'takeoff'
        else:
            self.time_last_up = None

        return 'Idle'

# Define the Takeoff state / 定义“起飞状态,自锁状态”
class Takeoff(smach.State):
    def __init__(self, drone_position):
        smach.State.__init__(self, outcomes=['lock'])

        self.drone_position = drone_position

        self.last_position = None

        self.stability_threshold = 0.05  # 稳定阈值（单位：米）

        self.stability_time = 5  # 位置变化稳定的时间（单位：秒）

        self.last_stability_time = None

    def execute(self, userdata):
        rospy.loginfo("Current state: Takeoff / 当前状态: 起飞状态")

        current_position = self.drone_position.pose.position

        if self.last_position is None:
            self.last_position = current_position
            self.last_stability_time = rospy.get_time()
            return 'Takeoff'

        distance = ((current_position.x - self.last_position.x) ** 2 +
                    (current_position.y - self.last_position.y) ** 2 +
                    (current_position.z - self.last_position.z) ** 2) ** 0.5

        if distance < self.stability_threshold:
            if self.last_stability_time is None:
                self.last_stability_time = rospy.get_time()
                return 'Takeoff'
            elif rospy.get_time() - self.last_stability_time >= self.stability_time:
                rospy.loginfo("Drone position is stable.")
                return 'lock'
        else:
            self.last_position = current_position
            self.last_stability_time = rospy.get_time()
            return 'Takeoff'

class Lock(smach.State):
    def __init__(self,hand_position):

        smach.State.__init__(self, outcomes=['unlock'])
        self.hand_position = hand_position
        self.time_last_x = None

        # todo
        # 寻找一个参考的正方向，保证此时你想要的手的姿态和无人机上手的位置对其，才可以解锁无人机
        self.x_axis = [1.0, 0.0, 0.0]
        self.direction_hold_time = 5

    def execute(self, userdata):
        rospy.loginfo("Current state: Takeoff / 当前状态: 起飞状态")

        hand_orientation = [
            self.hand_position.pose.orientation.x,
            self.hand_position.pose.orientation.y,
            self.hand_position.pose.orientation.z,
            self.hand_position.pose.orientation.w
        ]
        # todo
        # 调整各种参数
        rotation_matrix = trans.Rotation.from_quat(hand_orientation).as_matrix()

        cos_theta = rotation_matrix[0, 0]

        angle_degrees = math.acos(cos_theta) * (180.0 / math.pi)

        if angle_degrees < 15:
            if self.time_last_x is None:
                self.time_last_up = rospy.get_time()
                return 'Lock'
            elif rospy.get_time() - self.time_last_up >= self.direction_hold_time:
                return 'unlock'
            else:
                return 'Lock'
        else:
            self.time_last_x = None
            return 'Lock'

# Define the Unlock state / 定义“解锁状态”
class Unlock(smach.State):
    def __init__(self):
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['direction_flight'])

    def execute(self, userdata):
        # Log the current state / 打印当前状态日志
        rospy.loginfo("Current state: Unlock / 当前状态: 解锁状态")

        # Simulate unlocking process / 模拟解锁过程
        rospy.sleep(1)

        # Transition to the next state / 转移到下一状态
        return 'direction_flight'

# Define the DirectionFlight state / 定义“方向飞行状态”
class DirectionFlight(smach.State):
    def __init__(self, robot_name: str ,pub_ref_traj,namespace,node_name,hand_position,arm_position,uav_odom) -> None:
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['1_to_1_motion', 'land'])
        self.robot_name = robot_name
        self.hand_position = hand_position
        self.arm_position = arm_position
        self.uav_odom = uav_odom
        self.namespace = namespace
        self.node_name = node_name

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

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory type: based on mocap.")

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Frequency
        self.ts_pt_pub = 0.02 # [s]  callback

        # Callback Timer
        self.timer = None

        # 0 - direction_flight, 1 - 1_to_1_motion , 2 - land
        self.mode = 0
    def _callback_ps_tb(self):

        """Callback function to compute robot's x-axis direction vector from PoseStamped."""

        # 1.calculate the direction vector of movement
        direction = [
            self.hand_position.pose.position.x - self.arm_position.pose.position.x,
            self.hand_position.pose.position.y - self.arm_position.pose.position.y,
            self.hand_position.pose.position.z - self.arm_position.pose.position.z
        ]

        distance = math.sqrt(sum([x ** 2 for x in direction]))
        if distance > 0.25:
            move_position = [direction[i] + getattr(self.uav_odom.pose.pose.position, axis) for i, axis in
                             enumerate(['x', 'y', 'z'])]
        else:
            move_position = [getattr(self.uav_odom.pose.pose.position, axis) for axis in ['x', 'y', 'z']]

        hand_orientation = [self.hand_position.pose.orientation.x,
                            self.hand_position.pose.orientation.y,
                            self.hand_position.pose.orientation.z,
                            self.hand_position.pose.orientation.w]

        # 2.Path Calculation
        multi_dof_joint_traj = MultiDOFJointTrajectory()
        t_has_started = rospy.Time.now().to_sec() - self.start_time

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

        self.timer = rospy.Timer(rospy.Duration(self.ts_pt_pub), self._callback_ps_tb)

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

        rospy.loginfo("Current state: Direction Flight / 当前状态: 方向飞行状态")

        if self.mode == 0:
            return 'direction_flight'
        elif self.mode == 1:
            return '1_to_1_motion'
        elif self.mode == 2:
            return 'land'

    def __del__(self):
        self.timer.shutdown()


# Define the Motion1to1 state / 定义“1:1运动状态”
class Motion1to1(smach.State):
    def __init__(self,pub_ref_traj,hand_position,arm_position,uav_odom) -> None:
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['direction_flight', 'land'])
        self.pub_ref_traj = pub_ref_traj
        self.hand_position = hand_position
        self.arm_position = arm_position
        self.uav_odom = uav_odom

        self.initial_position = None

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

    def _call_back(self):

        if self.initial_position is None:
            self.initial_position = [
                self.hand_position.pose.position.x,
                self.hand_position.pose.position.y,
                self.hand_position.pose.position.z
            ]

        current_position = [
            self.hand_position.pose.position.x,
            self.hand_position.pose.position.y,
            self.hand_position.pose.position.z
        ]

        self.position_change = [
            current_position[i] - self.initial_position[i] for i in range(3)
        ]

        direction = [
            self.uav_odom.pose.pose.position.x + self.position_change[0],
            self.uav_odom.pose.pose.position.y + self.position_change[1],
            self.uav_odom.pose.pose.position.z + self.position_change[2]
        ]

        hand_orientation = [self.hand_position.pose.orientation.x,
                            self.hand_position.pose.orientation.y,
                            self.hand_position.pose.orientation.z,
                            self.hand_position.pose.orientation.w]

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

        # Log the current state / 打印当前状态日志
        rospy.loginfo("Current state: 1:1 Motion / 当前状态: 1:1 运动状态")



        return 'direction_flight'


# Define the Land state / 定义“降落状态”
class Land(smach.State):
    def __init__(self,land_pub):
        # Initialize the state with possible outcomes / 初始化状态并设置可能的输出结果
        smach.State.__init__(self, outcomes=['end'])
        self.land_pub = land_pub

    def execute(self, userdata):
        # Log the current state / 打印当前状态日志
        rospy.loginfo("Current state: Land / 当前状态: 降落状态")

        land_pub.publish(Empty())

        # Transition to the final state / 转移到结束状态
        return 'end'

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description="MPC Point Trajectory Publisher Node")
    parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")
    args = parser.parse_args()

    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("MocapControl_node")

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

    # Pub
    pub_ref_traj = rospy.Publisher(f"/{args.robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=5)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Add the states to the state machine
    with sm:
        # Define the transitions between states
        smach.StateMachine.add('IDLE', Idle(), transitions={'takeoff': 'TAKEOFF'})
        smach.StateMachine.add('TAKEOFF', Takeoff(), transitions={'lock': 'LOCK'})
        smach.StateMachine.add('LOCK', Lock(), transitions={'unlock': 'UNLOCK'})
        smach.StateMachine.add('UNLOCK', Unlock(), transitions={'direction_flight': 'DIRECTION_FLIGHT'})
        smach.StateMachine.add('DIRECTION_FLIGHT', DirectionFlight(),
                               transitions={'1_to_1_motion': 'MOTION1TO1', 'land': 'LAND'})
        smach.StateMachine.add('MOTION1TO1', Motion1to1(),
                               transitions={'direction_flight': 'DIRECTION_FLIGHT', 'land': 'LAND'})
        smach.StateMachine.add('LAND', Land(), transitions={'end': 'end'})

    # Create a SMACH introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Stop the introspection server after state machine execution is done
    sis.stop()