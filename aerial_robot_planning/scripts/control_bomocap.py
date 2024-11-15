#!/usr/bin/env python
"""
 Created by li-jiaxuan on 24-10-27.
"""

import sys
import os
import argparse

current_path = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, current_path)

import yaml
import rospy
import rospkg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Twist, Quaternion, Vector3,PoseStamped,Pose
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

# read parameters from yaml
rospack = rospkg.RosPack()
param_path = os.path.join(rospack.get_path("beetle"), "config", "BeetleNMPCFull.yaml")
with open(param_path, "r") as f:
    param_dict = yaml.load(f, Loader=yaml.FullLoader)
nmpc_params = param_dict["controller"]["nmpc"]

class MocapControl_node():
    """
    Hand Based on Mocap
    """
    def __init__(self, robot_name: str) -> None:
        self.robot_name = robot_name
        self.node_name = "MocapControl_node"
        rospy.init_node(self.node_name, anonymous=False)
        self.namespace = rospy.get_namespace().rstrip("/")

        # Sub --> feedback
        self.uav_odom = Odometry()
        rospy.Subscriber(f"/{robot_name}/uav/cog/odom", Odometry, self.sub_odom_callback)

        # Initialize the hand and arm position information
        self.hand_position = PoseStamped()
        self.arm_position = PoseStamped()

        # initialize vel_twist and acc_twist
        r_rate, p_rate, y_rate = 0.0, 0.0, 0.0
        r_acc, p_acc, y_acc = 0.0, 0.0, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        ax, ay, az = 0.0, 0.0, 0.0
        self.vel_twist = Twist(linear=Vector3(vx, vy, vz), angular=Vector3(r_rate, p_rate, y_rate))
        self.acc_twist = Twist(linear=Vector3(ax, ay, az), angular=Vector3(r_acc, p_acc, y_acc))

        # Pub
        self.pub_ref_traj = rospy.Publisher(f"/{self.robot_name}/set_ref_traj", MultiDOFJointTrajectory, queue_size=5)

        # nmpc and robot related
        self.N_nmpc = int(nmpc_params["T_pred"] / nmpc_params["T_integ"])

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Trajectory type: based on mocap.")

        self.start_time = rospy.Time.now().to_sec()
        rospy.loginfo(f"{self.namespace}/{self.node_name}: Initialized!")

        # Frequency
        self.ts_pt_pub = 0.02 # [s]  callback

        # Sub
        self.hand_pose_sub = rospy.Subscriber(
            "/hand/mocap/pose",
            PoseStamped,
            self.hand_position_callback,
            queue_size=1
        ) # 100HZ

        # Sub
        self.arm_pose_sub = rospy.Subscriber(
            "/arm/mocap/pose",
            PoseStamped,
            self.arm_position_callback,
            queue_size=1
        ) # 100HZ

        self.timer = rospy.Timer(rospy.Duration(self.ts_pt_pub), self.callback_ps_tb)

        rospy.loginfo(f"{self.namespace}/{self.node_name}: Timer started!")

    def callback_ps_tb(self,event):
        """Callback function to compute robot's x-axis direction vector from PoseStamped."""

        # 1.calculate the direction vector of movement
        direction = [
            self.hand_position.pose.position.x - self.arm_position.pose.position.x,
            self.hand_position.pose.position.y - self.arm_position.pose.position.y,
            self.hand_position.pose.position.z - self.arm_position.pose.position.z
        ]

        distance = sum([x ** 2 for x in direction])/2
        if distance > 0.5:
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

        for i in range(self.N_nmpc+1):

            traj_pt = MultiDOFJointTrajectoryPoint()
            traj_pt.transforms.append(Transform(translation=Vector3(*move_position), rotation=Quaternion(*hand_orientation)))
            traj_pt.velocities.append(self.vel_twist)
            traj_pt.accelerations.append(self.acc_twist)

            t_pred = i * 0.1
            t_cal = t_pred + t_has_started
            traj_pt.time_from_start = rospy.Duration.from_sec(t_cal)

            multi_dof_joint_traj.points.append(traj_pt)

        multi_dof_joint_traj.header.stamp = rospy.Time.now()
        multi_dof_joint_traj.header.frame_id = "map"

        self.pub_ref_traj.publish(multi_dof_joint_traj)

    def hand_position_callback(self,msg: PoseStamped):
        self.hand_position = msg

    def arm_position_callback(self,msg: PoseStamped):
        self.arm_position = msg

    def sub_odom_callback(self, msg: Odometry):
        self.uav_odom = msg

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="MPC Point Trajectory Publisher Node")
    parser.add_argument("robot_name", type=str, help="Robot name, e.g., beetle1, gimbalrotors")
    args = parser.parse_args()
    try:
        node = MocapControl_node(args.robot_name)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
