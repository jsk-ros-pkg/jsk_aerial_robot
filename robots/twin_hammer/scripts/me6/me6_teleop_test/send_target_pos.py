#!/usr/bin/env python

import os
import rospy
import xacro
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler

# FK用
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

class PoseTeleopME6:
    def __init__(self, urdf_path):
        rospy.init_node("me6_teleop_from_pose")

        # パラメータ
        self.delta_angle_thre = 0.1
        self.pub_rate = 5
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # URDFロード
        try:
            robot_description_config = xacro.process_file(urdf_path)
            rospy.set_param("/robot_description", robot_description_config.toxml())
        except Exception as e:
            rospy.logerr(f"URDF load failed: {e}")
            rospy.signal_shutdown("URDF load failed")
            return

        # IK
        self.base_link = "base_link"
        self.ee_link = "Link6"
        self.ik_solver = IK(self.base_link, self.ee_link)

        # FK用
        ok, kdl_tree = treeFromParam("/robot_description")
        if not ok:
            rospy.logerr("Failed to parse /robot_description into KDL tree.")
            rospy.signal_shutdown("KDL parse failed")
            return
        self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)

        # 状態管理
        self.last_joint_positions = np.zeros(len(self.joint_names))
        self.current_joint_positions = np.zeros(len(self.joint_names))
        self.joint_ready = False

        self.target_pos = np.zeros(3)
        self.target_rpy = np.zeros(3)
        self.target_received = False

        # Publisher
        self.gazebo_pub = rospy.Publisher("/me6_robot/joint_controller/command",
                                         JointTrajectory, queue_size=1)
        self.real_pub = rospy.Publisher("/me6_robot/joint_controller/follow_joint_trajectory/goal",
                                        FollowJointTrajectoryActionGoal, queue_size=1)

        # Subscriber
        rospy.Subscriber("/me6_robot/joint_states", JointState, self.jointstate_cb)
        rospy.Subscriber("/target_position", Point, self.position_cb)
        rospy.Subscriber("/target_rpy", Point, self.rpy_cb)

    def jointstate_cb(self, msg: JointState):
        joint_positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_positions.append(msg.position[idx])
            else:
                joint_positions.append(0.0)
        self.current_joint_positions = np.array(joint_positions)
        self.joint_ready = True

    def position_cb(self, msg: Point):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.target_received = True

    def rpy_cb(self, msg: Point):
        self.target_rpy = np.array([msg.x, msg.y, msg.z])
        self.target_received = True

    def limit_delta(self, next_joint):
        """
        next_joint: np.array
        self.last_joint_positions: np.array
        Δ制限をかけて返す
        """
        delta = next_joint - self.last_joint_positions
        delta = np.clip(delta, -self.delta_angle_thre, self.delta_angle_thre)
        return self.last_joint_positions + delta

    def publish_joint_command(self, joint_positions):
        # Gazebo
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start = rospy.Duration(1.0/self.pub_rate)
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(point)
        self.gazebo_pub.publish(traj)

        # 実機
        goal_msg = FollowJointTrajectoryActionGoal()
        goal_msg.goal.trajectory.joint_names = self.joint_names
        goal_msg.goal.trajectory.points.append(point)
        self.real_pub.publish(goal_msg)

        self.last_joint_positions = joint_positions

    def main(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            if self.joint_ready and self.target_received:
                # RPY → Quaternion
                q = quaternion_from_euler(self.target_rpy[0],
                                          self.target_rpy[1],
                                          self.target_rpy[2])
                sol = self.ik_solver.get_ik(
                    self.current_joint_positions,
                    self.target_pos[0], self.target_pos[1], self.target_pos[2],
                    q[0], q[1], q[2], q[3]
                )
                if sol is not None:
                    sol = np.array(sol)
                    # Δ制限をかける
                    sol_limited = self.limit_delta(sol)
                    self.publish_joint_command(sol_limited)
            rate.sleep()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")
    teleop = PoseTeleopME6(urdf_path)
    teleop.main()
