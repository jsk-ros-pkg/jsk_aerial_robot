#!/usr/bin/env python

import os
import rospy
import xacro
import numpy as np
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from trac_ik_python.trac_ik import IK
from control_msgs.msg import FollowJointTrajectoryActionGoal

# 追加: KDL で FK
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

script_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")

class MocapTeleopME6Test:
    def __init__(self, urdf_path):
        rospy.init_node("me6_teleop_from_mocap")

        # URDF を /robot_description に反映
        try:
            robot_description_config = xacro.process_file(urdf_path)
            robot_description_xml = robot_description_config.toxml()
            rospy.set_param("/robot_description", robot_description_xml)
            rospy.loginfo(f"URDF loaded to /robot_description from {urdf_path}")
        except Exception as e:
            rospy.logerr(f"Failed to load URDF: {e}")
            rospy.signal_shutdown("URDF load failed")
            return

        # ベースとEEリンク名（URDF/MoveItに合わせて）
        self.base_link = "base_link"
        self.ee_link   = "Link6"
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

        # IK ソルバ
        self.ik_solver = IK(self.base_link, self.ee_link)

        # KDL 構築（FK 用）
        ok, kdl_tree = treeFromParam("/robot_description")
        if not ok:
            rospy.logerr("Failed to parse /robot_description into KDL tree.")
            rospy.signal_shutdown("KDL parse failed")
            return
        self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)

        # 状態管理
        self.initial_pose_set = False
        self.last_joint_positions = [0.0] * len(self.joint_names)

        # Pub/Sub
        self.gazebo_joint_pub = rospy.Publisher(
            "/me6_robot/joint_controller/command", JointTrajectory, queue_size=1
        )
        self.real_machine_joint_pub = rospy.Publisher(
            "/me6_robot/joint_controller/follow_joint_trajectory/goal",
            FollowJointTrajectoryActionGoal, queue_size=10
        )
        rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.mocap_cb)
        rospy.Subscriber("/me6_robot/joint_states", JointState, self.jointstate_cb)

    def _fk_from_joints(self, joint_positions):
        """
        joint_positions: list[float] 長さが self.kdl_chain.getNrOfJoints() と一致していること
        戻り値: (pos: np.array(3,), quat: np.array(4,))  失敗時は (None, None)
        """
        if len(joint_positions) != self.kdl_chain.getNrOfJoints():
            rospy.logwarn(f"FK: joint size mismatch. expected {self.kdl_chain.getNrOfJoints()}, got {len(joint_positions)}")
            return None, None

        q = kdl.JntArray(self.kdl_chain.getNrOfJoints())
        for i, val in enumerate(joint_positions):
            q[i] = float(val)

        frame = kdl.Frame()
        status = self.fk_solver.JntToCart(q, frame)
        if status < 0:
            rospy.logwarn("FK solver failed.")
            return None, None

        p = frame.p
        R = frame.M
        # PyKDL は (x, y, z, w) で返す
        x, y, z, w = R.GetQuaternion()
        pos = np.array([p[0], p[1], p[2]])
        quat = np.array([x, y, z, w])
        return pos, quat

    def jointstate_cb(self, msg: JointState):
        # 関節順を self.joint_names に合わせて並べ替え
        joint_positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_positions.append(msg.position[idx])
            else:
                joint_positions.append(0.0)

        # 毎回更新（IK の初期解にも効く）
        self.last_joint_positions = joint_positions

        # 初回だけ FK で初期 EE 姿勢を確定（実機で TF が無いケース）
        if not self.initial_pose_set:
            pos, quat = self._fk_from_joints(joint_positions)
            if pos is not None:
                self.center_pos = pos
                self.center_quat = quat
                self.initial_pose_set = True
                rospy.loginfo(f"Initial end-effector pose from FK: pos={self.center_pos}, quat={self.center_quat}")
            else:
                rospy.logwarn_throttle(1.0, "Waiting for valid FK to set initial end-effector pose...")

    def mocap_cb(self, msg: PoseStamped):
        if not self.initial_pose_set:
            rospy.logwarn_throttle(1.0, "Initial end-effector pose not set yet (FK).")
            return

        # （例）固定ターゲット。必要に応じて msg を使った目標に置換してください
        target_pos = np.array([0.1, 0.2, 0.4])
        target_quat = np.array([0.0, 0.0, 0.0, 1.0])

        sol = self.ik_solver.get_ik(
            self.last_joint_positions,
            target_pos[0], target_pos[1], target_pos[2],
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]
        )

        if sol:
            point = JointTrajectoryPoint()
            point.positions = sol
            point.time_from_start = rospy.Duration(3.0)

            traj = JointTrajectory()
            traj.joint_names = self.joint_names
            traj.points.append(point)
            self.gazebo_joint_pub.publish(traj)

            goal_msg = FollowJointTrajectoryActionGoal()
            goal_msg.goal.trajectory.joint_names = self.joint_names
            goal_msg.goal.trajectory.points.append(point)
            # self.real_machine_joint_pub.publish(goal_msg)
        else:
            rospy.logwarn_throttle(1.0, "IK solution not found for given pose.")

if __name__ == "__main__":
    try:
        teleop = MocapTeleopME6Test(urdf_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
