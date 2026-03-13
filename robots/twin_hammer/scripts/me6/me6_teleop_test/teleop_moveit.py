#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import xacro
import numpy as np
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from tf.transformations import quaternion_from_euler, quaternion_multiply
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

from moveit_commander import MoveGroupCommander, RobotCommander, roscpp_initialize, roscpp_shutdown

from trac_ik_python.trac_ik import IK  # Δ角制限用にIKで見積もり

class TeleopMoveIt:
    def __init__(self, urdf_path, group_name="me6_arm"):
        roscpp_initialize([])
        rospy.init_node("teleop_moveit_node")

        # ---- パラメータ ----
        self.delta_angle_thre = 0.5  # [rad] 1ステップでの関節差分の上限
        self.pub_rate = 1            # [Hz]
        self.teleop_scale = 0.5
        self.offset_pos = np.array([0.0, 0.0, 0.5])
        self.offset_quat = quaternion_from_euler(0, np.pi/2, 0)
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        self.base_link = "base_link"
        self.ee_link = "Link6"

        # ---- URDF ロード ----
        try:
            robot_description_config = xacro.process_file(urdf_path)
            rospy.set_param("/robot_description", robot_description_config.toxml())
            rospy.loginfo("URDF loaded to /robot_description")
        except Exception as e:
            rospy.logerr("URDF load failed: %s", e)
            rospy.signal_shutdown("URDF load failed")
            return

        # ---- FK 用 KDL ----
        ok, kdl_tree = treeFromParam("/robot_description")
        if not ok:
            rospy.logerr("Failed to parse /robot_description into KDL tree.")
            rospy.signal_shutdown("KDL parse failed")
            return
        self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)

        # ---- MoveIt ----
        self.robot = RobotCommander()
        self.group = MoveGroupCommander(group_name)
        # 実機のMoveIt設定に合わせて必要ならスケーリング
        self.group.set_max_velocity_scaling_factor(0.5)
        self.group.set_max_acceleration_scaling_factor(0.5)

        # ---- TracIK（Δ角見積用） ----
        self.ik_estimator = IK(self.base_link, self.ee_link)

        # ---- 状態管理 ----
        self.current_joint_positions = np.zeros(len(self.joint_names))
        self.last_joint_positions = np.zeros(len(self.joint_names))
        self.joint_ready = False

        self.initial_joint_received = False
        self.center_pos = None
        self.center_quat = None
        self.device_init_pos = None
        self.device_init_quat = None
        self.last_valid_ee_pos = None

        # ---- I/O ----
        self.debug_target_pub = rospy.Publisher("/debug_target_pos", PoseStamped, queue_size=1)
        self.error_feedback_pub = rospy.Publisher("/twin_hammer/haptics_wrench", WrenchStamped, queue_size=1)

        rospy.Subscriber("/me6_robot/joint_states", JointState, self.jointstate_cb)
        rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.mocap_cb)

        # Mocap最新値
        self.latest_mocap_pos = None
        self.latest_mocap_quat = None

    # ---------- Callbacks ----------
    def mocap_cb(self, msg: PoseStamped):
        pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        self.latest_mocap_pos = pos
        self.latest_mocap_quat = quat

        if self.device_init_pos is None:
            self.device_init_pos = pos + self.offset_pos
            self.device_init_quat = quat

    def jointstate_cb(self, msg: JointState):
        joint_positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                joint_positions.append(msg.position[idx])
            else:
                joint_positions.append(0.0)
        self.current_joint_positions = np.array(joint_positions)
        self.last_joint_positions = np.array(joint_positions)
        self.joint_ready = True

        if not self.initial_joint_received:
            self.center_pos, self.center_quat = self.compute_fk(self.current_joint_positions)
            rospy.loginfo("Center EE pos from FK: %s", self.center_pos)
            self.initial_joint_received = True

    # ---------- Helpers ----------
    def compute_fk(self, joint_positions):
        fk_joint_array = kdl.JntArray(len(joint_positions))
        for i, val in enumerate(joint_positions):
            fk_joint_array[i] = val
        ee_frame = kdl.Frame()
        self.fk_solver.JntToCart(fk_joint_array, ee_frame)
        pos = np.array([ee_frame.p[0], ee_frame.p[1], ee_frame.p[2]])
        quat = np.array([0,0,0,1])  # 必要に応じて回転も算出可
        return pos, quat

    def limit_by_delta_angles_via_pose_scaling(self, target_pos, target_quat):
        """
        MoveItは内部で軌道を生成するため、直接Δ角を制限できない。
        ここではTracIKでIK解を「見積もり」し、Δ角が閾値を超える場合は
        目標Poseを現在から近づけてスケールダウンする。
        """
        sol = self.ik_estimator.get_ik(
            self.last_joint_positions,
            target_pos[0], target_pos[1], target_pos[2],
            target_quat[0], target_quat[1], target_quat[2], target_quat[3]
        )
        if sol is None:
            return target_pos, target_quat  # そのまま（計画に任せる）

        sol = np.array(sol)
        delta = sol - self.last_joint_positions
        max_delta = np.max(np.abs(delta))
        if max_delta <= self.delta_angle_thre:
            return target_pos, target_quat

        scale = max(self.delta_angle_thre / (max_delta + 1e-9), 0.1)  # 下限スケール
        # 位置のみスケール（姿勢はそのまま or slerpしたければ拡張）
        cur_pos = self.center_pos if self.last_valid_ee_pos is None else self.last_valid_ee_pos
        new_pos = cur_pos + (target_pos - cur_pos) * scale
        return new_pos, target_quat

    def publish_debug_pose(self, target_pos):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = target_pos[0]
        msg.pose.position.y = target_pos[1]
        msg.pose.position.z = target_pos[2]
        self.debug_target_pub.publish(msg)

    def publish_error_feedback(self, target_pos):
        if self.last_valid_ee_pos is None:
            return
        error_vec = (target_pos - self.last_valid_ee_pos) * self.teleop_scale
        wrench_msg = WrenchStamped()
        wrench_msg.wrench.force.x = error_vec[0]
        wrench_msg.wrench.force.y = error_vec[1]
        wrench_msg.wrench.force.z = error_vec[2]
        self.error_feedback_pub.publish(wrench_msg)

    # ---------- Main Loop ----------
    def main(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            if self.joint_ready and self.latest_mocap_pos is not None and self.initial_joint_received:
                delta_pos = (self.latest_mocap_pos - self.device_init_pos) * self.teleop_scale
                target_pos = self.center_pos + delta_pos
                target_quat = quaternion_multiply(self.latest_mocap_quat, self.offset_quat)

                # Δ角推定に基づく目標縮小
                target_pos_adj, target_quat_adj = self.limit_by_delta_angles_via_pose_scaling(target_pos, target_quat)

                # MoveItへ目標を渡して計画
                target_pose = PoseStamped()
                target_pose.header.stamp = rospy.Time.now()
                target_pose.header.frame_id = self.base_link
                target_pose.pose.position.x = target_pos_adj[0]
                target_pose.pose.position.y = target_pos_adj[1]
                target_pose.pose.position.z = target_pos_adj[2]
                target_pose.pose.orientation.x = target_quat_adj[0]
                target_pose.pose.orientation.y = target_quat_adj[1]
                target_pose.pose.orientation.z = target_quat_adj[2]
                target_pose.pose.orientation.w = target_quat_adj[3]

                self.group.set_pose_target(target_pose)
                plan = self.group.plan()
                if plan and hasattr(plan, "joint_trajectory") and len(plan.joint_trajectory.points) > 0:
                    self.group.execute(plan, wait=False)
                    self.last_valid_ee_pos = target_pos_adj.copy()
                    self.publish_debug_pose(target_pos_adj)
                else:
                    rospy.logwarn_throttle(0.2, "MoveIt plan failed")
                    self.publish_error_feedback(target_pos_adj)

            rate.sleep()

        roscpp_shutdown()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")
    node = TeleopMoveIt(urdf_path, group_name="me6_arm")
    node.main()
