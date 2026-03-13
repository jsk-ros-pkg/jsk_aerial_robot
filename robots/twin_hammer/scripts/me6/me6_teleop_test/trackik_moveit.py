#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import xacro
import numpy as np
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler, quaternion_multiply
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam

from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.srv import GetStateValidity, GetStateValidityRequest
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState as JS

class TeleopTracIKMoveItCollision:
    def __init__(self, urdf_path, group_name="me6_arm"):
        roscpp_initialize([])
        rospy.init_node("teleop_tracik_moveit_collision")

        # ---- params ----
        self.delta_angle_thre = 0.3
        self.pub_rate = 5
        self.teleop_scale = 0.4
        self.offset_pos = np.array([0.0,0.0,0.5])
        self.offset_quat = quaternion_from_euler(0, np.pi/2, 0)
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        self.base_link = "base_link"
        self.ee_link = "Link6"
        self.group_name = group_name

        # ---- URDF ----
        try:
            robot_description_config = xacro.process_file(urdf_path)
            rospy.set_param("/robot_description", robot_description_config.toxml())
            rospy.loginfo("URDF loaded to /robot_description")
        except Exception as e:
            rospy.logerr("URDF load failed: %s", e)
            rospy.signal_shutdown("URDF load failed")
            return

        # ---- FK ----
        ok, kdl_tree = treeFromParam("/robot_description")
        if not ok:
            rospy.logerr("Failed to parse /robot_description into KDL tree.")
            rospy.signal_shutdown("KDL parse failed")
            return
        self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)

        # ---- IK ----
        self.ik_solver = IK(self.base_link, self.ee_link)

        # ---- MoveIt collision service ----
        self.robot = RobotCommander()
        rospy.wait_for_service("/check_state_validity")
        self.csv_srv = rospy.ServiceProxy("/check_state_validity", GetStateValidity)

        # ---- state ----
        self.current_joint_positions = np.zeros(len(self.joint_names))
        self.last_joint_positions = np.zeros(len(self.joint_names))
        self.joint_ready = False

        self.initial_joint_received = False
        self.center_pos = None
        self.center_quat = None
        self.device_init_pos = None
        self.device_init_quat = None
        self.last_valid_ee_pos = None

        # ---- IO ----
        self.gazebo_pub = rospy.Publisher("/me6_robot/joint_controller/command", JointTrajectory, queue_size=1)
        self.real_pub = rospy.Publisher("/me6_robot/joint_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
        self.debug_target_pub = rospy.Publisher("/debug_target_pos", PoseStamped, queue_size=1)
        self.error_feedback_pub = rospy.Publisher("/twin_hammer/haptics_wrench", WrenchStamped, queue_size=1)

        rospy.Subscriber("/me6_robot/joint_states", JointState, self.jointstate_cb)
        rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.mocap_cb)

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
        quat = np.array([0,0,0,1])
        return pos, quat

    def limit_delta(self, next_joint):
        delta = next_joint - self.last_joint_positions
        delta = np.clip(delta, -self.delta_angle_thre, self.delta_angle_thre)
        return self.last_joint_positions + delta

    def is_self_collision(self, joints):
        req = GetStateValidityRequest()
        rs = RobotState()
        rs.joint_state = JS()
        rs.joint_state.name = self.joint_names
        rs.joint_state.position = joints.tolist()
        req.robot_state = rs
        req.group_name = self.group_name
        res = self.csv_srv(req)
        return not res.valid  # valid==False → 衝突あり

    def publish_joint_command(self, joint_positions):
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start = rospy.Duration(1.0/self.pub_rate)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(point)
        self.gazebo_pub.publish(traj)

        goal_msg = FollowJointTrajectoryActionGoal()
        goal_msg.goal.trajectory.joint_names = self.joint_names
        goal_msg.goal.trajectory.points.append(point)
        self.real_pub.publish(goal_msg)

        self.last_joint_positions = joint_positions

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

    # ---------- Main ----------
    def main(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            if self.joint_ready and self.latest_mocap_pos is not None and self.initial_joint_received:
                delta_pos = (self.latest_mocap_pos - self.device_init_pos) * self.teleop_scale
                target_pos = self.center_pos + delta_pos
                target_quat = quaternion_multiply(self.latest_mocap_quat, self.offset_quat)

                sol = self.ik_solver.get_ik(
                    self.last_joint_positions,
                    target_pos[0], target_pos[1], target_pos[2],
                    target_quat[0], target_quat[1], target_quat[2], target_quat[3]
                )

                if sol is not None:
                    sol = np.array(sol)
                    sol_limited = self.limit_delta(sol)
                    if not self.is_self_collision(sol_limited):
                        self.publish_joint_command(sol_limited)
                        self.last_valid_ee_pos = target_pos.copy()
                        self.publish_debug_pose(target_pos)
                    else:
                        rospy.logwarn_throttle(1.0, "Self-collision predicted by MoveIt. Command skipped.")
                        self.publish_error_feedback(target_pos)
                else:
                    rospy.logwarn_throttle(1.0, "IK solution not found")
                    self.publish_error_feedback(target_pos)

            rate.sleep()

        roscpp_shutdown()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.realpath(__file__))
    urdf_path = os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")
    node = TeleopTracIKMoveItCollision(urdf_path, group_name="me6_arm")
    node.main()
