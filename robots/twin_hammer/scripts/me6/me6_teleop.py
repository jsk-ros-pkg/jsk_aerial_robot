#!/usr/bin/env python

import os
import rospy
import xacro
import math
import numpy as np
import PyKDL as kdl
from kdl_parser_py.urdf import treeFromParam
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler, quaternion_multiply
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryActionGoal
from std_msgs.msg import Int32
import tf.transformations as tf


class TeleopCollisionList:
   def __init__(self, urdf_path):
       rospy.init_node("teleop_collision_list")

       self.delta_angle_thre = 0.1
       self.pub_rate = 5
       self.teleop_scale = 0.8
       self.feedback_scale = 1.0
       self.feedback_log_base = 1.45
       self.epsilon = 0.05  # [rad] joint angle thre for collision detection
    #    self.robot_init_joint_angles = np.array([2.37, -1.7, 2.0, -0.5, 0.77, 0.0])
       self.robot_init_joint_angles = np.array([0.87, -1.6, 1.6, 0.0, 2.27, 0.0])
       self.joints_upper_limit = np.array([2.37, -0.7, 2.5, 1.5, 2.57, 0.1])
       self.joints_lower_limit = np.array([0.87, -2.0, 0.0, -1.5, 0.47, -0.1])
       self.offset_pos = np.array([0.0,0.0,0.0])
       self.offset_quat = quaternion_from_euler(0, np.pi/2, 0)
       self.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
       self.base_link = "base_link"
       self.ee_link = "Link6"

       try:
           robot_description_config = xacro.process_file(urdf_path)
           rospy.set_param("/robot_description", robot_description_config.toxml())
           rospy.loginfo("URDF loaded to /robot_description")
       except Exception as e:
           rospy.logerr("URDF load failed: %s", e)
           rospy.signal_shutdown("URDF load failed")
           return

       script_dir = os.path.dirname(os.path.realpath(__file__))
       unsafe_path = os.path.join(script_dir, "me6_unsafe_configs.npy")
       if not os.path.exists(unsafe_path):
           rospy.logwarn("unsafe_configs.npy not found. Proceeding without list (no filtering).")
           self.unsafe = np.zeros((0, len(self.joint_names)))
       else:
           self.unsafe = np.load(unsafe_path)
           if self.unsafe.ndim == 1 and self.unsafe.size == 0:
               self.unsafe = np.zeros((0, len(self.joint_names)))
           rospy.loginfo("Loaded unsafe list: %s", self.unsafe.shape)

       ok, kdl_tree = treeFromParam("/robot_description")
       if not ok:
           rospy.logerr("Failed to parse /robot_description into KDL tree.")
           rospy.signal_shutdown("KDL parse failed")
           return
       self.kdl_chain = kdl_tree.getChain(self.base_link, self.ee_link)
       self.fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)
       self.ik_solver = IK(self.base_link, self.ee_link, solve_type="Distance", timeout=0.05)

       self.current_joint_positions = np.zeros(len(self.joint_names))
       self.last_joint_positions = np.zeros(len(self.joint_names))
       self.robot_pos_init_flag = False
       self.center_pos, self.center_quat = self.compute_fk(self.robot_init_joint_angles)
       self.device_init_pos = None
       self.device_init_quat = None
       self.latest_mocap_pos = None
       self.latest_mocap_quat = None
       self.last_valid_ee_pos = None
       self.cfs_connection_state = False

       self.gazebo_pub = rospy.Publisher("/me6_robot/joint_controller/command", JointTrajectory, queue_size=1)
       self.real_pub = rospy.Publisher("/me6_robot/joint_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)
       self.debug_target_pub = rospy.Publisher("/debug/target_pos", PoseStamped, queue_size=1)
       self.error_feedback_pub = rospy.Publisher("/twin_hammer/haptics_wrench", WrenchStamped, queue_size=1)
       rospy.Subscriber("/twin_hammer/mocap/pose", PoseStamped, self.mocap_cb)
       rospy.Subscriber('/cfs/connection_state', Int32, self.cfs_connection_cb, queue_size=1)


   def mocap_cb(self, msg: PoseStamped):
       pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
       quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
       self.latest_mocap_pos = pos
       self.latest_mocap_quat = quat

       if self.device_init_pos is None:
           self.device_init_pos = pos + self.offset_pos
           self.device_init_quat = quat
      
   def cfs_connection_cb(self, msg: Int32):
       self.cfs_connection_state = msg.data

   def zero_roll_from_quaternion(self, q):
       """
       q: (x, y, z, w)
       return: roll を 0 にしたクオータニオン (x, y, z, w)
       """
       # クオータニオン → オイラー角
       roll, pitch, yaw = tf.euler_from_quaternion(q)

       # roll を 0 に
       roll = 0.0

       # オイラー角 → クオータニオン
       q_new = tf.quaternion_from_euler(roll, pitch, yaw)
       return q_new

   def limit_target_x(self, z):
       wall_tan = 1.95/0.52 # z/x
       contact_limit_point = [0.31, 0.39] # [x,z]
       b = contact_limit_point[1] - wall_tan*contact_limit_point[0] # z = tan*x + b
       x_limit = (z - b) / wall_tan
       return x_limit

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

   def joints_clamp(self, joint_positions):
       clamped_joints = np.array([0.0] * len(joint_positions))
       for i in range(len(joint_positions)):
           if joint_positions[i] > self.joints_upper_limit[i]:
               clamped_joints[i] = self.joints_upper_limit[i]
           elif joint_positions[i] < self.joints_lower_limit[i]:
               clamped_joints[i] = self.joints_lower_limit[i]
           else:
               clamped_joints[i] = joint_positions[i]
       return clamped_joints

   def is_in_unsafe_list(self, joints):
       if self.unsafe.shape[0] == 0:
           return False
       diff = np.abs(self.unsafe - joints)  # [N, 6]
       max_abs = np.max(diff, axis=1)       # [N]
       return np.any(max_abs <= self.epsilon)

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

   def publish_debug_pose(self, target_pos, target_quat):
       msg = PoseStamped()
       msg.header.stamp = rospy.Time.now()
       msg.pose.position.x = target_pos[0]
       msg.pose.position.y = target_pos[1]
       msg.pose.position.z = target_pos[2]
       msg.pose.orientation.x = target_quat[0]
       msg.pose.orientation.y = target_quat[1]
       msg.pose.orientation.z = target_quat[2]
       msg.pose.orientation.w = target_quat[3]
       self.debug_target_pub.publish(msg)

   def publish_error_feedback(self, target_pos):
       if self.last_valid_ee_pos is None:
           return
       feedback_force = [0.0, 0.0, 0.0]
       for i in range(3):
           error = target_pos[i] - self.last_valid_ee_pos[i]
           if error>0:
               log_error = math.log(error, self.feedback_log_base)
           else:
               log_error = math.log(-error, self.feedback_log_base)
           feedback_force[i] = - log_error * self.feedback_scale
       wrench_msg = WrenchStamped()
       wrench_msg.wrench.force.x = feedback_force[0]
       wrench_msg.wrench.force.y = feedback_force[1]
       wrench_msg.wrench.force.z = feedback_force[2]
       self.error_feedback_pub.publish(wrench_msg)

   def main(self):
       rate = rospy.Rate(self.pub_rate)
       while not rospy.is_shutdown():
           if not self.robot_pos_init_flag:
               rospy.loginfo("robot position initializing")
               self.publish_joint_command(self.robot_init_joint_angles)
               self.last_valid_ee_pos = self.center_pos
               self.publish_debug_pose(self.center_pos,self.center_quat)
               rospy.sleep(2.0)
               self.robot_pos_init_flag = True
               rospy.loginfo("robot position initialized!!")

           if self.latest_mocap_pos is not None and self.robot_pos_init_flag:
               delta_pos = (self.latest_mocap_pos - self.device_init_pos) * self.teleop_scale
               delta_pos[0] = (self.latest_mocap_pos[0] - self.device_init_pos[0]) * 0.5
               target_pos = self.center_pos + delta_pos
               target_quat = quaternion_multiply(self.latest_mocap_quat, self.offset_quat)
               target_x_limit = self.limit_target_x(target_pos[2])
               if target_pos[0] > target_x_limit:
                  target_pos[0] = target_x_limit

               sol = self.ik_solver.get_ik(
                   self.last_joint_positions,
                   target_pos[0], target_pos[1], target_pos[2],
                   target_quat[0], target_quat[1], target_quat[2], target_quat[3]
               )

               if sol is not None:
                   sol = np.array(sol)
                   sol[5] += np.pi/2 # offset of the ee joint
                   sol_limited = self.limit_delta(sol)
                   clamped_joints = self.joints_clamp(sol_limited)
                   if not self.is_in_unsafe_list(clamped_joints):
                       self.publish_joint_command(clamped_joints)
                       self.last_valid_ee_pos = target_pos.copy()
                       self.publish_debug_pose(target_pos,target_quat)
                   else:
                       rospy.logwarn_throttle(1.0, "Rejected by unsafe joint list (near-collision).")
                       # self.publish_error_feedback(target_pos)
               else:
                   rospy.logwarn_throttle(1.0, "IK solution not found")
                   # self.publish_error_feedback(target_pos)

           rate.sleep()


if __name__ == "__main__":
   script_dir = os.path.dirname(os.path.realpath(__file__))
   urdf_path = os.path.join(script_dir, "../../../TCP-IP-ROS-6AXis/dobot_gazebo/urdf/me6_robot.xacro")
   node = TeleopCollisionList(urdf_path)
   node.main()



