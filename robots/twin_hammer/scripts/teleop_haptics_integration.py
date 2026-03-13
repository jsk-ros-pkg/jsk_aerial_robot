#!/usr/bin/env python

import rospy
import time
import math
import numpy as np
import tf.transformations as tf
from std_msgs.msg import UInt8, String, Int8
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped, WrenchStamped, Vector3Stamped
from scipy.spatial.transform import Rotation as R

def exponential(x, base, k_exp):
  return pow(x, base) * k_exp

def logarithm(x, base, k_log):
  return math.log(x, base) * k_log

class teleop_haptics_integration():

  def __init__(self):

    # Parameters
    self.robot_name = rospy.get_param("~robot_name", "gimbalrotor")
    self.control_mode = rospy.get_param("~control_mode", "pos") # "pos" or "vel"
    self.convert_method = rospy.get_param("~convert_method", "log") # "prop" or "exp" or "log"
    self.frame = rospy.get_param("~frame", "local") # "local" or "world"
    self.feedback_from_ang = rospy.get_param("~feedback_from_ang", "False")

    # Publishers
    self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
    self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
    self.feedback_pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)

    # Subscribers
    self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_state_cb)
    self.device_pos_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_pos_cb)
    self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
    self.teleop_mode_sub = rospy.Subscriber('/twin_hammer/teleop_mode', String, self.teleop_mode_cb)
    self.robot_wrench_sub = rospy.Subscriber('/cfs/data', WrenchStamped, self.robot_wrench_cb)

    # Messages
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
    self.target_att_nav = Vector3Stamped()
    self.haptics_wrench_msg = WrenchStamped()

    # States
    self.hovering = False
    self.landing = False
    self.device_pos = [None]*3
    self.device_att = [None]*3
    self.robot_pos = [None]*3
    self.robot_att = [None]*3
    self.device_init_pos = [None]*3
    self.device_init_att = [None]*3
    self.device_initialize_flag = False
    self.robot_init_pos = [None]*3
    self.robot_init_att = [None]*3
    self.robot_initialize_flag = False
    self.robot_vel_mode_fix_pos = [None]*3
    self.robot_vel_mode_fix_att = [None]*3
    self.device_att_unwrapped = [0.0]*3
    self.device_att_prev = [0.0]*3

    self.wait_flag = False
    self.pos_scale = 1.0
    self.vel_scale = 0.3
    self.ang_vel_scale = 0.1
    self.feedback_force_scale = 10.0
    self.feedback_torque_scale = 1.0
    self.robot_wrench = [0.0]*6
    self.filtered_robot_wrench_local = [0.0]*6
    self.Ad_R_robot = np.identity(6)
    self.Ad_R_inv_device = np.identity(6)
    self.moment_arm = np.array([-(0.044 + 0.025), 0, 0])
    self.k_p = 1.0
    self.exp_base = 1.45
    self.log_base = 1.45
    self.k_exp = 0.4
    self.k_log = 1.0
    self.k_att_diff = 1.0

  def flight_state_cb(self, msg):
    if msg.data == 5:
      self.hovering = True
    if msg.data == 4:
      self.landing = True

  def device_pos_cb(self, msg):
    self.device_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rot = R.from_quat(q)
    self.device_att = rot.as_euler('xyz')
    for i in range(3):
      current_angle = self.device_att[i]
      delta_angle = (current_angle - self.device_att_prev[i] + np.pi) % (2 * np.pi) - np.pi
      self.device_att_unwrapped[i] += delta_angle
      self.device_att_prev[i] = current_angle
    R_mat = rot.as_matrix()
    self.Ad_R_inv_device = np.block([
      [R_mat.T, np.zeros((3,3))],
      [np.zeros((3,3)), R_mat.T]
    ])
    if not self.device_initialize_flag:
      self.device_init_pos = self.device_pos
      self.device_init_att = self.device_att
      self.device_initialize_flag = True

  def robot_pos_cb(self, msg):
    self.robot_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rot = R.from_quat(q)
    self.robot_att = rot.as_euler('xyz')
    R_mat = rot.as_matrix()
    self.Ad_R_robot = np.block([
      [R_mat, np.zeros((3,3))],
      [np.zeros((3,3)), R_mat]
    ])
    if not self.robot_initialize_flag:
      self.robot_init_pos = self.robot_pos
      self.robot_init_att = self.robot_att
      self.robot_initialize_flag = True

  def teleop_mode_cb(self, msg):
    self.wait_flag = False
    self.device_initialize_flag = False
    self.robot_initialize_flag = False
    if msg.data == "pos":
      self.control_mode = "pos"
    if msg.data == "vel":
      self.control_mode = "vel"

  def robot_wrench_cb(self, msg):
    fx = msg.wrench.force.z
    fy = msg.wrench.force.x
    fz = msg.wrench.force.y
    tx = msg.wrench.torque.z
    ty = msg.wrench.torque.x
    tz = msg.wrench.torque.y
    wrench_local = [fx, fy, fz, tx, ty, tz]
    delay_param = 0.05
    for i in range(6):
      self.filtered_robot_wrench_local[i] = (1 - delay_param) * self.filtered_robot_wrench_local[i] + delay_param * wrench_local[i]
    wrench_world = np.dot(self.Ad_R_robot, self.filtered_robot_wrench_local)
    if self.frame == "local":
      self.robot_wrench = self.filtered_robot_wrench_local
    else:
      self.robot_wrench = wrench_world

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      target_pos = [0.0]*3
      target_att = [0.0]*3
      target_vel = [0.0]*3
      target_ang_vel = [0.0]*3
      feedback_wrench = [0.0]*6

      if self.device_init_pos is None or not self.hovering:
        self.device_initialize_flag = False
      if self.robot_init_pos is None or not self.hovering:
        self.robot_initialize_flag = False

      if self.device_initialize_flag and self.robot_initialize_flag:

        """ calc target pos and vel """
        vel_mode_pos_thre = [0.1,0.1,0.1]
        vel_mode_att_thre = [0.05,0.05,0.05]
        for i in range(3):
          device_pos_diff = self.device_pos[i] - self.device_init_pos[i]
          # device_att_diff = self.device_att_unwrapped[i] - self.device_init_att[i]
          device_att_diff = self.device_att[i] - self.device_init_att[i]
          target_pos[i] = (self.robot_init_pos[i] + device_pos_diff) * self.pos_scale
          target_att[i] = self.device_att[i]
          target_vel[i] = self.robot_pos[i] + device_pos_diff * self.vel_scale
          target_ang_vel[i] = self.robot_att[i] + device_att_diff * self.ang_vel_scale
          """ position fix for vel mode """
          if abs(target_vel[i]-self.robot_pos[i]) < vel_mode_pos_thre[i]:
            if self.robot_vel_mode_fix_pos[i] == None:
              self.robot_vel_mode_fix_pos[i] = self.robot_pos[i]
            target_vel[i] = self.robot_vel_mode_fix_pos[i]
          else:
            self.robot_vel_mode_fix_pos[i] = None
          if abs(target_ang_vel[i]-self.robot_att[i]) < vel_mode_att_thre[i]:
            if self.robot_vel_mode_fix_att[i] == None:
              self.robot_vel_mode_fix_att[i] = self.robot_att[i]
            target_ang_vel[i] = self.robot_vel_mode_fix_att[i]
          else:
            self.robot_vel_mode_fix_att[i] = None
          if self.control_mode == "vel":
            feedback_wrench[i] = - device_pos_diff * self.feedback_force_scale
            feedback_wrench[i+3] = - device_att_diff * self.feedback_torque_scale

        """ convert feedback wrench with log """
        k_force = 1.5
        k_torque = 1.0
        log_base = 1.45
        for i in range(3):
          if feedback_wrench[i] >= 0:
            feedback_wrench[i] = logarithm(feedback_wrench[i]+1, log_base, k_force)
          else:
            feedback_wrench[i] = -logarithm(-(feedback_wrench[i]-1), log_base, k_force)
          if feedback_wrench[i+3] >= 0:
            feedback_wrench[i+3] = logarithm(feedback_wrench[i+3]+1, log_base, k_torque)
          else:
            feedback_wrench[i+3] = -logarithm(-(feedback_wrench[i+3]-1), log_base, k_torque)

        """ calc feedback wrench from force sensor """
        haptics_wrench = [0.0]*6
        for i in range(len(self.robot_wrench)):
          wrench_i = self.robot_wrench[i]
          """ propotional conversion """
          if self.convert_method == "prop":
            haptics_wrench[i] = wrench_i * self.k_p
          """ exponential conversion """
          if self.convert_method == "exp":
            if wrench_i >= 0.0:
              haptics_wrench[i] = exponential(wrench_i, self.exp_base, self.k_exp)
            else:
              haptics_wrench[i] = -exponential(-wrench_i, self.exp_base, self.k_exp)
          """ log conversion """
          if self.convert_method == "log":
            '''
            if wrench_i>self.range_log:
              haptics_wrench[i] = logarithm(wrench_i, self.log_base, self.k_log)
            elif wrench_i<-self.range_log:
              haptics_wrench[i] = -logarithm(-wrench_i, self.log_base, self.k_log)
            else:
              haptics_wrench[i] = wrench_i * self.a_log
            '''
            if wrench_i >= 0:
              haptics_wrench[i] = logarithm(wrench_i+1, self.log_base, self.k_log)
            else:
              haptics_wrench[i] = -logarithm(-(wrench_i-1), self.log_base, self.k_log)

        """ force feedback from ang diff """
        if self.feedback_from_ang:
          att_diff = self.robot_att - self.device_att
          for i in range(len(att_diff)):
            if att_diff[i] >= 0:
              wrench_from_pos_diff = logarithm(att_diff[i]+1,self.log_base,self.k_att_diff)
            else:
              wrench_from_pos_diff = logarithm(-(att_diff[i]-1),self.log_base,self.k_att_diff)
            haptics_wrench[i] += wrench_from_pos_diff

        """ convert frame of feedback wrench """
        if self.frame == "world":
          haptics_wrench = np.dot(self.Ad_R_inv_device,haptics_wrench)

        for i in range(6):
          haptics_wrench[i] += feedback_wrench[i]

        """ limitation of feedback wrench for safety """
        force_limit = 10
        torque_limit = 1.5
        for i in range(3):
          haptics_wrench[i] = max(min(haptics_wrench[i], force_limit), -force_limit)
          haptics_wrench[i+3] = max(min(haptics_wrench[i+3], torque_limit), -torque_limit)

        """ limitation of target_pos and att in 0066 """
        """ x """
        if self.robot_pos[0] > 2.0:
          target_pos[0] = 2.0
          target_vel[0] = 2.0
        if self.robot_pos[0] < -1.3:
          target_pos[0] = -1.3
          target_vel[0] = -1.3
        """ y """
        if self.robot_pos[1] > 1.6:
          target_pos[1] = 1.6
          target_vel[1] = 1.6
        if self.robot_pos[1] < -1.6:
          target_pos[1] = -1.6
          target_vel[1] = -1.6
        """ z """
        if self.robot_pos[2] > 1.2:
          target_pos[2] = 1.2
          target_vel[2] = 1.2
        if self.robot_pos[2] < 0.3:
          target_pos[2] = 0.3
          target_vel[2] = 0.3
        """ roll and pitch """
        limit_angle = 0.35
        for i in range(2):
          target_att[i] = max(min(target_att[i], limit_angle), -limit_angle)
          target_ang_vel[i] = max(min(target_ang_vel[i], limit_angle), -limit_angle)


        if self.control_mode == "pos":
          self.flight_nav.target_pos_x = target_pos[0]
          self.flight_nav.target_pos_y = target_pos[1]
          self.flight_nav.target_pos_z = target_pos[2]
          self.flight_nav.target_yaw = target_att[2]
          self.flight_nav.target_roll = target_att[0]
          self.flight_nav.target_pitch = target_att[1]
          self.target_att_nav.vector.x = target_att[0]
          self.target_att_nav.vector.y = target_att[1]

        if self.control_mode == "vel":
          self.flight_nav.target_pos_x = target_vel[0]
          self.flight_nav.target_pos_y = target_vel[1]
          # self.flight_nav.target_vel_x = target_vel[0]
          # self.flight_nav.target_vel_y = target_vel[1]
          self.flight_nav.target_pos_z = target_pos[2] # not use vel for safety
          self.flight_nav.target_yaw = target_att[2]
          # self.flight_nav.target_omega_z = target_ang_vel[2]
          self.flight_nav.target_roll = target_att[0] # not use vel for safety
          self.flight_nav.target_pitch = target_att[1] # not use vel for safety
          self.target_att_nav.vector.x = target_att[0] # not use vel for safety
          self.target_att_nav.vector.y = target_att[1] # not use vel for safety

        self.haptics_wrench_msg.wrench.force.x = haptics_wrench[0]
        self.haptics_wrench_msg.wrench.force.y = haptics_wrench[1]
        self.haptics_wrench_msg.wrench.force.z = haptics_wrench[2]
        self.haptics_wrench_msg.wrench.torque.x = haptics_wrench[3]
        self.haptics_wrench_msg.wrench.torque.y = haptics_wrench[4]
        self.haptics_wrench_msg.wrench.torque.z = haptics_wrench[5]

      if self.hovering and not self.landing:
        if not self.wait_flag:
          rospy.sleep(3.0)
          self.wait_flag = True
        self.nav_pub.publish(self.flight_nav)
        self.att_pub.publish(self.target_att_nav)
        self.feedback_pub.publish(self.haptics_wrench_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("teleop_haptics_integration")
  Tracker = teleop_haptics_integration()
  Tracker.main()
