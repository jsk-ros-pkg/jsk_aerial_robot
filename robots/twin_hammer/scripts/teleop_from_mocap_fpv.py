#!/usr/bin/env python

import rospy
import time
import math
import tf.transformations as tf
from std_msgs.msg import UInt8, String
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped, WrenchStamped
from geometry_msgs.msg import Vector3Stamped

def exponential(x, base, k_exp):
  return pow(x,base) * k_exp

def logarithm(x, base, k_log):
  return math.log(x,base) * k_log

def unwrap_angle(prev, current):
  """
  prev, current: rad
  return: unwrapped current
  """
  if prev is None:
    return current
  diff = current - prev
  while diff > math.pi:
    diff -= 2.0 * math.pi
  while diff < -math.pi:
    diff += 2.0 * math.pi
  return prev + diff

def rotate_xy_world_to_body(dx, dy, yaw):
  c = math.cos(-yaw)
  s = math.sin(-yaw)
  bx = c * dx - s * dy
  by = s * dx + c * dy
  return bx, by
  
def rotate_xy_body_to_world(bx, by, yaw):
  c = math.cos(yaw)
  s = math.sin(yaw)
  wx = c * bx - s * by
  wy = s * bx + c * by
  return wx, wy


class teleop_from_mocap():

  def __init__(self):

    self.robot_name = rospy.get_param("~robot_name", "quadrotor")
    self.control_mode = rospy.get_param("~control_mode", "pos") # "pos" or "vel"
    self.pos_scale = rospy.get_param("~pos_scale", 1.0) # 1.0
    self.vel_scale = rospy.get_param("~vel_scale", 0.3) # 0.3
    self.ang_vel_scale = rospy.get_param("~ang_vel_scale", 0.15)
    self.feedback_force_scale = rospy.get_param("~feedback_force_scale", 5.0)
    self.feedback_torque_scale = rospy.get_param("~feedback_torque_scale", 0.8)
    self.velmode_pos_thre = rospy.get_param("~velmode_pos_thre", 0.25)
    self.velmode_att_thre = rospy.get_param("~velmode_att_thre", 0.25)

    self.nav_pub = rospy.Publisher('/'+self.robot_name+'/uav/nav', FlightNav, queue_size=1)
    # self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rot', DesireCoord, queue_size=1)
    self.att_pub = rospy.Publisher('/'+self.robot_name+'/final_target_baselink_rpy', Vector3Stamped, queue_size=1)
    self.feedback_pub = rospy.Publisher('/twin_hammer/feedback_from_device', WrenchStamped, queue_size=1)
    self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_state_cb)
    self.device_pos_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_pos_cb)
    self.robot_pos_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_pos_cb)
    self.teleop_mode_sub = rospy.Subscriber('/twin_hammer/teleop_mode', String, self.teleop_mode_cb)
    self.flight_nav = FlightNav()
    self.flight_nav.control_frame = FlightNav.WORLD_FRAME
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pos_z_nav_mode = FlightNav.POS_VEL_MODE
    """ for new FlightNav """
    self.flight_nav.roll_nav_mode = FlightNav.POS_VEL_MODE
    self.flight_nav.pitch_nav_mode = FlightNav.POS_VEL_MODE
    self.desire_att_nav = DesireCoord()
    self.target_att_nav = Vector3Stamped()
    self.haptics_wrench_msg = WrenchStamped()

    self.hovering = False
    self.landing = False
    self.device_pos = [None]*3
    self.device_att = [None]*3
    self.robot_pos = [None]*3
    self.robot_att = [None]*3
    self.device_init_pos = [None]*3
    self.device_init_att = [None]*3
    self.robot_init_pos = [None]*3
    self.robot_init_att = [None]*3
    self.robot_vel_mode_fix_pos = [None]*3
    self.robot_vel_mode_fix_att = [None]*3
    self.robot_local_fix_pos = [None]*3
    self.robot_local_fix_att = [None]*3

    self.device_initialize_flag = False
    self.robot_initialize_flag = False
    self.wait_flag = False

    self.device_yaw_unwrapped = None
    self.robot_yaw_unwrapped = None
    self.device_init_yaw_unwrapped = None
    self.robot_init_yaw_unwrapped = None

  def flight_state_cb(self,msg):
    if msg.data == 5:
      self.hovering = True
    if msg.data == 4:
      self.landing = True

  def device_pos_cb(self,msg):
    self.device_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    device_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    # self.device_att = tf.euler_from_quaternion(device_orientation_q)
    roll, pitch, yaw = tf.euler_from_quaternion(device_orientation_q)
    self.device_yaw_unwrapped = unwrap_angle(self.device_yaw_unwrapped, yaw)
    self.device_att = [roll, pitch, self.device_yaw_unwrapped]
    if self.device_initialize_flag == False:
      self.device_init_pos = self.device_pos
      # self.device_init_att = self.device_att
      self.device_init_att = self.device_att
      self.device_init_yaw_unwrapped = self.device_yaw_unwrapped
      self.device_initialize_flag = True

  def robot_pos_cb(self,msg):
    self.robot_pos = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    robot_orientation_q = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    # self.robot_att = tf.euler_from_quaternion(robot_orientation_q)
    roll, pitch, yaw = tf.euler_from_quaternion(robot_orientation_q)
    self.robot_yaw_unwrapped = unwrap_angle(self.robot_yaw_unwrapped, yaw)
    self.robot_att = [roll, pitch, self.robot_yaw_unwrapped]
    if self.robot_initialize_flag == False:
      self.robot_init_pos = self.robot_pos
      # self.robot_init_att = self.robot_att
      self.robot_init_att = self.robot_att
      self.robot_init_yaw_unwrapped = self.robot_yaw_unwrapped
      self.robot_initialize_flag = True

  def teleop_mode_cb(self,msg):
    self.robot_initialize_flag = False
    self.device_initialize_flag = False
    self.wait_flag = False
    if msg.data == "pos":
      self.control_mode = "pos"
    if msg.data == "vel":
      self.control_mode = "vel"   

  def main(self):
    target_pos = [0.0,0.0,0.0]
    target_att = [0.0,0.0,0.0]
    target_vel = [0.0,0.0,0.0]
    target_ang_vel = [0.0,0.0,0.0]
    feedback_wrench = [0.0,0.0,0.0,0.0,0.0,0.0]
    log_base = 1.45

    r = rospy.Rate(40)
    while not rospy.is_shutdown():

      if self.device_init_pos is None or not self.hovering:
        self.device_initialize_flag = False
      if self.robot_init_pos is None or not self.hovering:
        self.robot_initialize_flag = False

      if self.device_initialize_flag and self.robot_initialize_flag:
        """ temp hard cording for FPV"""
        device_dx_world = self.device_pos[0] - self.device_init_pos[0]
        device_dy_world = self.device_pos[1] - self.device_init_pos[1]
        device_dz_world = self.device_pos[2] - self.device_init_pos[2]
        # device_dyaw = self.device_att[2] - self.device_init_att[2]
        device_dyaw = self.device_yaw_unwrapped - self.device_init_yaw_unwrapped
        # device_dx_local, device_dy_local = rotate_xy_world_to_body(
        #   device_dx_world, device_dy_world, self.device_init_att[2])
        device_dx_local, device_dy_local = rotate_xy_world_to_body(
          device_dx_world, device_dy_world, self.device_init_yaw_unwrapped)

        robot_dx_local = device_dx_local * self.pos_scale
        robot_dy_local = device_dy_local * self.pos_scale
        # robot_dx_world, robot_dy_world = rotate_xy_body_to_world(
        #   robot_dx_local, robot_dy_local, self.robot_init_att[2])
        robot_dx_world, robot_dy_world = rotate_xy_body_to_world(
          robot_dx_local, robot_dy_local, self.robot_init_yaw_unwrapped)
        target_pos[0] = self.robot_init_pos[0] + robot_dx_world
        target_pos[1] = self.robot_init_pos[1] + robot_dy_world
        target_pos[2] = self.robot_init_pos[2] + device_dz_world * self.pos_scale
        for i in range(3):
          target_att[i] = self.device_att[i]
        target_att[2] = self.robot_init_att[2] + device_dyaw

        robot_vx_local = device_dx_local * self.vel_scale
        robot_vy_local = device_dy_local * self.vel_scale
        robot_vyaw = device_dyaw * self.ang_vel_scale
        
        if abs(device_dx_local) < self.velmode_pos_thre:
          robot_vx_local = 0.0
        if abs(device_dy_local) < self.velmode_pos_thre:
          robot_vy_local = 0.0
          
        if abs(device_dyaw) < self.velmode_att_thre:
          if self.robot_local_fix_att[2] == None:
            self.robot_local_fix_att[2] = self.robot_att[2]
          target_ang_vel[2] = self.robot_local_fix_att[2]
        else:
          self.robot_local_fix_att[2] = None
          target_ang_vel[2] = self.robot_att[2] + robot_vyaw

        # robot_vx_world, robot_vy_world = rotate_xy_body_to_world(
        #   robot_vx_local, robot_vy_local, self.robot_att[2])
        robot_vx_world, robot_vy_world = rotate_xy_body_to_world(
          robot_vx_local, robot_vy_local, self.robot_yaw_unwrapped)
        target_vel[0] = self.robot_pos[0] + robot_vx_world
        target_vel[1] = self.robot_pos[1] + robot_vy_world

        if device_dx_local >= 0:
          feedback_wrench_x_local = -logarithm(device_dx_local+1, log_base, self.feedback_force_scale)
        else:
          feedback_wrench_x_local = logarithm(-device_dx_local+1, log_base, self.feedback_force_scale)
        if device_dy_local >= 0:
          feedback_wrench_y_local = -logarithm(device_dy_local+1, log_base, self.feedback_force_scale)
        else:
          feedback_wrench_y_local = logarithm(-device_dy_local+1, log_base, self.feedback_force_scale)
        if device_dyaw >= 0:
          feedback_wrench_yaw = -logarithm(device_dyaw+1, log_base, self.feedback_torque_scale)
        else:
          feedback_wrench_yaw = logarithm(-device_dyaw+1, log_base, self.feedback_torque_scale)
        # feedback_wrench[0], feedback_wrench[1] = rotate_xy_body_to_world(
        #   feedback_wrench_x_local, feedback_wrench_y_local, self.device_att[2])
        feedback_wrench[0], feedback_wrench[1] = rotate_xy_body_to_world(
          feedback_wrench_x_local, feedback_wrench_y_local, self.device_yaw_unwrapped)
        feedback_wrench[5] = feedback_wrench_yaw


        """ limitation of z and att for safety """
        if self.robot_pos[2] > 1.2:
          target_pos[2] = 1.2
          target_vel[2] = 0.0
        if self.robot_pos[2] < 0.3:
          target_pos[2] = 0.3
          target_vel[2] = 0.0
        limit_angle = 0.2
        for i in range(2):  # except yaw
          if target_att[i] > limit_angle:
            target_att[i] = limit_angle
          if target_att[i] < -limit_angle:
            target_att[i] = -limit_angle
          if target_ang_vel[i] > limit_angle:
            target_ang_vel[i] = limit_angle
          if target_ang_vel[i] < -limit_angle:
            target_ang_vel[i] = -limit_angle
        force_limit = 15
        torque_limit = 2.0
        for i in range(3):
          if feedback_wrench[i] > force_limit:
            feedback_wrench[i] = force_limit
          if feedback_wrench[i] < -force_limit:
            feedback_wrench[i] = -force_limit
          if feedback_wrench[i+3] > torque_limit:
            feedback_wrench[i+3] = torque_limit
          if feedback_wrench[i+3] < -torque_limit:
            feedback_wrench[i+3] = -torque_limit

        if self.control_mode == "pos":
          self.flight_nav.target_pos_x = target_pos[0]
          self.flight_nav.target_pos_y = target_pos[1]
          self.flight_nav.target_pos_z = target_pos[2]
          self.flight_nav.target_yaw = target_att[2]
          """ for new FlightNav """
          self.flight_nav.target_roll = target_att[0]
          self.flight_nav.target_pitch = target_att[1]
          self.desire_att_nav.roll = target_att[0]
          self.desire_att_nav.pitch = target_att[1]
          self.target_att_nav.vector.x = target_att[0]
          self.target_att_nav.vector.y = target_att[1]
          
        if self.control_mode == "vel":
          self.flight_nav.target_pos_x = target_vel[0]
          self.flight_nav.target_pos_y = target_vel[1]
          # self.flight_nav.target_pos_z = target_vel[2]
          self.flight_nav.target_pos_z = target_pos[2]
          self.flight_nav.target_yaw = target_ang_vel[2]
          # self.flight_nav.target_yaw = target_att[2]
          """ for new FlightNav """
          self.flight_nav.target_roll = target_att[0]
          self.flight_nav.target_pitch = target_att[1]
          self.desire_att_nav.roll = target_att[0]
          self.desire_att_nav.pitch = target_att[1]
          self.target_att_nav.vector.x = target_att[0]
          self.target_att_nav.vector.y = target_att[1]
          
          self.haptics_wrench_msg.wrench.force.x = feedback_wrench[0]
          self.haptics_wrench_msg.wrench.force.y = feedback_wrench[1]
          self.haptics_wrench_msg.wrench.force.z = feedback_wrench[2]
          self.haptics_wrench_msg.wrench.torque.x = feedback_wrench[3]
          self.haptics_wrench_msg.wrench.torque.y = feedback_wrench[4]
          self.haptics_wrench_msg.wrench.torque.z = feedback_wrench[5]

      if self.hovering and not self.landing:
        if not self.wait_flag:
          rospy.sleep(3.0)
          self.wait_flag = True
        self.nav_pub.publish(self.flight_nav)
        # self.att_pub.publish(self.desire_att_nav)
        self.att_pub.publish(self.target_att_nav)
        if self.control_mode == "vel":
          self.feedback_pub.publish(self.haptics_wrench_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("teleop_form_mocap")
  Tracker = teleop_from_mocap()
  Tracker.main()
