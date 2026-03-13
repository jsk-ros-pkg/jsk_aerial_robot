#!/usr/bin/env python

import rospy
import time
import math
import numpy as np
import tf.transformations as tf
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int8
from aerial_robot_msgs.msg import FlightNav
from geometry_msgs.msg import PoseStamped, WrenchStamped
from std_srvs.srv import Empty

def exponential(x, base, k_exp):
  return pow(x,base) * k_exp

def logarithm(x, base, k_log):
  return math.log(x,base) * k_log

class force_feedback_from_robot():

  def __init__(self):

    self.robot_name = rospy.get_param("~robot_name", "gimbalrotor")
    self.convert_method = rospy.get_param("~convert_method", "prop") # "prop" or "exp" or "log"
    self.frame = rospy.get_param("~frame", "local") # "local" or "world"
    self.feedback_from_ang = rospy.get_param("~feedback_from_ang", "False")

    # self.haptics_switch_pub = rospy.Publisher('/twin_hammer/haptics_switch', Int8, queue_size=1)
    self.wrench_pub = rospy.Publisher('/twin_hammer/feedback_from_robot', WrenchStamped, queue_size=1)
    self.robot_mocap_sub = rospy.Subscriber('/'+self.robot_name+'/mocap/pose', PoseStamped, self.robot_mocap_cb)
    self.device_mocap_sub = rospy.Subscriber('/twin_hammer/mocap/pose', PoseStamped, self.device_mocap_cb)
    # self.robot_wrench_sub = rospy.Subscriber('/cfs/data', WrenchStamped, self.robot_wrench_cb)
    self.robot_wrench_sub = rospy.Subscriber('/filtered_ftsensor', WrenchStamped, self.robot_wrench_cb)

    self.haptics_wrench_msg = WrenchStamped()
    self.robot_wrench = []
    self.filterd_robot_wrench_local = [0.0,0.0,0.0,0.0,0.0,0.0]
    self.robot_att = [0.0,0.0,0.0]
    self.device_att = [0.0,0.0,0.0]
    self.k_att_diff = 1.0
    self.Ad_R_robot = np.block([
      [np.identity(3), np.zeros((3,3))],
      [np.zeros((3,3)), np.identity(3)]
    ])
    self.Ad_R_inv_device = np.block([
      [np.identity(3), np.zeros((3,3))],
      [np.zeros((3,3)), np.identity(3)]
    ])
    self.moment_arm = np.array([-(0.044 + 0.025), 0, 0])
    # for gimbalrotor
    self.k_p = 2.0
    self.exp_base = 1.45
    self.log_base = 1.45
    self.k_exp = 0.4
    self.k_log = 1.0
    # for me6
    self.k_p = 0.5
    self.exp_base = 1.45
    self.log_base = 1.45
    self.k_exp = 0.2
    self.k_log = 0.8
    """ belows are dependent valuables """
    self.range_log = math.e
    self.a_log = self.k_log / (math.e * math.log(self.log_base))

    rospy.wait_for_service("/cfs_sensor_calib")
    calib_srv = rospy.ServiceProxy("/cfs_sensor_calib", Empty)
    try:
      res_calib = calib_srv()
    except rospy.ServiceException as e:
      rospy.logerr(f"force sensor calibration service call failed: {e}")

  def robot_mocap_cb(self,msg):
    # self.robot_att = [msg.pose.orientation.roll, msg.pose.orientation.pitch, msg.pose.orientation.yaw] 
    q = [msg.pose.orientation.x,msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rot = R.from_quat(q)
    self.robot_att = rot.as_euler('xyz')
    R_mat = rot.as_matrix()
    self.Ad_R_robot = np.block([
    [R_mat, np.zeros((3, 3))],
    [np.zeros((3, 3)), R_mat]
    ])

  def device_mocap_cb(self,msg):
    # self.device_att = [msg.pose.orientation.roll, msg.pose.orientation.pitch, msg.pose.orientation.yaw] 
    q = [msg.pose.orientation.x,msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    rot = R.from_quat(q)
    self.device_att = rot.as_euler('xyz')
    R_mat = rot.as_matrix()
    self.Ad_R_inv_device = np.block([
    [R_mat.T, np.zeros((3, 3))],
    [np.zeros((3, 3)), R_mat.T]
    ])

  def robot_wrench_cb(self,msg):
    """ below mapping is depend on the direction for installing force sensor """
    force_x = msg.wrench.force.z
    force_y = msg.wrench.force.x
    force_z = msg.wrench.force.y
    torque_x = msg.wrench.torque.z
    torque_y = msg.wrench.torque.x
    torque_z = msg.wrench.torque.y

    force_np = np.array([force_x,force_y,force_z])
    torque_from_force = np.cross(force_np,self.moment_arm)
    wrench_local = [force_x, force_y, force_z, torque_x, torque_y, torque_z]
    # wrench_local = [force_x, force_y, force_z, torque_x+torque_from_force[0], torque_y+torque_from_force[1], torque_z+torque_from_force[2]]
    delay_param = 0.05
    for i in range (len(wrench_local)):
      self.filterd_robot_wrench_local[i] = (1-delay_param) * self.filterd_robot_wrench_local[i] + delay_param * wrench_local[i]
    wrench_world = np.dot(self.Ad_R_robot,self.filterd_robot_wrench_local)
    if self.frame == "local":
      self.robot_wrench = self.filterd_robot_wrench_local
    if self.frame == "world":
      self.robot_wrench = wrench_world

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      haptics_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

      for i in range(len(self.robot_wrench)):
        wrench_i = self.robot_wrench[i]

        """ propotional conversion """
        if self.convert_method == "prop":
          haptics_wrench[i] = wrench_i * self.k_p

        """ exponential conversion """
        if self.convert_method == "exp":
          if wrench_i>=0.0:
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
          if wrench_i>=0:
            haptics_wrench[i] = logarithm(wrench_i+1, self.log_base, self.k_log)
          if wrench_i<-self.range_log:
            haptics_wrench[i] = -logarithm(-(wrench_i-1), self.log_base, self.k_log)

      """ force feedback from ang diff """
      # if self.feedback_from_ang:
      #   att_diff = self.robot_att - self.device_att
      #   for i in range(len(att_diff)):
      #     wrench_from_pos_diff = logarithm(att_diff[i],self.log_base,self.k_att_diff)
      #     haptics_wrench[i] += wrench_from_pos_diff

      # if self.frame == "world":
      #   haptics_wrench = np.dot(self.Ad_R_inv_device,haptics_wrench)

      force_limit = 15
      torque_limit = 1.5
      for i in range(3):
        if haptics_wrench[i] > force_limit:
          haptics_wrench[i] = force_limit
        if haptics_wrench[i] < -force_limit:
          haptics_wrench[i] = -force_limit
        if haptics_wrench[i+3] > torque_limit:
          haptics_wrench[i+3] = torque_limit
        if haptics_wrench[i+3] < -torque_limit:
          haptics_wrench[i+3] = -torque_limit

      self.haptics_wrench_msg.wrench.force.x = haptics_wrench[0]
      self.haptics_wrench_msg.wrench.force.y = haptics_wrench[1]
      self.haptics_wrench_msg.wrench.force.z = haptics_wrench[2]
      self.haptics_wrench_msg.wrench.torque.x = haptics_wrench[3]
      self.haptics_wrench_msg.wrench.torque.y = haptics_wrench[4]
      self.haptics_wrench_msg.wrench.torque.z = haptics_wrench[5]
      self.wrench_pub.publish(self.haptics_wrench_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("force_feedback_from_robot")
  Tracker = force_feedback_from_robot()
  Tracker.main()
