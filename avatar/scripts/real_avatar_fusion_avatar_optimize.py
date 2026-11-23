#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
import copy
from sensor_msgs.msg import JointState
from std_msgs.msg import *

class fusion_avatar_optimize():
  
  def __init__(self):
    self.fc_rp_min_thre = rospy.get_param("/hydrus/fc_rp_min_thre", 1.8)
    self.debug = rospy.get_param("~debug", False)
    self.robot_name = rospy.get_param("~robot_ns", "hydrus")

    topic_name = '/'+self.robot_name+'/joints_ctrl'
    if self.debug:
      topic_name = '/'+self.robot_name+'/joint_states'

    self.avatar_pos_sub = rospy.Subscriber('/'+self.robot_name+'/avatar_pos', JointState, self.avatar_pos_Cb)
    self.optimize_pos_sub = rospy.Subscriber('/'+self.robot_name+'/opt_joint_angles', JointState, self.optimize_pos_Cb)
    self.max_fc_rp_min_sub = rospy.Subscriber('/'+self.robot_name+'/opt_max_fcrpmin', Float64, self.max_fc_rp_min_Cb)
    self.fc_rp_min_sub = rospy.Subscriber('/'+self.robot_name+'/fc_rp_min', Float64, self.fc_rp_min_Cb)
    self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_stateCb)
    self.target_pos_pub = rospy.Publisher(topic_name, JointState, queue_size=10)
    self.fusion_torque_balance_pub = rospy.Publisher('/'+self.robot_name+'/fusion_torque_balance', Float64, queue_size=10)
    self.fusion_pos_balance_pub = rospy.Publisher('/'+self.robot_name+'/fusion_pos_balance', Float64, queue_size=10)

    self.avatar_joint_angles = JointState()
    self.opt_joint_angles = JointState()
    self.target_joint_angles = JointState()
    self.target_joint_pos_list = []
    self.fusion_torque_balance = Float64()
    self.fusion_pos_balance = Float64()
    self.opt_max_fcrpmin = 5.0
    self.fc_rp_min = 0.0
    self.angle_limit = 1.57
    self.Hovering = False
    self.pos_balance = 1.0
    self.torque_balance = 1.0

  def avatar_pos_Cb(self,msg):
    self.avatar_joint_angles = msg
    self.target_joint_angles = copy.deepcopy(self.avatar_joint_angles)
    self.target_joint_pos_list = list(self.target_joint_angles.position)

  def optimize_pos_Cb(self,msg):
    self.opt_joint_angles = msg
    l = list(self.opt_joint_angles.position)
    if self.robot_name == 'hydrus_xi':
      del self.opt_joint_angles.name[0::2]
      del l[0::2]
      self.opt_joint_angles.position = tuple(l)

  def max_fc_rp_min_Cb(self,msg):
    self.opt_max_fcrpmin = msg.data

  def fc_rp_min_Cb(self,msg):
    self.fc_rp_min = msg.data

  def flight_stateCb(self,msg):
    flight_state = msg.data
    if flight_state == 5:
        self.Hovering = True 


  def balance_decision(self, opt_max_fcrpmin, fc_rp_min, fc_rp_min_thre):
    rp_range = opt_max_fcrpmin - fc_rp_min_thre
    rp_diff = fc_rp_min - fc_rp_min_thre
    # linear
    # A = rp_diff/rp_range
    # sin
    A = math.sin((math.pi/2)*(rp_diff/rp_range))
    if A >1.0:
      A = 1.0
    if A < 0.0:
      A = 0.0
    self.pos_balance = A
    self.torque_balance = A-0.1

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():  
      if self.avatar_joint_angles.position != []:
        if self.Hovering==True or self.debug==True:
          self.balance_decision(opt_max_fcrpmin=self.opt_max_fcrpmin, fc_rp_min=self.fc_rp_min,  fc_rp_min_thre=self.fc_rp_min_thre)
        else:
          self.pos_balance = 0.4
          self.torque_balance = 0.4
        self.fusion_torque_balance.data = self.torque_balance
        self.fusion_pos_balance.data = self.pos_balance

        # fusion angles
        for i in range(len(self.opt_joint_angles.position)):
          avatar_pos = self.avatar_joint_angles.position[i]
          opt_pos = self.opt_joint_angles.position[i]
          target_pos = avatar_pos*self.pos_balance + opt_pos*(1-self.pos_balance)
          if target_pos > self.angle_limit:
            target_pos = self.angle_limit
          if target_pos < -self.angle_limit:
            target_pos = -self.angle_limit
          self.target_joint_pos_list[i] = target_pos
        self.target_joint_angles.position = tuple(self.target_joint_pos_list)
        
        if self.Hovering==True or self.debug==True:
          self.target_pos_pub.publish(self.target_joint_angles)
          self.fusion_torque_balance_pub.publish(self.fusion_torque_balance)
          self.fusion_pos_balance_pub.publish(self.fusion_pos_balance)
        
      r.sleep()

if __name__ == "__main__":
  rospy.init_node("fusion_avatar_optimize")
  Tracker = fusion_avatar_optimize()
  Tracker.main()  

