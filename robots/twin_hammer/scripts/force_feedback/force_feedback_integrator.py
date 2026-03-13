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

class force_feedback_integrator():

  def __init__(self):
    self.haptics_wrench_pub = rospy.Publisher('/twin_hammer/haptics_wrench', WrenchStamped, queue_size=1)
    self.robot_feedback_sub = rospy.Subscriber('/twin_hammer/feedback_from_robot', WrenchStamped, self.robot_feedback_cb)
    self.device_feedback_sub = rospy.Subscriber('/twin_hammer/feedback_from_device', WrenchStamped, self.device_feedback_cb)
    self.obstacle_feedback_sub = rospy.Subscriber('/twin_hammer/feedback_from_obstacle', WrenchStamped, self.obstacle_feedback_cb)

    self.haptics_wrench_msg = WrenchStamped()
    self.robot_feedback_wrench = [0.0]*6
    self.device_feedback_wrench = [0.0]*6
    self.obstacle_feedback_wrench = [0.0]*6
    self.robot_weight = 1.0
    self.device_weight = 1.0
    self.obstacle_weight = 1.0
    self.force_limit = 15.0
    self.torque_limit = 3.0
    # time.sleep(0.5)

  def robot_feedback_cb(self,msg):
    self.robot_feedback_wrench[0] = msg.wrench.force.x
    self.robot_feedback_wrench[1] = msg.wrench.force.y
    self.robot_feedback_wrench[2] = msg.wrench.force.z
    self.robot_feedback_wrench[3] = msg.wrench.torque.x
    self.robot_feedback_wrench[4] = msg.wrench.torque.y
    self.robot_feedback_wrench[5] = msg.wrench.torque.z

  def device_feedback_cb(self,msg):
    self.device_feedback_wrench[0] = msg.wrench.force.x
    self.device_feedback_wrench[1] = msg.wrench.force.y
    self.device_feedback_wrench[2] = msg.wrench.force.z
    self.device_feedback_wrench[3] = msg.wrench.torque.x
    self.device_feedback_wrench[4] = msg.wrench.torque.y
    self.device_feedback_wrench[5] = msg.wrench.torque.z

  def obstacle_feedback_cb(self,msg):
    self.obstacle_feedback_wrench[0] = msg.wrench.force.x
    self.obstacle_feedback_wrench[1] = msg.wrench.force.y
    self.obstacle_feedback_wrench[2] = msg.wrench.force.z
    self.obstacle_feedback_wrench[3] = msg.wrench.torque.x
    self.obstacle_feedback_wrench[4] = msg.wrench.torque.y
    self.obstacle_feedback_wrench[5] = msg.wrench.torque.z

  def main(self):
    r = rospy.Rate(40)
    while not rospy.is_shutdown():
      haptics_wrench = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

      for i in range(6):
        haptics_wrench[i] = self.robot_feedback_wrench[i]*self.robot_weight + self.device_feedback_wrench[i]*self.device_weight + self.obstacle_feedback_wrench[i]*self.obstacle_weight

      for i in range(3):
        if haptics_wrench[i] > self.force_limit:
          haptics_wrench[i] = self.force_limit
        if haptics_wrench[i] < -self.force_limit:
          haptics_wrench[i] = -self.force_limit
        if haptics_wrench[i+3] > self.torque_limit:
          haptics_wrench[i+3] = self.torque_limit
        if haptics_wrench[i+3] < -self.torque_limit:
          haptics_wrench[i+3] = -self.torque_limit

      self.haptics_wrench_msg.wrench.force.x = haptics_wrench[0]
      self.haptics_wrench_msg.wrench.force.y = haptics_wrench[1]
      self.haptics_wrench_msg.wrench.force.z = haptics_wrench[2]
      self.haptics_wrench_msg.wrench.torque.x = haptics_wrench[3]
      self.haptics_wrench_msg.wrench.torque.y = haptics_wrench[4]
      self.haptics_wrench_msg.wrench.torque.z = haptics_wrench[5]
      self.haptics_wrench_pub.publish(self.haptics_wrench_msg)

      r.sleep()

if __name__ == "__main__":
  rospy.init_node("force_feedback_integrator")
  Tracker = force_feedback_integrator()
  Tracker.main()
