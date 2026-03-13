#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty
# from spinal.msg import ServoControlCmd

class init_gimbal_angle():
  def __init__(self):
    self.init_angles = {}

    self.pub = rospy.Publisher('/twin_hammer/gimbals_ctrl', JointState, queue_size=1)
    self.start_sub = rospy.Subscriber('/twin_hammer/start', Empty, self.start_cb)
    self.takeoff_sub = rospy.Subscriber('/twin_hammer/takeoff', Empty, self.takeoff_cb)
    self.land_sub = rospy.Subscriber('/twin_hammer/land', Empty, self.land_cb)
    self.init_flag = False
  
  def start_cb(self,msg):
    self.init_flag = True

  def takeoff_cb(self,msg):
    self.init_flag = False

  def land_cb(self,msg):
    self.init_flag = True

  def main(self):
    while not rospy.is_shutdown():
      gimbal_msg = JointState()
      gimbal_msg.position = [0.0, 0.0, 0.0, 0.0]
      if self.init_flag:
        self.pub.publish(gimbal_msg)


if __name__=='__main__':
  rospy.init_node('init_gimbal_angles')
  Tracker = init_gimbal_angle()
  Tracker.main()
