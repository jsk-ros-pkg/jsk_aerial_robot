#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

class dummy_mocap_node():

  def __init__(self):
    self.mocap_pub = rospy.Publisher('/twin_hammer/mocap/pose', PoseStamped, queue_size=1)

  def main(self):
    while not rospy.is_shutdown():
      mocap_msg = PoseStamped()
      mocap_msg.header.stamp = rospy.Time.now()
      mocap_msg.pose.position.x = 0.0
      mocap_msg.pose.position.y = 0.0
      mocap_msg.pose.position.z = 0.0
      mocap_msg.pose.orientation.x = 0.0
      mocap_msg.pose.orientation.y = 0.0
      mocap_msg.pose.orientation.z = 0.0
      mocap_msg.pose.orientation.w = 1.0
      self.mocap_pub.publish(mocap_msg)

if __name__ == "__main__":
  rospy.init_node("dummy_mocap_node")
  Tracker = dummy_mocap_node()
  Tracker.main()