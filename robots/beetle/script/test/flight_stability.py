#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from spinal.msg import DesireCoord

class RotTrack():
  def __init__(self):
    self.rate = rospy.Rate(10)
    self.rot_pub = rospy.Publisher('assembled/final_target_baselink_rot', DesireCoord, queue_size=1)
    self.target_rot = DesireCoord()
    self.t = 0
  def main(self):
    while not rospy.is_shutdown():
      self.t = self.t + 0.1
      self.target_rot.roll = 0
      self.target_rot.pitch = math.sin(self.t)
      self.rot_pub.publish(self.target_rot)
      self.rate.sleep()


if __name__ == "__main__":

  rospy.init_node("rot_track")
  rot_track = RotTrack()
  rot_track.main()



