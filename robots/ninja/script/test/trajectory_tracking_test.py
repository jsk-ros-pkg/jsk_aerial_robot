#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
import numpy as np
from aerial_robot_msgs.msg import FlightNav, PoseControlPid

class LemniTrajFollow():
  def __init__(self):
    self.period = 80.0
    self.radius = 1.0
    self.init_theta = 0.0

    self.nav_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=1)

    self.center_pos_x = 0.0
    self.center_pos_y = 0.0

    self.omega = math.pi / self.period
    self.nav_rate = 1.0/ 40.0

    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE

    time.sleep(0.5)

  def lemniscate_polar(self, theta, a):
    r = np.sqrt(self.radius**2 * np.cos(2 * theta))
    return r

  def polar_to_cartesian(self, r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y


  def main(self):
    t = 0.0
    while not rospy.is_shutdown():
      if t > self.period / self.nav_rate:
        return
      theta_cand = self.init_theta + t * self.nav_rate * self.omega
      theta = 0.0
      if 0 < theta_cand and theta_cand < math.pi/ 4.0:
        theta = theta_cand
      elif math.pi/ 4.0 <= theta_cand  <  math.pi / 2.0:
        theta = 3 * math.pi / 2 - theta_cand
      elif math.pi/ 2.0 <= theta_cand   < 3 * math.pi/ 4.0:
        theta = 3 * math.pi / 2.0  - theta_cand
      elif 3 * math.pi/ 4.0 <= theta_cand < math.pi:
        theta = math.pi  + theta_cand
      elif theta_cand >= math.pi:
        return
      r = self.lemniscate_polar(theta, self.radius)
      x, y = self.polar_to_cartesian(r, theta)

      if not (math.isnan(x) or math.isnan(y)):
        self.target_pos_x = self.center_pos_x + x
        self.target_pos_y = self.center_pos_y + y
      rospy.loginfo(theta_cand)
      rospy.loginfo(self.target_pos_x)
      self.flight_nav.target_pos_x = self.target_pos_x
      self.flight_nav.target_pos_y = self.target_pos_y
      self.nav_pub.publish(self.flight_nav)
      t = t+1
      time.sleep(self.nav_rate)


if __name__ == "__main__":

  rospy.init_node("lemni_trajectory_follow")

  Tracker = LemniTrajFollow()
  Tracker.main()



