#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav, PoseControlPid

class CircTrajFollow():
  def __init__(self):
    self.period = rospy.get_param("~period", 40.0)
    self.radius = rospy.get_param("~radius", 1.0)
    self.init_theta = rospy.get_param("~init_theta", 0.0)
    self.yaw = rospy.get_param("~yaw", True)
    self.loop = rospy.get_param("~loop", False)

    self.nav_pub = rospy.Publisher("uav/nav", FlightNav, queue_size=1)
    self.control_sub = rospy.Subscriber("debug/pose/pid", PoseControlPid, self.controlCb)

    self.center_pos_x = None
    self.center_pos_y = None

    self.omega = 2 * math.pi / self.period
    self.velocity = self.omega * self.radius

    self.nav_rate = rospy.get_param("~nav_rate", 20.0) # hz
    self.nav_rate = 1 / self.nav_rate

    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    if self.yaw:
      self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE

    signal.signal(signal.SIGINT, self.stopRequest)

    time.sleep(0.5)

  def controlCb(self, msg):

    self.initial_target_yaw = msg.yaw.target_p

    self.center_pos_x = msg.x.target_p - math.cos(self.init_theta) * self.radius
    self.center_pos_y = msg.y.target_p - math.sin(self.init_theta) * self.radius

    rospy.loginfo("the center position is [%f, %f]", self.center_pos_x, self.center_pos_y)

    self.control_sub.unregister()

  def stopRequest(self, signal, frame):
    rospy.loginfo("stop following")
    self.flight_nav.target_vel_x = 0
    self.flight_nav.target_vel_y = 0
    self.flight_nav.target_omega_z = 0
    self.nav_pub.publish(self.flight_nav)

    sys.exit(0)

  def main(self):

    cnt = 0

    while not rospy.is_shutdown():

      if self.center_pos_y is None:
        rospy.loginfo_throttle(1.0, "not yet receive the controller message")
        time.sleep(self.nav_rate)
        continue

      theta = self.init_theta + cnt * self.nav_rate * self.omega
      self.target_pos_x = self.center_pos_x + math.cos(theta) * self.radius
      self.target_pos_y = self.center_pos_y + math.sin(theta) * self.radius
      self.flight_nav.target_pos_x = self.target_pos_x
      self.flight_nav.target_pos_y = self.target_pos_y
      self.flight_nav.target_vel_x = -math.sin(theta) * self.velocity
      self.flight_nav.target_vel_y = math.cos(theta) * self.velocity

      if self.yaw:
        self.flight_nav.target_yaw = self.initial_target_yaw + cnt * self.nav_rate * self.omega
        self.flight_nav.target_omega_z = self.omega

      self.nav_pub.publish(self.flight_nav)

      cnt += 1

      if cnt == self.period // self.nav_rate:
        if self.loop:
          cnt = 0
        else:

          time.sleep(0.1)
          self.flight_nav.target_vel_x = 0
          self.flight_nav.target_vel_y = 0
          self.flight_nav.target_omega_z = 0
          self.nav_pub.publish(self.flight_nav)

          break # only one loop

      time.sleep(self.nav_rate)


if __name__ == "__main__":

  rospy.init_node("circle_trajectory_follow")

  Tracker = CircTrajFollow()
  Tracker.main()



