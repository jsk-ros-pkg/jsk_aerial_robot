#!/usr/bin/env python
from std_msgs.msg import Empty
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped

import rospy
import numpy as np
import tf
import rosgraph
import sys, select, termios, tty, time
import math


class AssembledNav():
    def __init__(self):
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        robot_names = [topic.split('/teleop')[0] for topic in teleop_topics]
        self.nav_pubs = []
        self.rot_pubs = []
        for name in robot_names:
            ns = name
            self.nav_pubs.append(rospy.Publisher(ns + '/uav/nav', FlightNav, queue_size=1))
            self.rot_pubs.append(rospy.Publisher(ns + '/final_target_baselink_rot', DesireCoord, queue_size=1))

        self.nav_sub = rospy.Subscriber("assembled/uav/nav",FlightNav, self.navCb)
        self.rot_sub = rospy.Subscriber("assembled/final_target_baselink_rot",DesireCoord, self.rotCb)
        self.nav_rate = rospy.Rate(40)
        
        self.flight_nav = FlightNav()
        self.flight_nav.target = FlightNav.BASELINK
        
        self.target_rot = DesireCoord()
        time.sleep(0.5)

    def navCb(self, msg):
        self.flight_nav.pos_xy_nav_mode = msg.pos_xy_nav_mode
        self.flight_nav.pos_z_nav_mode = msg.pos_z_nav_mode
        self.flight_nav.yaw_nav_mode = msg.yaw_nav_mode
        self.flight_nav.target_pos_x = msg.target_pos_x
        self.flight_nav.target_pos_y = msg.target_pos_y
        self.flight_nav.target_vel_x = msg.target_vel_x
        self.flight_nav.target_vel_y = msg.target_vel_y
        self.flight_nav.target_pos_z = msg.target_pos_z
        self.flight_nav.target_omega_z = msg.target_omega_z
        self.flight_nav.target_yaw = msg.target_yaw
        for nav_pub in self.nav_pubs:
            nav_pub.publish(self.flight_nav)
    def rotCb(self, msg):
        self.target_rot.roll = msg.roll
        self.target_rot.pitch = msg.pitch
        for rot_pub in self.rot_pubs:
            rot_pub.publish(self.target_rot)
    def main(self):
        rospy.spin()


if __name__ == "__main__":

  rospy.init_node("assembled_nav")

  nav = AssembledNav()
  nav.main()



