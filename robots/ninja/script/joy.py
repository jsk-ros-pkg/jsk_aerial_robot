#!/usr/bin/env python
from std_msgs.msg import Empty
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JoyFeedbackArray

import rospy
import numpy as np
import tf
import rosgraph
import sys, select, termios, tty, time
import math


class BeetleJoy():
    def __init__(self):
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()
        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        robot_names = [topic.split('/teleop')[0] for topic in teleop_topics]
        self.joy_pubs = []
        self.joy_fb_pubs = []
        for name in robot_names:
            ns = name
            self.joy_pubs.append(rospy.Publisher(ns + '/joy', Joy, queue_size=1))
            self.joy_fb_pubs.append(rospy.Publisher(ns + '/joy/set_feedback', JoyFeedbackArray, queue_size=1))

        self.joy_sub = rospy.Subscriber("/beetle1/joy",Joy, self.joyCb)
        self.joy_fb_sub = rospy.Subscriber("/beetle1/joy/set_feedback",JoyFeedbackArray, self.joyFbCb)
        self.nav_rate = rospy.Rate(40)
        
        self.joy = Joy()
        self.joy_fb = JoyFeedbackArray()
        
        time.sleep(0.5)

    def joyCb(self, msg):
        self.joy = msg

    def joyFbCb(self, msg):
        self.joy_fb = msg

    def main(self):
        while not rospy.is_shutdown():
            for joy_pub in self.joy_pubs:
                joy_pub.publish(self.joy)
            for joy_fb_pub in self.joy_fb_pubs:
                joy_fb_pub.publish(self.joy_fb)
        self.nav_rate.sleep()

if __name__ == "__main__":

  rospy.init_node("beetle_joy")

  joy = BeetleJoy()
  joy.main()



