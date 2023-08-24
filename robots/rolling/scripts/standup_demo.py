#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tft
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from spinal.msg import DesireCoord
from std_msgs.msg import Empty

class RollingDemo:
    def __init__(self):
        self.cog_odom_sub_ = rospy.Subscriber("/rolling/uav/cog/odom", Odometry, self.cogOdomcallback)
        self.cog_odom = Odometry()
        self.cog_euler = np.zeros(3)
        self.baselink_odom_sub = rospy.Subscriber("/rolling/uav/baselink/odom", Odometry, self.baselinkOdomCallback)
        self.baselink_odom = Odometry()
        self.baselink_euler = np.zeros(3)
        self.desire_coord_pub = rospy.Publisher("/rolling/desire_coordinate", DesireCoord, queue_size=1)
        self.flight_nav_pub = rospy.Publisher("/rolling/uav/nav", FlightNav, queue_size=1)
        self.start_flag = False
        self.start_sub = rospy.Subscriber("rolling/start_demo", Empty, self.startDemoCallback)
        self.radius = rospy.get_param("/rolling/circle_radius")
        self.stable_cnt = 0
        self.timer_freq = 20
        self.target_phi = 0
        self.initial_cog_odom = None
        self.initial_baselink_euler = None

        self.timer = rospy.Timer(rospy.Duration(1 / self.timer_freq), self.timerCallback)

    def cogOdomcallback(self, msg):
        self.cog_odom = msg
        self.cog_euler = tft.euler_from_quaternion([self.cog_odom.pose.pose.orientation.x, self.cog_odom.pose.pose.orientation.y, self.cog_odom.pose.pose.orientation.z, self.cog_odom.pose.pose.orientation.w])

    def baselinkOdomCallback(self, msg):
        self.baselink_odom = msg
        self.baselink_euler = tft.euler_from_quaternion([self.baselink_odom.pose.pose.orientation.x, self.baselink_odom.pose.pose.orientation.y, self.baselink_odom.pose.pose.orientation.z, self.baselink_odom.pose.pose.orientation.w])

    def startDemoCallback(self, msg):
        self.start_flag = True
        self.initial_cog_odom = self.cog_odom
        self.initial_baselink_euler = self.baselink_euler
        rospy.logwarn("initial position of cog = (%lf, %lf, %lf)", self.initial_cog_odom.pose.pose.position.x, self.initial_cog_odom.pose.pose.position.y, self.initial_cog_odom.pose.pose.position.z)
        rospy.logwarn("initial euler angle of baselink = (%lf, %lf, %lf)", self.initial_baselink_euler[0], self.initial_baselink_euler[1], self.initial_baselink_euler[2])

    def timerCallback(self, event):
        if not self.start_flag:
            return
        rospy.logwarn_once("start demo")
        rospy.logwarn_once("radius = %lf", self.radius)

        cog_z = self.cog_odom.pose.pose.position.z
        if abs(cog_z / self.radius) < 1.0:
            self.target_phi = np.arcsin(cog_z / self.radius)
        else:
            rospy.logwarn_once("not publish because of arcsin error")

        self.desire_coord = DesireCoord()
        self.desire_coord.roll = self.target_phi
        self.desire_coord_pub.publish(self.desire_coord)

        flight_nav_msg = FlightNav()
        flight_nav_msg.target = FlightNav.COG
        flight_nav_msg.pos_xy_nav_mode = 2
        flight_nav_msg.target_pos_x = self.initial_cog_odom.pose.pose.position.x - self.radius * (1 - np.cos(self.target_phi)) * np.cos(self.initial_baselink_euler[2] + np.pi / 2.0)
        flight_nav_msg.target_pos_y = self.initial_cog_odom.pose.pose.position.y - self.radius * (1 - np.cos(self.target_phi)) * np.sin(self.initial_baselink_euler[2] + np.pi / 2.0)
        self.flight_nav_pub.publish(flight_nav_msg)

if __name__ == '__main__':
    rospy.init_node("rolling_demo_node")

    RollingDemo()

    rospy.spin()
