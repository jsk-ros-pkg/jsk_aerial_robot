#!/usr/bin/env python

import rospy
import numpy as np
import tf.transformations as tft
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry
from spinal.msg import DesireCoord
from std_msgs.msg import Empty, Bool, Float32, Int16

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
        self.z_i_control_flag_pub = rospy.Publisher("/rolling/z_i_control_flag", Bool, queue_size=1)
        self.z_i_term_pub = rospy.Publisher("/rolling/z_i_term", Float32, queue_size=1)
        self.control_mode_pub = rospy.Publisher("/rolling/control_mode", Int16, queue_size=1)
        self.stay_current_pose_pub = rospy.Publisher("/rolling/stay_current_position", Empty, queue_size=1)

        self.start_flag = False
        self.rolling_demo_flag = False
        self.start_sub = rospy.Subscriber("rolling/standup_demo", Empty, self.startDemoCallback)
        self.rolling_demo_sub = rospy.Subscriber("rolling/rolling_demo", Empty, self.rollingDemoCallback)

        self.radius = rospy.get_param("/rolling/circle_radius")
        self.gravity = 9.81
        self.stable_cnt = 0
        self.timer_freq = 20
        self.target_phi = 0
        self.target_pos_z_ratio = 0.6
        self.ground_compensation_ratio = 0.6
        self.initial_cog_odom = None
        self.initial_baselink_euler = None

        self.emargency_roll_thresh = 0.4

        self.pitch_step = 0.2
        self.step_length_in_xy = self.pitch_step * self.radius

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

    def rollingDemoCallback(self, msg):
        rospy.logwarn("rolling demo callback")

        # need to set i term of z before disable err_i update
        z_i_term_msg = Float32()
        z_i_term_msg.data = self.gravity * self.ground_compensation_ratio
        self.z_i_term_pub.publish(z_i_term_msg)
        rospy.logwarn("set z i term to %lf", z_i_term_msg.data)

        self.initial_cog_odom = self.cog_odom
        self.initial_baselink_euler = self.baselink_euler

        z_i_control_msg = Bool()
        z_i_control_msg.data = False
        self.z_i_control_flag_pub.publish(z_i_control_msg)
        rospy.logwarn("disable z i control")

        stay_current_pose_msg = Empty()
        self.stay_current_pose_pub.publish(stay_current_pose_msg)
        rospy.logwarn("set target xy and yaw to current state")

        self.rolling_demo_flag = True

    def timerCallback(self, event):
        if not self.start_flag:
            return
        rospy.logwarn_once("start demo")
        rospy.logwarn_once("radius = %lf", self.radius)

        if self.start_flag and not self.rolling_demo_flag:
            cog_z = self.cog_odom.pose.pose.position.z
            if abs(cog_z / self.radius) < 1.0:
                self.target_phi = np.arcsin(cog_z / self.radius)
            else:
                rospy.logwarn_once("not publish because of arcsin error")

            desire_coord = DesireCoord()
            desire_coord.roll = self.target_phi
            self.desire_coord_pub.publish(desire_coord)

            flight_nav_msg = FlightNav()
            flight_nav_msg.target = FlightNav.COG
            flight_nav_msg.pos_xy_nav_mode = FlightNav.POS_MODE
            flight_nav_msg.target_pos_x = self.initial_cog_odom.pose.pose.position.x - self.radius * (1 - np.cos(self.target_phi)) * np.cos(self.initial_baselink_euler[2] + np.pi / 2.0)
            flight_nav_msg.target_pos_y = self.initial_cog_odom.pose.pose.position.y - self.radius * (1 - np.cos(self.target_phi)) * np.sin(self.initial_baselink_euler[2] + np.pi / 2.0)
            self.flight_nav_pub.publish(flight_nav_msg)

        if self.start_flag and self.rolling_demo_flag:
            flight_nav_msg = FlightNav()
            flight_nav_msg.target = FlightNav.COG
            flight_nav_msg.pos_z_nav_mode = 2

            safety = None

            if np.abs(self.baselink_euler[0] - 1.57) > self.emargency_roll_thresh:
                safety = False
                flight_nav_msg.target_pos_z = self.radius
                z_i_control_msg = Bool()
                z_i_control_msg.data = True
                z_i_term_msg = Float32()
                z_i_term_msg.data = self.gravity
                rospy.logerr("roll of baselink is %lf. set target z to %lf", self.baselink_euler[0], flight_nav_msg.target_pos_z)
            else:
                safety = True
                flight_nav_msg.target_pos_z = self.radius * self.target_pos_z_ratio
                z_i_control_msg = Bool()
                z_i_control_msg.data = False
                z_i_term_msg = Float32()
                z_i_term_msg.data = self.gravity * self.ground_compensation_ratio
                rospy.logwarn("roll of baselink is %lf. set target z to %lf", self.baselink_euler[0], flight_nav_msg.target_pos_z)

            self.z_i_term_pub.publish(z_i_term_msg)
            self.z_i_control_flag_pub.publish(z_i_control_msg)


            # http://blog.livedoor.jp/ddrerizayoi/archives/56803925.html
            # y = \tan\psi(x - x_init) + y_init
            # x \sin\psi - y \cos\psi - x_init \sin\psi + y_init \cos\psi = 0
            psi = self.initial_baselink_euler[2]
            initial_x = self.initial_cog_odom.pose.pose.position.x
            initial_y = self.initial_cog_odom.pose.pose.position.y
            initial_theta = self.initial_baselink_euler[1]

            cur_x = self.cog_odom.pose.pose.position.x
            cur_y = self.cog_odom.pose.pose.position.y

            a = np.sin(psi)
            b = - np.cos(psi)
            c = -initial_x * np.sin(psi) + initial_y * np.cos(psi)

            symmetry_x = cur_x - 2 * (a * cur_x + b * cur_y + c) * np.sin(psi)
            symmetry_y = cur_y - 2 * (a * cur_x + b * cur_y + c) * (- np.cos(psi))

            center_x = (cur_x + symmetry_x) / 2.0
            center_y = (cur_y + symmetry_y) / 2.0

            if safety:
                flight_nav_msg.pos_xy_nav_mode = 2
                flight_nav_msg.target_pos_x = center_x + self.step_length_in_xy * np.cos(psi)
                flight_nav_msg.target_pos_y = center_y + self.step_length_in_xy * np.sin(psi)
            else:
                flight_nav_msg.pos_xy_nav_mode = 2
                flight_nav_msg.target_pos_x = center_x
                flight_nav_msg.target_pos_y = center_y

            self.flight_nav_pub.publish(flight_nav_msg)

            d_x = flight_nav_msg.target_pos_x - initial_x
            d_y = flight_nav_msg.target_pos_y - initial_y
            step = np.sqrt(d_x * d_x + d_y * d_y)
            d_theta = step / self.radius

            desire_coord = DesireCoord()
            desire_coord.roll = 1.57
            desire_coord.pitch = initial_theta + d_theta
            self.desire_coord_pub.publish(desire_coord)




if __name__ == '__main__':
    rospy.init_node("rolling_demo_node")

    RollingDemo()

    rospy.spin()
