#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
import copy
import cv2

from aerial_robot_base.robot_interface import RobotInterface
from aerial_robot_base.state_machine import *
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from aerial_robot_msgs.msg import FlightNav
from message_filters import ApproximateTimeSynchronizer, Subscriber
# import cv2
import numpy as np
from std_msgs.msg import Empty
import time
import numpy as np
from tf.transformations import quaternion_inverse, quaternion_multiply, euler_from_quaternion, random_quaternion
from geometry_msgs.msg import PoseStamped, Wrench, Vector3, Vector3Stamped, WrenchStamped, Quaternion, QuaternionStamped


class PaintDemo():
    def __init__(self):
        # self.ri = RobotInterface()
        rospy.sleep(1.0) # wait for joint updated
        self.nav_pub = rospy.Publisher('uav/nav', FlightNav, queue_size=1)
        self.robot = RobotInterface()
        self.body_mocap_sub = rospy.Subscriber('mocap/pose', PoseStamped, self.body_mocap_cb)
        self.rotor5_mocap_sub = rospy.Subscriber('thrust5/mocap/pose', PoseStamped, self.rotor5_mocap_cb)

        self.state = 0

        self.update_hz = 50
        self.close_count = 0
        self.body_mocap_update_stamp = 0
        self.rotor5_mocap_update_stamp = 0
        self.start_pos = None

    def body_mocap_cb(self, msg):
        self.body_pos = np.array([msg.pose.position.x,
                                  msg.pose.position.y,
                                  msg.pose.position.z])
        self.body_ori = np.array([msg.pose.orientation.x,
                                  msg.pose.orientation.y,
                                  msg.pose.orientation.z,
                                  msg.pose.orientation.w])
        self.body_mocap_update_stamp = rospy.Time.now().to_sec()
    
    def rotor5_mocap_cb(self, msg):
        self.rotor5_pos = np.array([msg.pose.position.x,
                                    msg.pose.position.y,
                                    msg.pose.position.z])
        self.rotor5_ori = np.array([msg.pose.orientation.x,
                                    msg.pose.orientation.y,
                                    msg.pose.orientation.z,
                                    msg.pose.orientation.w])
        self.rotor5_mocap_update_stamp = rospy.Time.now().to_sec()

    def run(self):
        r = rospy.Rate(self.update_hz)
        self.start_pos = self.robot.getCogPos()

        dest_yaw = -1.57

        while not rospy.is_shutdown():

            if self.state == 0:
                ## step1: go to the starting point
                input("Press Enter to go to the starting point...")
                self.robot.goPosYaw(pos = np.array([0.0, 0.0, 0.6]), yaw = dest_yaw, vel_thresh = 0.05, yaw_thresh = 0.05)
                
                user_input = input("Press Enter to proceed to next step...")
                if user_input == '':
                    self.state = 1

            elif self.state == 1:
                ## step2: slowly goes forward and reach for the wall
                input("Press Enter to slowly approach the wall...")

                if self.body_mocap_update_stamp + 1.0 < rospy.Time.now().to_sec() or \
                self.rotor5_mocap_update_stamp + 1.0 < rospy.Time.now().to_sec():
                    print("mocap data timeout")
                    self.robot.goVel(vel = np.array([0.0, 0.0, 0.0]))
                    continue

                while not rospy.is_shutdown():
                    self.robot.goVel(vel = np.array([0.0, 0.2, 0.0]))
                    # if self.body_pos[1] - self.rotor5_pos[1] < 0.15:
                    self.close_count += 1
                    rospy.sleep(0.1)
                    # else:
                        # self.close_count = 0
                
                    if self.close_count > 50:
                        self.robot.goVel(vel = np.array([0.0, 0.0, 0.0]))
                        print("Reached the wall")

                        user_input = input("Press Enter to proceed to next step...")
                        if user_input == '':
                            self.state = 2
                            break
                
            elif self.state == 2:
                ## step3: start painting (slowly move along the wall and paint)

                ## if the robot rotate in the yaw direction, adjust the yaw rate
                ## if the body_pos and rotor5_pos in the y direction differ too much, adjust the y velocity
                ## if there is not problem listed above, slowly move forward in the x direction

                input("Press Enter to start painting...")

                while not rospy.is_shutdown():

                    if self.body_mocap_update_stamp + 1.0 < rospy.Time.now().to_sec() or \
                    self.rotor5_mocap_update_stamp + 1.0 < rospy.Time.now().to_sec():
                        print("mocap data timeout")
                        self.robot.goVel(vel = np.array([0.0, 0.0, 0.0]))
                        continue

                    angle = euler_from_quaternion(self.body_ori, axes='sxyz')
                    yaw = angle[2]
                    if abs(yaw - dest_yaw) > math.radians(10):
                        yaw_rate = -0.1 if yaw > 0 else 0.1
                        self.robot.rotateYaw(yaw = yaw_rate)
                        print("adjusting yaw")
                        continue
                    
                    # if self.rotor5_pos[1] - self.body_pos[1] > 0.1:
                    #     y_vel = 0.05
                    #     self.robot.goVel(vel = np.array([0.0, y_vel, 0.0]))
                    #     print("adjusting y position")
                    #     continue

                    x_vel = 0.05
                    self.robot.goVel(vel = np.array([x_vel, 0.0, 0.0]))
                    print("painting...")

                    if self.body_pos[0] > 0.6:
                        self.robot.goVel(vel = np.array([0.0, 0.0, 0.0]))
                        print("Finished painting")

                        user_input = input("Press Enter to proceed to next step...")
                        if user_input == '':
                            self.state = 3
                            break
            
            elif self.state == 3:
                ## step4: return to the starting point and land
                self.robot.goPos(pos = self.start_pos, vel_thresh = 0.05)

            rospy.sleep(0.1)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("paint_demo")
    node = PaintDemo()
    node.run()