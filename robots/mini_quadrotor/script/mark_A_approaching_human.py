#!/usr/bin/env python
#-*-coding: utf-8 -*-

import rospy
import math
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav

class Approaching_human():
    def __init__(self):
        self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state',UInt8,self.flight_state_cb)
        self.flight_state_msg = UInt8()
        self.flight_start_flag = False
        self.flight_state_flag = False
        self.approaching_flag = False

        self.n = 0
        self.rotate_cnt = 0
        self.rotate_flag = True

        self.camera_height = 720
        self.camera_width = 960
        self.camera_param = 0.8
        
        self.rect_sub = rospy.Subscriber('/human/output/rects',RectArray,self.rect_cb)
        self.rects = RectArray()
        self.max_index = 0
        self.max_rect = RectArray()
        self.max_rect_pos = Vector3()
        self.max_rect_area = 0.0
        self.max_thresh_area  = 370000 #rospy.get_param("~max_thresh_area",370000)
        
        self.move_pub = rospy.Publisher('/quadrotor/uav/nav',FlightNav,queue_size = 10)
        self.move_msg = FlightNav()
        self.land_pub =rospy.Publisher('/quadrotor/teleop_command/land',Empty,queue_size = 10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)


    def rect_cb(self,msg):
        self.rects = msg
        self.n = len (self.rects.rects)

    def flight_state_cb(self,msg):
        self.flight_state_msg = msg

    def flight_and_rotate_state(self):
        if self.flight_state_msg == 3:
            self.flight_start_flag = True
        if self.flight_state_msg == 5:
            self.flight_state_flag = True
        else:
            self.flight_state_flag = False
        if self.flight_state_flag:
            if self.flight_start_flag:
                rospy.sleep(2.0)
                self.flight_start_flag = False
        if self.rotate_cnt >= 4:
                self.rotate_flag = True
                self.rotate_cnt = 0

    def finding_max_rect(self):
        max_area = 0
        for i in range(self.n):
            rect = self.rects.rects[i]
            tmp_area = rect.height*rect.width
            if tmp_area >= max_area:
                max_area = tmp_area
                self.max_index = i
        self.max_rect = self.rects.rects[self.max_index]
        self.max_rect_area = max_area

    def rotate_yaw(self):
        yaw = 0
        self.max_rect_pos.x = (self.max_rect.x + (self.max_rect.width/2)) - (self.camera_width/2)
        self.max_rect_pos.y = (self.max_rect.y + (self.max_rect.height/2)) + (self.camera_height/2)
        yaw = self.max_rect_pos.x * (math.pi/self.camera_width)* self.camera_param
        self.move_msg.target_yaw = yaw
        self.move_msg.target_pos_z = self.max_rect_pos.y * 0.001
        self.move_pub.publish(self.move_msg)
        self.rotate_flag = False

    def approaching_human(self):
        if self.max_thresh_area > self.max_rect_area:
            self.move_msg.target_pos_y = self.max_rect_area*0.001
            self.move_pub.publish(self.move_msg)
            self.rotate_cnt += 1
        else:
            self.move_msg.target_pos_y = 0.0
            self.move_pub.publish(self.move_msg)
            self.land_pub.publish()

    #最初に飛行開始、人物見つけたら近づいていく　ある程度近づいたらlandする
    def timerCallback(self,event):
        self.flight_and_rotate_state()
        if self.flight_state_flag:
            if self.n >= 1:
                self.finding_max_rect()
                if self_rotate_flag:
                    self.rotate_yaw()
                self.approaching_human()
            else:
                self.move_msg.target_yaw = 0.4
                self.move_pub.publish(self.move_msg)

if __name__ == '__main__':
    rospy.init_node("Approaching_human")
    node = Approaching_human()
    rospy.spin()
