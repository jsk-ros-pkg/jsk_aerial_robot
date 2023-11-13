#!/usr/bin/env python

import rospy
import roslib
roslib.load_manifest('learning_tf')
import math
import numpy as np
import cv2
import tf
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import tf2_
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class Approaching_human():
    def __init__(self):
        self.flight_state_sub = rospy.Subscriber('/quadrotor/flight_state',UInt8,self.flight_state_cb)
        self.flight_state_msg = UInt8()
        self.flight_start_flag = True
        self.flight_state_flag = False
        self.approaching_flag = False

        self.n = 0
        self.rotate_cnt = 0
        self.rotate_flag = True

        self.camera_height = 480
        self.camera_width = 640
        self.camera_param = 0.8
        #rect(ROS image)
        self.rect_sub = rospy.Subscriber('/human',RectArray,self.rect_cb)
        self.rects = RectArray()
        self.max_index = 0
        self.max_rect = RectArray()
        self.max_rect_pos = Vector3()
        self.max_rect_pixel_pos = Vector3()
        self.max_rect_area = 0.0
        self.max_thresh_area  = 370000 #rospy.get_param("~max_thresh_area",370000)

        #depth(OpenCV image)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw',Image,self.camera_depth_cb)
        #self.camera_depth_image = Image()
        self.bridge = CvBridge()
        self.cv_image = np.empty((self.camera_height,self.camera_width))
        self.depth = 0
        self.prev_depth = 0
        self.depth_sum = 0
        self.depth_thresh = 5000

        self.move_pub = rospy.Publisher('/quadrotor/uav/nav',FlightNav,queue_size = 10)
        self.move_msg = FlightNav()
        self.land_pub =rospy.Publisher('/quadrotor/teleop_command/land',Empty,queue_size = 10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)


    def rect_cb(self,msg):
        self.rects = msg
        self.n = len (self.rects.rects)

    def flight_state_cb(self,msg):
        self.flight_state_msg = msg

    def camera_depth_cb(self,msg):
        #self.camera_depth_image = msg
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)

    def flight_rotate_state(self):
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
        #画面中心から見たrectの中心位置
        self.max_rect_pos.x = (self.max_rect.x + (self.max_rect.width/2)) - (self.camera_width/2)
        self.max_rect_pos.y = (self.max_rect.y + (self.max_rect.height/2)) + (self.camera_height/2)
        #ピクセル指定したいときのrectの中心位置
        self.max_rect_pos.x = self.max_rect.x + (self.max_rect.width/2)
        self.max_rect_pos.y = self.max_rect.y + (self.max_rect.height/2)
        self.depth = self.cv_image.item(self.max_rect_pos.x,self.max_rect_pos.y)
        depth = self.camera_depth.data(self.max_rect_pos.x,self.max_rect_pos.y)
        print(depth)
        yaw = self.max_rect_pos.x * (math.pi/self.camera_width)* self.camera_param
        self.move_msg.target_yaw = yaw
        self.move_msg.target_pos_z = self.max_rect_pos.y * 0.001
        self.move_pub.publish(self.move_msg)
        self.rotate_flag = False

    def relative_pos(self):
        self.max_rect_pixel_pos.x = int(self.max_rect.x + (self.max_rect.width/2))
        self.max_rect_pixel_pos.y = int(self.max_rect.y + (self.max_rect.height/2))
        depth = self.cv_image.item(self.max_rect_pixel_pos.y,self.max_rect_pixel_pos.x)
        print(depth)
        if self.flight_start_flag:
            self.depth = depth/1000 - 0.3 #############
        else:
            if depth <= self.depth_thresh and 0 < depth:
                self.depth = depth/1000 - 0.3 #############
                self.depth_thresh = self.prev_depth*1000 + 800 
            else:
                self.depth = 0
                print("else")
        rospy.loginfo("depth: %s",self.depth)
        rospy.loginfo("prev_depth: %s",self.prev_depth)

    def approaching_human(self):
        if self.max_thresh_area > self.max_rect_area:
            self.move_msg.target_pos_y = self.max_rect_area*0.001

        else:
            self.move_msg.target_pos_y = 0.0
            self.move_pub.publish(self.move_msg)
            self.land_pub.publish()

    #最初に飛行開始、人物見つけたら近づいていく　ある程度近づいたらlandする
    def timerCallback(self,event):
        self.flight_state()
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

