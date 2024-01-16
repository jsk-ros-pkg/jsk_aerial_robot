#!/usr/bin/env python

import rospy
import math
import numpy as np
import cv2
import tf
from jsk_recognition_msgs.msg import RectArray
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Empty
from std_msgs.msg import UInt8
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Odometry

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
        self.land_cnt = 0
        self.rotate_flag = True
        self.offset_yaw = 2*math.pi

        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info',CameraInfo,self.camera_info_cb)
        self.camera_info = CameraInfo()
        self.target_pos = Vector3()
        self.target_pos_pub = rospy.Publisher('/3D_pos',Vector3,queue_size = 1)
        self.camera_height = 720
        self.camera_width = 1280
        self.camera_param = 0.3
        #rect(ROS image)
        self.rect_sub = rospy.Subscriber('/human',RectArray,self.rect_cb)
        self.rects = RectArray()
        self.max_index = 0
        self.max_rect = RectArray()
        self.max_rect_pos = Vector3()
        self.max_rect_pixel_pos = Vector3()
        self.max_rect_area = 0.0
        self.max_thresh_area  = 370000 #rospy.get_param("~max_thresh_area",370000)

        self.odom_sub = rospy.Subscriber('/quadrotor/uav/cog/odom',Odometry,self.odom_cb)
        self.pose = Odometry()
        self.euler = Vector3()

        self.Kp = 0.1
        self.Ki = 0.0001
        self.Kd = 0.01
        #self.offset = 0.09/(0.08*abs(self.degree[2]-self.initial_position[2])+0.3) + 1

        #depth(OpenCV image)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw',Image,self.camera_depth_cb)
        #self.camera_depth_image = Image()
        self.bridge = CvBridge()
        self.cv_image = np.empty((self.camera_height,self.camera_width))
        self.depth = 0
        self.prev_depth = 0
        self.depth_sum = 0
        self.depth_thresh_max = 10000
        self.depth_thresh_min = 0
        self.min_depth = 10.0

        self.perching_cnt = 0
        self.perching_pub = rospy.Publisher('/perching_state',UInt8,queue_size = 1)
        self.perching_data = UInt8()

        self.move_pub = rospy.Publisher('/quadrotor/uav/nav',FlightNav,queue_size = 10)
        self.move_msg = FlightNav()
        self.move_msg.control_frame = 1
        self.move_msg.target = 1
        self.move_msg.pos_xy_nav_mode = 1
        self.move_msg.yaw_nav_mode = 2
        self.land_pub =rospy.Publisher('/quadrotor/teleop_command/land',Empty,queue_size = 10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timerCallback)

    def camera_info_cb(self,msg):
        self.camera_info = msg
        self.camera_height = self.camera_info.height
        self.camera_width = self.camera_info.width

    def rect_cb(self,msg):
        self.rects = msg
        self.n = len (self.rects.rects)

    def flight_state_cb(self,msg):
        self.flight_state_msg = msg
        #print(self.flight_state_msg)

    def camera_depth_cb(self,msg):
        #self.camera_depth_image = msg
        self.cv_image = self.bridge.imgmsg_to_cv2(msg)

    def odom_cb(self,msg):
        self.pose = msg.pose.pose
        self.height = self.pose.position.z
        quaternion_x = self.pose.orientation.x
        quaternion_y = self.pose.orientation.y
        quaternion_z = self.pose.orientation.z
        quaternion_w = self.pose.orientation.w
        euler = tf.transformations.euler_from_quaternion((quaternion_x,quaternion_y,quaternion_z,quaternion_w))
        self.euler = Vector3(x=euler[0],y=euler[1],z=euler[2])

    def flight_rotate_state(self):
        # if self.flight_state_msg.data == 3:
        #     self.flight_start_flag = True
            # print(self.flight_start_flag.data)
        if self.flight_state_msg.data == 5:#5
            self.flight_state_flag = True
        else:
            self.flight_state_flag = False

        # if self.flight_state_flag:
        #     if self.flight_start_flag:
        #         rospy.sleep(2.0)
        #         self.flight_start_flag = False
        if self.rotate_cnt >= 20:
            self.rotate_flag = True
            self.rotate_cnt = 0
    
    # def flight_state(self):
    #     if self.flight_state_msg.data == 5:#5
    #         self.flight_state_flag = True
    #     else:
    #         self.flight_state_flag = False
    # def rotate_state(self):
    #     if self.rotate_cnt >= 20:
    #         self.rotate_flag = True
    #         self.rotate_cnt = 0

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

    def pos_cal(self):
        self.max_rect_pos.x = (self.max_rect.x + (self.max_rect.width/2)) - (self.camera_width/2)
        self.max_rect_pos.y = -(self.max_rect.y + (self.max_rect.height/2)) + (self.camera_height/2)
        self.camera_height = self.camera_info.height
        self.camera_width = self.camera_info.width
        # fx,cx = self.camera_info.K[0],self.camera_info.K[2]
        # fy,cy = self.camera_info.K[4],self.camera_info.K[5]
        # target_pos_x = (self.max_rect.x - cx)  / fx
        # target_pos_y = (self.max_rect.y - cy) * self.depth / fy
        # rospy.loginfo("target_pos: %s",target_pos_x)
        # self.target_pos.z = self.depth

        # camera_K = np.reshape(self.camera_info.K,(3,3))
        # target_camera_pos = np.array([self.max_rect_pos.x,self.max_rect_pos.y,1])
        # print(target_camera_pos)
        # K_inv = np.linalg.inv(camera_K.T)
        
        # target_pos = np.dot(target_camera_pos,K_inv)*self.depth
        # self.target_pos.x,self.target_pos.y,self.target_pos.z = target_pos[0],target_pos[1],target_pos[2]
        # self.target_pos_pub.publish(self.target_pos)

    def rotate_yaw(self):
        yaw = 0
        yaw = self.max_rect_pos.x * (math.pi/self.camera_width)* self.camera_param
        rospy.loginfo("max_rect_pos: %s",self.max_rect_pos.x)
        self.move_msg.target_yaw = -yaw + self.euler.z
        self.target_pos.z = self.move_msg.target_yaw
        # self.move_msg.target_yaw = yaw
        rospy.loginfo("rotate_yaw: %s",self.move_msg.target_yaw)
        # self.move_msg.target_pos_z = self.max_rect_pos.y * 0.001 + my height
        self.move_pub.publish(self.move_msg)
        self.rotate_flag = False

    def relative_pos(self):
        self.max_rect_pixel_pos.x = int(self.max_rect.x + (self.max_rect.width/2))
        self.max_rect_pixel_pos.y = int(self.max_rect.y + (self.max_rect.height/2))
        self.target_pos.x = self.max_rect_pixel_pos.x
        self.target_pos.y = self.max_rect_pixel_pos.y
        depth = self.cv_image.item(self.max_rect_pixel_pos.y,self.max_rect_pixel_pos.x)
        print(depth)
        # pixel_cnt = 0
        # depth_sum = 0
        # depth_tmp = 0
        # for i in range (10):
        #     x = np.random.randint(-30,30)
        #     y = np.random.randint(-30,30)
        #     pixel_x = self.max_rect_pixel_pos.x + x
        #     pixel_y = self.max_rect_pixel_pos.y + y
        #     depth_center = self.cv_image.item(self.max_rect_pixel_pos.y,self.max_rect_pixel_pos.x)
        #     depth_tmp = self.cv_image.item(pixel_y,pixel_x)
        #     if depth_tmp <= (depth_center + 200) and (depth_center- 200) <= depth_tmp:
        #         depth_sum += depth_tmp
        #         pixel_cnt += 1
        # if pixel_cnt > 0:
        #     depth = depth_sum /pixel_cnt
        # else:
        #     depth = depth_center
        # print(depth)
        
        if self.flight_start_flag:
            self.depth = depth/1000 - 0.5 #############
        else:
            if depth <= self.depth_thresh_max and self.depth_thresh_min <= depth:
                self.depth = depth/1000 - 0.5 #############
                self.depth_thresh_max = self.prev_depth*1000 + 800
                self.depth_thresh_min = self.prev_depth*1000 - 800
                if self.min_depth >= self.depth:
                    if self.prev_depth - self.depth < 0.15:
                        self.min_depth = self.depth
            elif depth <= 0:
                self.depth = 0
            else:
                self.depth = 1000000
                print("else")
        rospy.loginfo("depth: %s",self.depth)
        #rospy.loginfo("prev_depth: %s",self.prev_depth)


    def PID_control(self):
        self.output = self.depth * self.Kp + (self.depth - self.prev_depth) * self.Kd + self.depth_sum*self.Ki
        self.prev_depth = self.depth
        self.depth_sum += self.depth
        rospy.loginfo("vel: %s",self.output)

    # def approaching_human(self):
    #     if self.max_thresh_area > self.max_rect_area:
    #         self.move_msg.target_pos_y = self.max_rect_area*0.001

    #     else:
    #         self.move_msg.target_pos_y = 0.0
    #         self.move_pub.publish(self.move_msg)
    #         self.land_pub.publish()
    #add PID control to approach human slowly (relative pos -> acc, vel)

    def ready_for_perching(self):
        # depth_tmp = 0
        # depth_sum = 0
        # for i in range (100):
        #     x = np.random.randint(0,(self.camera_width -1))
        #     y = np.random.randint(0,(self.camera_height -1))
        #     depth_tmp = self.cv_image.item(y,x)
        #     depth_sum += depth_tmp
        # if depth_sum < 100:
        self.perching_cnt += 1
        print("perching")
        print(self.perching_cnt)


    def timerCallback(self,event):
        self.flight_rotate_state()
        self.perching_data = 0
        self.perching_pub.publish(self.perching_data)
        if self.flight_state_flag:
            if self.n >= 1:
                self.finding_max_rect()
                if self.rotate_flag:
                    self.rotate_yaw()
                    rospy.loginfo("rotate!")
                self.relative_pos()
                self.pos_cal()
                print(self.land_cnt)
                self.target_pos_pub.publish(self.target_pos)
                if self.depth > 0 and self.depth < 100000:
                    self.PID_control()
                    self.move_msg.target_vel_x = self.output
                    self.move_pub.publish(self.move_msg)
                    rospy.loginfo("go!")
                    #rospy.loginfo("cmd_vel: %s",self.output)
                    self.land_cnt = 0
                    self.flight_start_flag = False
                elif self.depth <= 0:
                    self.move_msg.target_vel_x = 0.0
                    self.move_pub.publish(self.move_msg)
                    rospy.loginfo("stop!")
                    self.land_cnt += 1
                    if self.land_cnt >= 10:
                        for i in range (10):
                            self.perching_data = 1
                            self.perching_pub.publish(self.perching_data)
                        #rospy.loginfo("land!")
                        #self.land_pub.publish()
                        rospy.sleep(5.0)
                        self.land_cnt = 0
                self.rotate_cnt += 1
            else:
                rospy.loginfo("don't see people")
                self.move_msg.target_vel_x = 0.0
                self.move_pub.publish(self.move_msg)
                rospy.loginfo("stop! because not see people")
                if self.min_depth > 2.0:
                    self.move_msg.target_yaw = self.euler.z + 0.01
                    self.move_pub.publish(self.move_msg)
                    rospy.loginfo("not see yaw: %s",self.move_msg.target_yaw)
                    rospy.sleep(1.0)
                # self.move_msg.yaw_nav_mode = 0


        # self.flight_state()
        # if self.flight_state_flag:
        #     while self.land_cnt < 30:
        #         print(self.land_cnt)
        #         self.rotate_state()
        #         self.perching_data = 0
        #         self.perching_pub.publish(self.perching_data)
        #         if self.n >= 1:
        #             self.finding_max_rect()
        #             if self.rotate_flag:
        #                 self.rotate_yaw()
        #                 rospy.loginfo("rotate!")
        #             self.relative_pos()
        #             self.pos_cal()
        #             self.target_pos_pub.publish(self.target_pos)
        #             if self.depth > 0:
        #                 self.PID_control()
        #                 self.move_msg.target_vel_x = self.output
        #                 self.move_pub.publish(self.move_msg)
        #                 rospy.loginfo("go!")
        #                 #rospy.loginfo("cmd_vel: %s",self.output)
        #                 self.land_cnt = 0
        #                 self.flight_start_flag = False
        #             else:
        #                 self.move_msg.target_vel_x = 0.0
        #                 self.move_pub.publish(self.move_msg)
        #                 rospy.loginfo("stop!")
        #                 self.land_cnt += 1
        #             self.rotate_cnt += 1
        #         else:
        #             rospy.loginfo("don't see people")
        #             self.move_msg.target_vel_x = 0.0
        #             self.move_pub.publish(self.move_msg)
        #             rospy.loginfo("stop! because not see people")
        #             if self.min_depth < 0.1:
        #                 self.land_cnt += 1
        #     if self.land_cnt >= 30:
        #         self.ready_for_perching()
        #         if self.perching_cnt >= 10:
        #             self.perching_data = 1
        #             self.perching_pub.publish(self.perching_data)
        #             rospy.loginfo("land!")
        #             self.land_pub.publish()

if __name__ == '__main__':
    rospy.init_node("Approaching_human")
    node = Approaching_human()
    rospy.spin()

