#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Vector3, PoseStamped, Pose, Quaternion
from std_msgs.msg import Empty, Float64
from util import dist
from variables import *
import numpy as np
from scipy.spatial.transform import Rotation as R

class Navigator():
    def __init__(self, deceleration_radius = 1.0, dist_close = 0.3):
        self.drone_pose_sub = rospy.Subscriber('crobat/mocap/pose',                 
                                               PoseStamped,
                                               self.drone_pose_sub_callback)
        self.chest_pose_sub = rospy.Subscriber('mocap_node/mocap/chest/pose',                      
                                               PoseStamped,                                         
                                               self.chest_pose_sub_callback)
        self.hand_pose_sub = rospy.Subscriber('mocap_node/mocap/hand/pose',
                                              PoseStamped,                                          
                                              self.hand_pose_sub_callback)                          
        self.deceleration_radius = deceleration_radius
        self.approach_start_sub = rospy.Subscriber('approach_start',
                                                   Empty,
                                                   self.approach_start_sub_callback)
        self.approach_stop_sub = rospy.Subscriber('approach_stop',
                                                  Empty,
                                                  self.approach_stop_sub_callback)
        self.approach_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped)
        self.distance_chest_drone_on_XY_plane_pub = rospy.Publisher('distance/chest_drone_on_XY_plane', Float64)
        self.distance_chest_hand_on_XY_plane_pub = rospy.Publisher('distance/chest_hand_on_XY_plane', Float64)
        self.distance_hand_drone_on_XY_plane_pub = rospy.Publisher('distance/hand_drone_on_XY_plane', Float64)
        self.circular_pub = rospy.Publisher('circular', Float64)
        self.running = False
        self.dist_close = dist_close

    def drone_pose_sub_callback(self, msg):
        self.drone_pose = msg.pose

    def chest_pose_sub_callback(self, msg):
        self.chest_pose = msg.pose

    def hand_pose_sub_callback(self, msg):
        self.hand_pose = msg.pose

    def approach_start_sub_callback(self, msg):
        if not self.running:
            self.run()

    def approach_stop_sub_callback(self, msg):
        if self.running:
            self.running = False
        
    def run(self, timeout=120):
        rospy.loginfo('approach started')
        self.running = True
        start_t = rospy.get_time()
        while rospy.get_time() <= start_t + timeout and self.running:
            chest_pos = self.chest_pose.position
            hand_pos = self.hand_pose.position
            drone_pos = self.drone_pose.position
            distance_chest_drone_on_XY_plane = dist(chest_pos, drone_pos, True)
            distance_chest_hand_on_XY_plane = dist(chest_pos, hand_pos, True)
            distance_hand_drone_on_XY_plane = dist(hand_pos, drone_pos, True)
            self.distance_chest_drone_on_XY_plane_pub.publish(Float64(distance_chest_drone_on_XY_plane))
            self.distance_chest_hand_on_XY_plane_pub.publish(Float64(distance_chest_hand_on_XY_plane))
            self.distance_hand_drone_on_XY_plane_pub.publish(Float64(distance_hand_drone_on_XY_plane))
            height_above_hand = 0.3
            scale = 0.8 if distance_hand_drone_on_XY_plane > self.dist_close else 0.5
            goal_dist_z = (hand_pos.z + height_above_hand - drone_pos.z) * scale
            goal_pos_z = hand_pos.z + height_above_hand - goal_dist_z
            goal_ori = Quaternion(0, 0, 0, 0)
            rospy.loginfo('hand_pos: {}'.format(hand_pos))
            rospy.loginfo('chest_pos: {}'.format(chest_pos))
            rospy.loginfo('drone_pos: {}'.format(drone_pos))
            if distance_chest_drone_on_XY_plane < distance_chest_hand_on_XY_plane:
                self.circular_pub.publish(1)
                rospy.loginfo('drone is running on circle.')
                vector_c_h = np.array([hand_pos.x - chest_pos.x, hand_pos.y - chest_pos.y])
                vector_c_d = np.array([drone_pos.x - chest_pos.x, drone_pos.y - chest_pos.y])
                cross = np.cross(vector_c_h, vector_c_d)
                dot = np.dot(vector_c_h, vector_c_d)
                goal_theta = np.arctan2(cross, dot) * scale
                r_matrix = np.array([[np.cos(goal_theta), -np.sin(goal_theta)],
                                     [np.sin(goal_theta), np.cos(goal_theta)]])
                vector_c_g = r_matrix @ vector_c_h
                goal_pos = Vector3(chest_pos.x + vector_c_g[0],
                                   chest_pos.y + vector_c_g[1],
                                   goal_pos_z)
                goal_pose = PoseStamped()
                goal_pose.pose.position = goal_pos
                goal_pose.pose.orientation = goal_ori
                self.approach_pub.publish(goal_pose)
                rospy.loginfo('goal_pos: {}'.format(goal_pos))
            else:
                self.circular_pub.publish(0)
                if distance_chest_drone_on_XY_plane < self.deceleration_radius:
                    goal_dist_x = (hand_pos.x - drone_pos.x) * scale
                    goal_dist_y = (hand_pos.y - drone_pos.y) * scale
                    goal_pose = PoseStamped()
                    goal_pose.pose.position = goal_pos
                    goal_pose.pose.orientation = goal_ori
                    self.approach_pub.publish(goal_pose)
                    rospy.loginfo('goal_pos: {}'.format(goal_pos))
                    rospy.loginfo('goal_dist_y: {}'.format(goal_dist_y))
                else:
                    rospy.loginfo('outside of deceleration_radius')
                    vector_d_h = np.array([hand_pos.x - drone_pos.x, hand_pos.y - drone_pos.y])
                    vector_d_h_normalized = vector_d_h / np.linalg.norm(vector_d_h)
                    goal_pos = Vector3(drone_pos.x + vector_d_h_normalized[0] * self.deceleration_radius * (1 - scale),
                                       drone_pos.y + vector_d_h_normalized[1] * self.deceleration_radius * (1 - scale),
                                       goal_pos_z)
                    goal_pose = PoseStamped()
                    goal_pose.pose.position = goal_pos
                    goal_pose.pose.orientation = goal_ori
                    self.approach_pub.publish(goal_pose)
                    rospy.loginfo('vector_d_h: {}'.format(vector_d_h))
                    rospy.loginfo('vector_y: {}'.format(vector_d_h_normalized[1]))
                    rospy.loginfo('goal_pos: {}'.format(goal_pos))
            rospy.sleep(0.1)
        self.running = False
        rospy.loginfo('drone stay')

    def look_at_quaternion(self, r, p, theta_scale=0.1):
        vector_r_p = np.array([p.x - r.x, p.y - r.y])
        e_x = np.array([1, 0])
        cross = np.cross(e_x, vector_r_p)
        dot = np.dot(e_x, vector_r_p)
        chest_theta = np.arctan2(cross, dot)
        drone_ori = self.drone_pose.orientation
        current_theta = R.from_quat([drone_ori.x, drone_ori.y, drone_ori.z, drone_ori.w]).as_euler('xyz')[2]
        goal_theta = (chest_theta - current_theta) * theta_scale + current_theta
        quaternion = R.from_euler('xyz', [0, 0, goal_theta]).as_quat()
        quaternion = Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        return quaternion  
    

if __name__ == '__main__':
    rospy.init_node('navigator')
    Navigator()
    rospy.spin()
    
