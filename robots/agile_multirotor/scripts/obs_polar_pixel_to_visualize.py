#!/usr/bin/env python3
import rospy


from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from aerial_robot_msgs.msg import ObstacleArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np

class AgileQuadState:
    def __init__(self, quad_state,transition):
        # self.t = quad_state.t

        self.pos = np.array([quad_state.pose.pose.position.x-transition[0],
                             quad_state.pose.pose.position.y-transition[1],
                             quad_state.pose.pose.position.z], dtype=np.float32)
        self.att = np.array([quad_state.pose.pose.orientation.x,
                             quad_state.pose.pose.orientation.y,
                             quad_state.pose.pose.orientation.z,
                             quad_state.pose.pose.orientation.w], dtype=np.float32)
        self.vel = np.array([quad_state.twist.twist.linear.x,
                             quad_state.twist.twist.linear.y,
                             quad_state.twist.twist.linear.z], dtype=np.float32)
        self.omega = np.array([quad_state.twist.twist.angular.x,
                               quad_state.twist.twist.angular.y,
                               quad_state.twist.twist.angular.z], dtype=np.float32)

class PolarPixelVisualizeNode:
    def __init__(self):
        rospy.init_node('obs_polar_pixel_visualize_node', anonymous=False)

        quad_name = rospy.get_param("~robot_ns")
        self.body_r = rospy.get_param("~body_r")  #radius of quadrotor(0.25~0.5)
        shift_x = rospy.get_param("~shift_x")
        shift_y = rospy.get_param("~shift_y")

        theta_list = np.array([5,15,25,35,45,60,75,90, 105, 120]) #deg
        acc_theta_list = np.array([1, 4, 7, 10]) #deg
        max_deg = 135 #deg
        resolution_deg = 0.25 #deg
        self.min_detection_range = 0.1
        self.max_detection_range = 10 #max_detection_range when learning. this is not the same as the real hokuyo hardware

        self.theta_list = np.deg2rad(np.concatenate([-np.flip(theta_list), theta_list])) #rad
        self.acc_theta_list = np.deg2rad(np.concatenate([-np.flip(acc_theta_list), acc_theta_list])) #rad
        self.max_rad = np.deg2rad(max_deg) #rad
        self.resolution_rad = np.deg2rad(resolution_deg) #rad
        self.range_num : int = int(np.round(max_deg*2/resolution_deg))+1
        self.translation_position = np.array([shift_x, shift_y],dtype="float32")

        # init state
        self.publish_commands = False
        self.state = None

        self.obs_polar_pixel_sub = rospy.Subscriber("/" + quad_name + "/debug/obs_polar_pixel", ObstacleArray, self.obs_polar_pixel_conversion_callback, 
                                        queue_size=1, tcp_nodelay=True)
        self.odom_sub = rospy.Subscriber("/" + quad_name + "/uav/cog/odom", Odometry, self.state_callback,
                                    queue_size=1, tcp_nodelay=True)
        self.start_sub = rospy.Subscriber("/" + quad_name + "/start_navigation", Empty, self.start_callback,
                                    queue_size=1, tcp_nodelay=True)
        self.visualize_polar_pixel_pub = rospy.Publisher("/" + quad_name + "/debug/visualize/obs_depth", LaserScan,
                                          queue_size=1)
        self.quadrotor_pos_pub = rospy.Publisher("/" + quad_name + "/debug/visualize/quadrotor_pos", Marker,
                                          queue_size=1)
        print("Initialization completed!")


    def obs_polar_pixel_conversion_callback(self, obs_polar_pixel: ObstacleArray):
        vel_calc_boundary = 0.3
        # state_data: ObstacleArray
        # state_data header is 0 , should examine if it works
        obs_pp_visualize = LaserScan()
        obs_pp_visualize.header.stamp = obs_polar_pixel.header.stamp
        obs_pp_visualize.header.frame_id = "multirotor/laser_frame"
        obs_pp_visualize.angle_min = -self.max_rad
        obs_pp_visualize.angle_max = self.max_rad
        obs_pp_visualize.angle_increment = self.resolution_rad
        obs_pp_visualize.range_min = self.min_detection_range
        obs_pp_visualize.range_max = self.max_detection_range
        obs_pp_visualize.ranges = [float("inf")]*self.range_num
        obs_pp_visualize.intensities = [0]*self.range_num

        vel_direction = 0 if np.linalg.norm(np.array([self.state.vel[0], self.state.vel[1]]))<vel_calc_boundary \
            else np.arctan2(self.state.vel[1], self.state.vel[0]) # rad

        for theta, obs_depth in zip(self.theta_list, obs_polar_pixel.boxel):
            idx:int  = int((theta + self.max_rad)/self.resolution_rad)
            if idx < self.range_num:
                obs_pp_visualize.ranges[idx] = self.depth_visualize(obs_depth)
        
        for theta, acc_obs_depth in zip(self.acc_theta_list, obs_polar_pixel.acc_boxel):
            idx:int  = int((theta + vel_direction + self.max_rad)/self.resolution_rad)
            if idx < self.range_num:
                obs_pp_visualize.ranges[idx] = self.depth_visualize(acc_obs_depth)
                obs_pp_visualize.intensities[idx] = 0.5
        self.visualize_polar_pixel_pub.publish(obs_pp_visualize)
    
    def state_callback(self, state_data):
        self.state = AgileQuadState(state_data,self.translation_position)
        marker_data = Marker()
        marker_data.header.frame_id = "world"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "quadrotor"
        marker_data.id = 0

        marker_data.action = Marker.ADD
        marker_data.type = Marker.CYLINDER
        marker_data.pose.position.x = self.state.pos[0] + self.translation_position[0]
        marker_data.pose.position.y = self.state.pos[1] + self.translation_position[1]
        marker_data.pose.position.z = self.state.pos[2]
        marker_data.pose.orientation.x = 0.0
        marker_data.pose.orientation.y = 0.0
        marker_data.pose.orientation.z = 0.0
        marker_data.pose.orientation.w = 1.0
        marker_data.scale.x = self.body_r
        marker_data.scale.y = self.body_r
        marker_data.scale.z = 0.1
        marker_data.color.a = 0.5
        marker_data.color.r = 1.0
        marker_data.color.g = 1.0
        marker_data.color.b = 1.0
        self.quadrotor_pos_pub.publish(marker_data)

    def start_callback(self, data):
        self.publish_commands = True

    def depth_visualize(self, depth):
        depth += self.body_r/self.max_detection_range
        real_depth = depth*self.max_detection_range
        return real_depth


if __name__ == '__main__':
    agile_pilot_node = PolarPixelVisualizeNode()
    rospy.spin()