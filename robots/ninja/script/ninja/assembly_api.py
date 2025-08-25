#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math,time
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import TwistStamped
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from ninja.dynamixel_control_api import DynamixelControl
from ninja.utils import coordTransformer
import numpy as np
import tf

#### state classes ####

"""
Idle -> Standby -> Approach -> Assembly
 ^            |          |
 | _ _ _ _ _ _|_ _ _ _ _ |
                
"""

class IdleState(smach.State):
    def __init__(self,
                 run_rate = 40):

        smach.State.__init__(self, outcomes=['idle','start'])

        self.run_rate = rospy.Rate(run_rate)
        self.motion_start_flag = False

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/motion_start", Empty, self.motionStartCb)

    def execute(self, userdata):
        if self.motion_start_flag:
            return 'start'
        else:
            self.run_rate.sleep()
            return 'idle'

    def motionStartCb(self,msg):
        self.motion_start_flag = True

class StandbyState(smach.State):
    def __init__(self,
                 robot_name = 'ninja1',
                 robot_id = 1,
                 male_servo_id = 8,
                 female_servo_id = 6,
                 real_machine = True,
                 standby_servo_angle_male = 4050,
                 leader = 'ninja2',
                 leader_id = 2,
                 x_offset = 0.12,
                 y_offset = 0,
                 z_offset = 0,
                 x_tol = 0.02,
                 y_tol = 0.02,
                 z_tol = 0.01,
                 roll_tol = 0.05,
                 pitch_tol = 0.05,
                 yaw_tol = 0.08,
                 attach_dir = -1.0,
                 approach_mode = 'nav',
                 run_rate = 40,
                 left_cp_name = 'pitch_connect_point',
                 right_cp_name = 'yaw_connect_point'):

        smach.State.__init__(self, outcomes=['done','in_process','emergency'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.standby_servo_angle_male = standby_servo_angle_male
        self.leader = leader
        self.leader_id = leader_id
        self.leader_target_pose = []
        self.leader_pre_target_pose = []
        self.target_offset = np.zeros(4)
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.x_tol = x_tol
        self.y_tol = y_tol
        self.z_tol = z_tol
        self.roll_tol = roll_tol
        self.pitch_tol = pitch_tol
        self.yaw_tol = yaw_tol
        self.attach_dir = attach_dir
        self.target_cog_pos = np.zeros(3)
        self.target_att = np.zeros(3)
        self.approach_mode = approach_mode
        self.run_rate = rospy.Rate(run_rate)
        if attach_dir < 0:
            self.follower_cp_name = right_cp_name
            self.leader_cp_name = left_cp_name
        else:
            self.follower_cp_name = left_cp_name
            self.leader_cp_name = right_cp_name
        # flags
        self.emergency_flag = False
        self.follower_male_mech_activated = False

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # publisher
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.follower_traj_pub = rospy.Publisher(self.robot_name+"/target_pose", PoseStamped, queue_size=10)
        # self.follower_att_pub = rospy.Publisher(self.robot_name+"/final_target_baselink_rot", DesireCoord, queue_size=10)
        if(self.attach_dir < 0):
            self.follower_docking_pub = rospy.Publisher(self.robot_name+"/docking_cmd", Bool, queue_size=10)
            self.dynamixel_servo = DynamixelControl(self.robot_name,self.robot_id,self.male_servo_id,self.real_machine)
        else:
            self.follower_docking_pub = rospy.Publisher(self.leader+"/docking_cmd", Bool, queue_size=10)
            self.dynamixel_servo = DynamixelControl(self.leader,self.leader_id,self.male_servo_id,self.real_machine)            

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)
        self.leader_target_pose_sub = rospy.Subscriber(self.leader + "/debug/pose/pid" ,PoseControlPid, self.leaderTargetPoseCb)

        # messages
        self.docking_msg = Bool()

        # utils
        self.coordTransformer = coordTransformer()

        # position offset while StandbyState
        if(self.attach_dir < 0):
            self.target_offset = np.array([-self.x_offset, self.y_offset, self.z_offset, 1]) 
        else:
            self.target_offset = np.array([self.x_offset, self.y_offset, self.z_offset, 1])
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol]) # position error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pitch_tol, self.yaw_tol]) # attitude error torelance

    def execute(self, userdata):
        if not self.follower_male_mech_activated:
            if self.real_machine:
                self.dynamixel_servo.sendTargetAngle(self.standby_servo_angle_male)
            else:
                self.docking_msg.data = True
                self.follower_docking_pub.publish(self.docking_msg)
            self.follower_male_mech_activated = True

        # calculate current follower position in leader coordinate
        try:
            follower_from_leader = self.listener.lookupTransform(self.leader+'/'+self.leader_cp_name, self.robot_name+'/'+self.follower_cp_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'

        # set target odom in leader coordinate
        #TODO: determine leader namespace dynamically
        self.br.sendTransform((self.target_offset[0], self.target_offset[1], self.target_offset[2]),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "follower_target_odom",
                              self.leader+'/'+self.leader_cp_name)

        self.br.sendTransform((self.target_cog_pos[0], self.target_cog_pos[1], self.target_cog_pos[2]),
                              tf.transformations.quaternion_from_euler(self.target_att[0], self.target_att[1], self.target_att[2]),
                             rospy.Time.now(),
                              "follower_target_cog",
                              "world")

        pos_error = np.array(self.target_offset[:3] - follower_from_leader[0])
        if(pos_error[0] * self.attach_dir > 0):
            pos_error[0] = 0.0
        att_error = np.array([0,0,0])-tf.transformations.euler_from_quaternion(follower_from_leader[1])

        rospy.loginfo(pos_error)
        rospy.loginfo(att_error)

        #check if pos and att error are within the torrelance
        if np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol)):
            return 'done'
        elif self.emergency_flag:
            return 'emergency'
        else:
            nav_msg_follower = FlightNav()
            nav_msg_follower.target = 1
            nav_msg_follower.control_frame = 0
            nav_msg_follower.pos_xy_nav_mode=2
            nav_msg_follower.pos_z_nav_mode=2
            nav_msg_follower.yaw_nav_mode = 2
            nav_msg_follower.target_pos_x = self.target_cog_pos[0]
            nav_msg_follower.target_pos_y = self.target_cog_pos[1]
            nav_msg_follower.target_pos_z = self.target_cog_pos[2]
            nav_msg_follower.target_yaw = self.target_att[2]

            traj_msg_follower = PoseStamped()
            traj_msg_follower.header.stamp = rospy.Time.now()
            traj_msg_follower.pose.position.x = self.target_cog_pos[0]
            traj_msg_follower.pose.position.y = self.target_cog_pos[1]
            traj_msg_follower.pose.position.z = self.target_cog_pos[2]
            target_att_qr = tf.transformations.quaternion_from_euler(self.target_att[0], self.target_att[1], self.target_att[2])
            traj_msg_follower.pose.orientation.x = target_att_qr[0]
            traj_msg_follower.pose.orientation.y = target_att_qr[1]
            traj_msg_follower.pose.orientation.z = target_att_qr[2]
            traj_msg_follower.pose.orientation.w = target_att_qr[3]

            if self.leader_target_pose != self.leader_pre_target_pose:
                self.leader_pre_target_pose = self.leader_target_pose
                if(self.approach_mode == 'nav'):
                    self.follower_nav_pub.publish(nav_msg_follower)
                elif(self.approach_mode == 'trajectory'):
                    self.follower_traj_pub.publish(traj_msg_follower)
                else:
                    rospy.logerr("Invalid approach mode is setted")
                    return 'fail'

            # roll and pitch
            link_rot_follower = DesireCoord()
            link_rot_follower.roll = self.target_att[0]
            link_rot_follower.pitch = self.target_att[1]
            # self.follower_att_pub.publish(link_rot_follower)
            self.run_rate.sleep()
            return 'in_process'

    def emergencyCb(self,msg):
        nav_msg_follower = FlightNav()
        nav_msg_follower.target = 0
        nav_msg_follower.control_frame = 0
        nav_msg_follower.pos_xy_nav_mode=2
        if self.attach_dir < 0:
            nav_msg_follower.target_pos_x = -1.0
        else:
            nav_msg_follower.target_pos_x = 1.0
        nav_msg_follower.target_pos_y = 0
        self.follower_nav_pub.publish(nav_msg_follower)
        self.emergency_flag = True

    def leaderTargetPoseCb(self, msg):
        leader_target_x = msg.x.target_p
        leader_target_y = msg.y.target_p
        leader_target_z = msg.z.target_p
        leader_target_yaw = msg.yaw.target_p
        self.leader_target_pose = [leader_target_x, leader_target_y, leader_target_z, leader_target_yaw]
        # calculate target position of follower's docking mechanism.
        leader_cog_2_world_home = self.coordTransformer.getHomoMatFromVector([leader_target_x, leader_target_y, leader_target_z], [0,0,leader_target_yaw])
        leader_dock_point_2_cog_homo = np.linalg.inv(self.coordTransformer.getHomoMatFromCoordName(self.leader+'/'+ self.leader_cp_name, self.leader+'/'+ 'cog'))
        target_cp_pos = (leader_cog_2_world_home @ leader_dock_point_2_cog_homo @ self.target_offset)[:3]
        # calculate target pose of follower's docking mechanism.
        relative_pose_of_cp = np.eye(4)
        relative_pose_of_cp[:3,3] = self.target_offset[:3]
        rotation_matrix = tf.transformations.euler_matrix(0,0,0)
        relative_pose_of_cp[:3, :3] = rotation_matrix[:3, :3]
        target_cp_pose = leader_cog_2_world_home @ leader_dock_point_2_cog_homo @ relative_pose_of_cp
        target_cp_pos = target_cp_pose[:3,3]
        target_cp_att = np.array(tf.transformations.euler_from_matrix(target_cp_pose[:3,:3]))

        follower_dock_point_2_cog_homo = self.coordTransformer.getHomoMatFromCoordName(self.robot_name+'/'+self.follower_cp_name, self.robot_name+'/'+'cog')
        target_cog_pose = target_cp_pose @ follower_dock_point_2_cog_homo
        self.target_att = np.array(tf.transformations.euler_from_matrix(target_cog_pose[:3,:3]))
        # rospy.loginfo(self.target_att)
        self.target_cog_pos = target_cog_pose[:3,3]

class ApproachState(smach.State):
    def __init__(self,
                 robot_name = 'ninja1',
                 robot_id = 1,
                 male_servo_id = 8,
                 female_servo_id = 6,
                 real_machine = True,
                 leader = 'ninja2',
                 leader_id = 2,
                 contact_vel_x = 0.1,
                 contact_vel_y = 0,
                 contact_vel_z = 0,
                 x_offset = 0.0,
                 y_offset = 0.0,
                 z_offset = 0.0,
                 x_tol = 0.02,
                 y_tol = 0.3,
                 z_tol = 0.3,
                 # x_tol = 1.0,
                 # y_tol = 1.0,
                 # z_tol = 1.0,
                 roll_tol = 0.08,
                 pitch_tol = 0.08,
                 yaw_tol = 0.5,
                 x_danger_thre = 0.02,
                 y_danger_thre = 0.1,
                 z_danger_thre = 0.1,
                 roll_danger_thre = 0.35,
                 pitch_danger_thre = 0.35,
                 yaw_danger_thre = 0.35,
                 attach_dir = -1.0,
                 approach_mode = 'nav',
                 run_rate = 40,
                 left_cp_name = 'pitch_connect_point',
                 right_cp_name = 'yaw_connect_point'):

        smach.State.__init__(self, outcomes=['done','in_process','fail','emergency'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.leader = leader
        self.leader_id = leader_id
        self.leader_pre_target_pose = []
        self.target_offset = np.zeros(4)
        self.contact_vel_x = contact_vel_x
        self.contact_vel_y = contact_vel_y
        self.contact_vel_z = contact_vel_z
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.x_tol = x_tol
        self.y_tol = y_tol
        self.z_tol = z_tol
        self.roll_tol = roll_tol
        self.pitch_tol = pitch_tol
        self.yaw_tol = yaw_tol
        self.x_danger_thre = x_danger_thre
        self.y_danger_thre = y_danger_thre
        self.z_danger_thre = z_danger_thre
        self.roll_danger_thre = roll_danger_thre
        self.pitch_danger_thre = pitch_danger_thre
        self.yaw_danger_thre = yaw_danger_thre
        self.attach_dir = attach_dir
        self.target_cog_pos = np.zeros(3)
        self.target_att = np.zeros(3)
        self.approach_mode = approach_mode
        self.run_rate = rospy.Rate(run_rate)
        if attach_dir < 0:
            self.follower_cp_name = right_cp_name
            self.leader_cp_name = left_cp_name
        else:
            self.follower_cp_name = left_cp_name
            self.leader_cp_name = right_cp_name

        # flags
        self.emergency_flag = False
        self.force_switching_flag = False

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # publisher
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.follower_traj_pub = rospy.Publisher(self.robot_name+"/target_pose", PoseStamped, queue_size=10)
        # self.follower_att_pub = rospy.Publisher(self.robot_name+"/final_target_baselink_rot", DesireCoord, queue_size=10)
        self.leader_nav_pub = rospy.Publisher(self.leader+"/uav/nav", FlightNav, queue_size=10)
        self.assembly_nav_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
        self.follower_twist_pub = rospy.Publisher(self.robot_name+"/target_velocity_twist",TwistStamped,queue_size=1)
        self.leader_twist_pub = rospy.Publisher(self.leader+"/target_velocity_twist",TwistStamped,queue_size=1)
        
        if(self.attach_dir < 0):
            self.follower_docking_pub = rospy.Publisher(self.robot_name+"/docking_cmd", Bool, queue_size=10)
            self.dynamixel_servo = DynamixelControl(self.robot_name,self.robot_id,self.male_servo_id,self.real_machine)
        else:
            self.follower_docking_pub = rospy.Publisher(self.leader+"/docking_cmd", Bool, queue_size=10)
            self.dynamixel_servo = DynamixelControl(self.leader,self.leader_id,self.male_servo_id,self.real_machine)

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)
        self.force_switching_sub = rospy.Subscriber("/force_switching",Empty,self.forceSwitchCb)
        self.leader_target_pose_sub = rospy.Subscriber(self.leader + "/debug/pose/pid" ,PoseControlPid, self.leaderTargetPoseCb)

        # utils
        self.coordTransformer = coordTransformer()

        # position offset while ApproachState
        if(self.attach_dir < 0):
            self.target_offset = np.array([-self.x_offset, self.y_offset, self.z_offset,1.0])
        else:
            self.target_offset = np.array([self.x_offset, self.y_offset, self.z_offset,1.0])
        # position error torelance
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol])
        # attitude error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pitch_tol, self.yaw_tol])
        # position danger threshold
        self.pos_danger_thre = np.array([self.x_danger_thre, self.y_danger_thre, self.z_danger_thre]) 
        # attitude danger threshold
        self.att_danger_thre = np.array([self.roll_danger_thre, self.pitch_danger_thre, self.yaw_danger_thre]) 

    def execute(self, userdata):
        if self.real_machine:
            self.dynamixel_servo.torqueEnable(0)
        # calculate now follower position in leader coordinate
        #TODO: determine leader namespace dynamically
        try:
            follower_from_leader = self.listener.lookupTransform(self.leader+'/'+self.leader_cp_name, self.robot_name+'/'+self.follower_cp_name, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'

        # set target odom in leader coordinate
        #TODO: determine leader namespace dynamically
        self.br.sendTransform((self.target_offset[0], self.target_offset[1], self.target_offset[2]),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "follower_target_odom",
                              self.leader+"/"+ self.leader_cp_name)

        self.br.sendTransform((self.target_cog_pos[0], self.target_cog_pos[1], self.target_cog_pos[2]),
                              tf.transformations.quaternion_from_euler(self.target_att[0], self.target_att[1], self.target_att[2]),
                              rospy.Time.now(),
                              "follower_target_cog",
                              "world")

        vel_cog_f = self.coordTransformer.transform_vector(
            from_frame=f"{self.robot_name}/cog",
            to_frame=f"{self.robot_name}/{self.follower_cp_name}",
            vec=[ self.contact_vel_x * self.attach_dir * -1.0,
                  0.0,
                  0.0 ]
        )

        vel_cog_l = self.coordTransformer.transform_vector(
            from_frame=f"{self.leader}/cog",
            to_frame=f"{self.leader}/{self.leader_cp_name}",
            vec=[ self.contact_vel_x * self.attach_dir,
                  0.0,
                  0.0 ]
        )


        pos_error = np.array(self.target_offset[:3] - follower_from_leader[0])
        att_error = np.array([0,0,0])-tf.transformations.euler_from_quaternion(follower_from_leader[1])

        #check if pos and att error are within the torrelance
        if (np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol))) or self.force_switching_flag:
        
            return 'done'
        elif self.emergency_flag:
            return 'emergency'
        elif np.all(np.greater(np.abs(pos_error),self.pos_danger_thre)) or np.all(np.greater(np.abs(att_error),self.att_danger_thre)):
            return 'fail'
        else:
            #follower
            # nav_msg_follower = FlightNav()
            # nav_msg_follower.target = 1
            # nav_msg_follower.control_frame = 0
            # nav_msg_follower.pos_xy_nav_mode=2
            # nav_msg_follower.pos_z_nav_mode=2
            # nav_msg_follower.yaw_nav_mode = 2
            # nav_msg_follower.target_pos_x = self.target_cog_pos[0]
            # nav_msg_follower.target_pos_y = self.target_cog_pos[1]
            # nav_msg_follower.target_pos_z = self.target_cog_pos[2]
            # nav_msg_follower.target_yaw = self.target_att[2]

            #follower
            nav_msg_follower = FlightNav()
            nav_msg_follower.target = 1
            nav_msg_follower.control_frame = 1
            nav_msg_follower.pos_xy_nav_mode=1
            # nav_msg_follower.target_vel_x = self.contact_vel_x * self.attach_dir*(-1.0)
            nav_msg_follower.target_vel_x = float(vel_cog_f[0])
            nav_msg_follower.target_vel_y = float(vel_cog_f[1])
            #leader
            nav_msg_leader = FlightNav()
            nav_msg_leader.target = 1
            nav_msg_leader.control_frame = 1
            nav_msg_leader.pos_xy_nav_mode=1
            # nav_msg_leader.target_vel_x = self.contact_vel_x * self.attach_dir
            nav_msg_leader.target_vel_x = float(vel_cog_l[0])
            nav_msg_leader.target_vel_y = float(vel_cog_l[1])
            #TODO:assembly
            nav_msg_assembly = FlightNav()
            if self.attach_dir < 0:
                nav_msg_assembly.target = 2
            else:
                nav_msg_assembly.target = 3
            nav_msg_assembly.control_frame = 1
            nav_msg_assembly.pos_xy_nav_mode=1
            nav_msg_assembly.target_vel_x = self.contact_vel_x * self.attach_dir


            leader_twist_msg = TwistStamped()
            leader_twist_msg.header.stamp = rospy.Time.now()
            leader_twist_msg.header.frame_id = f"{self.leader}/cog"
            leader_twist_msg.twist.linear.x = vel_cog_l[0]
            leader_twist_msg.twist.linear.y = vel_cog_l[1]
            leader_twist_msg.twist.linear.z = vel_cog_l[2]
            leader_twist_msg.twist.angular.x = 0.0
            leader_twist_msg.twist.angular.y = 0.0
            leader_twist_msg.twist.angular.z = 0.0
            self.leader_twist_pub.publish(leader_twist_msg)

            follower_twist_msg = TwistStamped()
            follower_twist_msg.header.stamp = rospy.Time.now()
            follower_twist_msg.header.frame_id = f"{self.robot_name}/cog"
            follower_twist_msg.twist.linear.x = vel_cog_f[0]
            follower_twist_msg.twist.linear.y = vel_cog_f[1]
            follower_twist_msg.twist.linear.z = vel_cog_f[2]
            follower_twist_msg.twist.angular.x = 0.0
            follower_twist_msg.twist.angular.y = 0.0
            follower_twist_msg.twist.angular.z = 0.0
            self.follower_twist_pub.publish(follower_twist_msg)

            # traj_msg_follower = PoseStamped()
            # traj_msg_follower.header.stamp = rospy.Time.now()
            # traj_msg_follower.pose.position.x = self.target_cog_pos[0]
            # traj_msg_follower.pose.position.y = self.target_cog_pos[1]
            # traj_msg_follower.pose.position.z = self.target_cog_pos[2]
            # target_att_qr = tf.transformations.quaternion_from_euler(self.target_att[0], self.target_att[1], self.target_att[2])
            # traj_msg_follower.pose.orientation.x = target_att_qr[0]
            # traj_msg_follower.pose.orientation.y = target_att_qr[1]
            # traj_msg_follower.pose.orientation.z = target_att_qr[2]
            # traj_msg_follower.pose.orientation.w = target_att_qr[3]

            # if self.leader_target_pose != self.leader_pre_target_pose:
            #     self.leader_pre_target_pose = self.leader_target_pose
            #     if(self.approach_mode == 'nav'):
            #         self.follower_nav_pub.publish(nav_msg_follower)
            #     elif(self.approach_mode == 'trajectory'):
            #         self.follower_traj_pub.publish(traj_msg_follower)
            #     else:
            #         rospy.logerr("Invalid approach mode is setted")
            #         return 'fail'
            self.follower_nav_pub.publish(nav_msg_follower)
            self.leader_nav_pub.publish(nav_msg_leader)
            self.assembly_nav_pub.publish(nav_msg_leader)

            # roll and pitch
            link_rot_follower = DesireCoord()
            link_rot_follower.roll = self.target_att[0]
            link_rot_follower.pitch = self.target_att[1]
            # self.follower_att_pub.publish(link_rot_follower)
            self.run_rate.sleep()
            return 'in_process'

    def emergencyCb(self,msg):
        #follower
        nav_msg_follower = FlightNav()
        nav_msg_follower.target = 0
        nav_msg_follower.control_frame = 0
        nav_msg_follower.pos_xy_nav_mode=2
        if self.attach_dir < 0:
            nav_msg_follower.target_pos_x = -1.0
        else:
            nav_msg_follower.target_pos_x = 1.0
        nav_msg_follower.target_pos_y = 0
        #leader
        nav_msg_leader = FlightNav()
        nav_msg_leader.pos_xy_nav_mode= 6
        
        self.follower_nav_pub.publish(nav_msg_follower)
        self.leader_nav_pub.publish(nav_msg_leader)
        self.emergency_flag = True

    def forceSwitchCb(self,msg):
        self.force_switching_flag = True

    def leaderTargetPoseCb(self, msg):
        leader_target_x = msg.x.target_p
        leader_target_y = msg.y.target_p
        leader_target_z = msg.z.target_p
        leader_target_yaw = msg.yaw.target_p
        self.leader_target_pose = [leader_target_x, leader_target_y, leader_target_z, leader_target_yaw]
        # calculate target position of follower's docking mechanism.
        leader_cog_2_world_home = self.coordTransformer.getHomoMatFromVector([leader_target_x, leader_target_y, leader_target_z], [0,0,leader_target_yaw])
        leader_dock_point_2_cog_homo = np.linalg.inv(self.coordTransformer.getHomoMatFromCoordName(self.leader+'/'+ self.leader_cp_name, self.leader+'/'+ 'cog'))
        target_cp_pos = (leader_cog_2_world_home @ leader_dock_point_2_cog_homo @ self.target_offset)[:3]
        # calculate target pose of follower's docking mechanism.
        relative_pose_of_cp = np.eye(4)
        relative_pose_of_cp[:3,3] = self.target_offset[:3]
        rotation_matrix = tf.transformations.euler_matrix(0,0,0)
        relative_pose_of_cp[:3, :3] = rotation_matrix[:3, :3]
        target_cp_pose = leader_cog_2_world_home @ leader_dock_point_2_cog_homo @ relative_pose_of_cp
        target_cp_pos = target_cp_pose[:3,3]
        target_cp_att = np.array(tf.transformations.euler_from_matrix(target_cp_pose[:3,:3]))

        follower_dock_point_2_cog_homo = self.coordTransformer.getHomoMatFromCoordName(self.robot_name+'/'+self.follower_cp_name, self.robot_name+'/'+'cog')
        target_cog_pose = target_cp_pose @ follower_dock_point_2_cog_homo
        self.target_att = np.array(tf.transformations.euler_from_matrix(target_cog_pose[:3,:3]))
        # rospy.loginfo(self.target_att)
        self.target_cog_pos = target_cog_pose[:3,3]

class AssemblyState(smach.State):
    def __init__(self,
                 robot_name = 'ninja1',
                 robot_id = 1,
                 male_servo_id = 8,
                 female_servo_id = 6,
                 real_machine = True,
                 leader = 'ninja2',
                 leader_id = 2,
                 attach_dir = -1.0):
        smach.State.__init__(self, outcomes=['done','emergency'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.leader = leader
        self.leader_id = leader_id
        self.attach_dir = attach_dir

        # flags
        self.emergency_flag = False

        # publisher
        self.follower_docking_pub = rospy.Publisher(self.robot_name+"/docking_cmd", Bool, queue_size=10)
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.leader_nav_pub = rospy.Publisher(self.leader+"/uav/nav", FlightNav, queue_size=10)
        self.assembly_nav_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
        self.flag_pub = rospy.Publisher('/' + self.robot_name + '/assembly_flag', KeyValue, queue_size = 1)
        self.flag_pub_leader = rospy.Publisher('/' + self.leader + '/assembly_flag', KeyValue, queue_size = 1)

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)

        #messeges
        self.flag_msg = KeyValue()
        self.docking_msg = Bool()
        #messages
        self.nav_msg = FlightNav()
        time.sleep(0.5)

    def execute(self, userdata):
        if self.emergency_flag:
            return 'emergency'
        if not self.real_machine:
            rospy.sleep(1.0)
        self.nav_msg.pos_xy_nav_mode= 6
        self.follower_nav_pub.publish(self.nav_msg)
        self.leader_nav_pub.publish(self.nav_msg)
        self.assembly_nav_pub.publish(self.nav_msg)
        rospy.sleep(0.2)
        self.flag_msg.key = str(self.robot_id)
        self.flag_msg.value = '1'
        self.flag_pub.publish(self.flag_msg)
        self.flag_msg.key = str(self.leader_id)
        self.flag_msg.value = '1'
        self.flag_pub_leader.publish(self.flag_msg)
        rospy.sleep(1.0)
        return 'done'

    def emergencyCb(self,msg):
        nav_msg_follower = FlightNav()
        nav_msg_follower.target = 0
        nav_msg_follower.control_frame = 0
        nav_msg_follower.pos_xy_nav_mode=2
        if self.attach_dir < 0:
            nav_msg_follower.target_pos_x = -1.0
        else:
            nav_msg_follower.target_pos_x = 1.0
        nav_msg_follower.target_pos_y = 0
        self.follower_nav_pub.publish(nav_msg_follower)
        self.emergency_flag = True

#### main class ####
class AssembleDemo():
    def __init__(self):
        rospy.init_node("assemble_demo")

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded','interupted'])
        with sm_top:
            smach.StateMachine.add('StandbyState', StandbyState(), transitions={'done':'ApproachState', 'in_process':'StandbyState', 'emergency':'interupted'})
            smach.StateMachine.add('ApproachState', ApproachState(), transitions={'done':'AssemblyState', 'in_process':'ApproachState', 'fail':'StandbyState', 'emergency':'interupted'})
            smach.StateMachine.add('AssemblyState', AssemblyState(), transitions={'done':'succeeded', 'emergency':'interupted'})
 
        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    try:
        assemble_demo = AssembleDemo();
        assemble_demo.main()
    except rospy.ROSInterruptException: pass
