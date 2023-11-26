#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math,time
from std_msgs.msg import Empty, String, Bool
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from beetle.kondo_control import KondoControl
import numpy as np
import tf

#### state classes ####

"""
Standby -> Approach -> Assembly
 ^            |          |
 | _ _ _ _ _ _|_ _ _ _ _ |
                
"""

class StandbyState(smach.State):
    def __init__(self,
                 robot_name = 'beetle1',
                 robot_id = 1,
                 male_servo_id = 5,
                 female_servo_id = 6,
                 real_machine = False,
                 unlock_servo_angle_male = 7000,
                 lock_servo_angle_male = 8800,
                 unlock_servo_angle_female = 11000,
                 lock_servo_angle_female = 5600,
                 leader = 'beetle2',
                 leader_id = 2,
                 airframe_size = 0.52,
                 x_offset = 0.1,
                 y_offset = 0,
                 z_offset = 0,
                 x_tol = 0.01,
                 y_tol = 0.01,
                 z_tol = 0.01,
                 roll_tol = 0.08,
                 pitch_tol = 0.08,
                 yaw_tol = 0.08,
                 root_fc_dis = 0.129947,
                 attach_dir = -1):

        smach.State.__init__(self, outcomes=['done','in_process','emergency'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.unlock_servo_angle_male = unlock_servo_angle_male
        self.lock_servo_angle_male = lock_servo_angle_male
        self.unlock_servo_angle_female = unlock_servo_angle_female
        self.lock_servo_angle_female = lock_servo_angle_female
        self.leader = leader
        self.leader_id = leader_id
        self.airframe_size = airframe_size
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.x_tol = x_tol
        self.y_tol = y_tol
        self.z_tol = z_tol
        self.roll_tol = roll_tol
        self.pitch_tol = pitch_tol
        self.yaw_tol = yaw_tol
        self.root_fc_dis = root_fc_dis
        self.attach_dir = attach_dir

        # flags
        self.emergency_flag = False
        self.follower_male_mech_activated = False

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # publisher
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.follower_att_pub = rospy.Publisher(self.robot_name+"/final_target_baselink_rot", DesireCoord, queue_size=10)
        if(self.attach_dir < 0):
            self.follower_docking_pub = rospy.Publisher(self.robot_name+"/docking_cmd", Bool, queue_size=10)
            self.kondo_servo = KondoControl(self.leader,self.leader_id,self.female_servo_id,self.real_machine)
        else:
            self.follower_docking_pub = rospy.Publisher(self.leader+"/docking_cmd", Bool, queue_size=10)
            self.kondo_servo = KondoControl(self.robot_name,self.robot_id,self.female_servo_id,self.real_machine)
            
        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)

        # messages
        self.docking_msg = Bool()

        # position offset while StandbyState
        if(self.attach_dir < 0):
            self.target_offset = np.array([-(self.airframe_size + self.x_offset), self.y_offset, self.z_offset]) 
        else:
            self.target_offset = np.array([(self.airframe_size + self.x_offset), self.y_offset, self.z_offset])
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol]) # position error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pitch_tol, self.yaw_tol]) # attitude error torelance

    def execute(self, userdata):
        if not self.follower_male_mech_activated:
            if self.real_machine:
                self.kondo_servo.sendTargetAngle(self.lock_servo_angle_female)
            else:
                self.docking_msg.data = True
                self.follower_docking_pub.publish(self.docking_msg)
            self.follower_male_mech_activated = True
        # calculate current follower position in leader coordinate
        #TODO: determine leader namespace dynamically
        try:
            leader_from_world = self.listener.lookupTransform('/world', self.leader+'/root', rospy.Time(0))
        except (tf.LookupException, tf_from_neighboring.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'
        try:
            follower_from_leader = self.listener.lookupTransform(self.leader+'/root', self.robot_name+'/root', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'

        # set target odom in leader coordinate
        #TODO: determine leader namespace dynamically
        self.br.sendTransform((self.target_offset[0], self.target_offset[1] , self.target_offset[2] + self.root_fc_dis),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "follower_target_odom",
                              self.leader+"/root")

        # convert target position from leader coord to world coord
        try:
            homo_transformed_target_odom = self.listener.lookupTransform('/world', '/follower_target_odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'
        target_pos = homo_transformed_target_odom[0]
        target_att = tf.transformations.euler_from_quaternion(homo_transformed_target_odom[1])

        pos_error = np.array(self.target_offset - follower_from_leader[0])
        att_error = np.array([0,0,0])-tf.transformations.euler_from_quaternion(follower_from_leader[1])
        rospy.loginfo(pos_error)
        #check if pos and att error are within the torrelance
        if np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol)):
            return 'done'
        elif self.emergency_flag:
            return 'emergency'
        else:
            # x,y,z and yaw
            nav_msg_follower = FlightNav()
            nav_msg_follower.target = 0
            nav_msg_follower.control_frame = 0
            nav_msg_follower.pos_xy_nav_mode=2
            nav_msg_follower.pos_z_nav_mode=2
            nav_msg_follower.yaw_nav_mode = 2
            nav_msg_follower.target_pos_x = target_pos[0]
            nav_msg_follower.target_pos_y = target_pos[1]
            nav_msg_follower.target_pos_z = target_pos[2]
            nav_msg_follower.target_yaw = target_att[2]
            self.follower_nav_pub.publish(nav_msg_follower)

            # roll and pitch
            link_rot_follower = DesireCoord()
            link_rot_follower.roll = target_att[0]
            link_rot_follower.pitch = target_att[1]
            # self.follower_att_pub.publish(link_rot_follower)
            return 'in_process'

    def emergencyCb(self,msg):
        nav_msg_follower = FlightNav()
        nav_msg_follower.target = 0
        nav_msg_follower.control_frame = 0
        nav_msg_follower.pos_xy_nav_mode=2
        nav_msg_follower.target_pos_x = -1.5
        nav_msg_follower.target_pos_y = 0
        self.follower_nav_pub.publish(nav_msg_follower)
        self.emergency_flag = True
        
class ApproachState(smach.State):
    def __init__(self,
                 robot_name = 'beetle1',
                 robot_id = 1,
                 male_servo_id = 5,
                 female_servo_id = 6,
                 real_machine = False,
                 unlock_servo_angle_male = 7000,
                 lock_servo_angle_male = 8800,
                 unlock_servo_angle_female = 11000,
                 lock_servo_angle_female = 5600,
                 leader = 'beetle2',
                 leader_id = 2,
                 airframe_size = 0.52,
                 x_offset = 0,
                 y_offset = 0,
                 z_offset = 0,
                 x_tol = 0.02,
                 y_tol = 0.02,
                 z_tol = 0.02,
                 roll_tol = 0.08,
                 pitch_tol = 0.08,
                 yaw_tol = 0.08,
                 root_fc_dis = 0.129947,
                 x_danger_thre = 0.02,
                 y_danger_thre = 0.1,
                 z_danger_thre = 0.1,
                 roll_danger_thre = 0.35,
                 pitch_danger_thre = 0.35,
                 yaw_danger_thre = 0.35,
                 attach_dir = -1):

        smach.State.__init__(self, outcomes=['done','in_process','fail','emergency'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.unlock_servo_angle_male = unlock_servo_angle_male
        self.lock_servo_angle_male = lock_servo_angle_male
        self.unlock_servo_angle_female = unlock_servo_angle_female
        self.lock_servo_angle_female = lock_servo_angle_female
        self.leader = leader
        self.leader_id = leader_id
        self.airframe_size = airframe_size
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.z_offset = z_offset
        self.x_tol = x_tol
        self.y_tol = y_tol
        self.z_tol = z_tol
        self.roll_tol = roll_tol
        self.pitch_tol = pitch_tol
        self.yaw_tol = yaw_tol
        self.root_fc_dis = root_fc_dis
        self.x_danger_thre = x_danger_thre
        self.y_danger_thre = y_danger_thre
        self.z_danger_thre = z_danger_thre
        self.roll_danger_thre = roll_danger_thre
        self.pitch_danger_thre = pitch_danger_thre
        self.yaw_danger_thre = yaw_danger_thre
        self.attach_dir = attach_dir

        # flags
        self.emergency_flag = False        

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # publisher
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.follower_att_pub = rospy.Publisher(self.robot_name+"/final_target_baselink_rot", DesireCoord, queue_size=10)

        # subscriber
        self.emergency_stop_sub = rospy.Subscriber("/emergency_assembly_interuption",Empty,self.emergencyCb)        

        # position offset while ApproachState
        if(self.attach_dir < 0):
            self.target_offset = np.array([-(self.airframe_size + self.x_offset), self.y_offset, self.z_offset]) 
        else:
            self.target_offset = np.array([(self.airframe_size + self.x_offset), self.y_offset, self.z_offset])
        # position error torelance
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol])
        # attitude error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pitch_tol, self.yaw_tol])
        # position danger threshold
        self.pos_danger_thre = np.array([self.x_danger_thre, self.y_danger_thre, self.z_danger_thre]) 
        # attitude danger threshold
        self.att_danger_thre = np.array([self.roll_danger_thre, self.pitch_danger_thre, self.yaw_danger_thre]) 

    def execute(self, userdata):
        # calculate now follower position in leader coordinate
        #TODO: determine leader namespace dynamically
        try:
            follower_from_leader = self.listener.lookupTransform(self.leader+'/root', self.robot_name+'/root', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'

        # set target odom in leader coordinate
        #TODO: determine leader namespace dynamically
        self.br.sendTransform((self.target_offset[0] + 0.05, self.target_offset[1] , self.target_offset[2] + self.root_fc_dis),
                              tf.transformations.quaternion_from_euler(0, 0, 0),
                              rospy.Time.now(),
                              "follower_target_odom",
                              self.leader+"/root")

        # convert target position from leader coord to world coord
        try:
            homo_transformed_target_odom = self.listener.lookupTransform('/world', '/follower_target_odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in process'
        target_pos = homo_transformed_target_odom[0]
        target_att = tf.transformations.euler_from_quaternion(homo_transformed_target_odom[1])

        pos_error = np.array(self.target_offset - follower_from_leader[0])
        att_error = att_error = np.array([0,0,0])-tf.transformations.euler_from_quaternion(follower_from_leader[1])

        rospy.loginfo(pos_error)

        #check if pos and att error are within the torrelance
        if np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol)):
            return 'done'
        elif self.emergency_flag:
            return 'emergency'
        elif np.all(np.greater(np.abs(pos_error),self.pos_danger_thre)) or np.all(np.greater(np.abs(att_error),self.att_danger_thre)):
            return 'fail'
        else:
            # x,y,z and yaw
            nav_msg_follower = FlightNav()
            nav_msg_follower.target = 0
            nav_msg_follower.control_frame = 0
            nav_msg_follower.pos_xy_nav_mode=2
            nav_msg_follower.pos_z_nav_mode=2
            nav_msg_follower.yaw_nav_mode = 2
            nav_msg_follower.target_pos_x = target_pos[0]
            nav_msg_follower.target_pos_y = target_pos[1]
            nav_msg_follower.target_pos_z = target_pos[2]
            nav_msg_follower.target_yaw = target_att[2]
            self.follower_nav_pub.publish(nav_msg_follower)

            # roll and pitch
            link_rot_follower = DesireCoord()
            link_rot_follower.roll = target_att[0]
            link_rot_follower.pitch = target_att[1]
            # self.follower_att_pub.publish(link_rot_follower)

            return 'in_process'

    def emergencyCb(self,msg):
        nav_msg_follower = FlightNav()
        nav_msg_follower.target = 0
        nav_msg_follower.control_frame = 0
        nav_msg_follower.pos_xy_nav_mode=2
        nav_msg_follower.target_pos_x = -1.5
        nav_msg_follower.target_pos_y = 0
        self.follower_nav_pub.publish(nav_msg_follower)
        self.emergency_flag = True

class AssemblyState(smach.State):
    def __init__(self,
                 robot_name = 'beetle1',
                 robot_id = 1,
                 male_servo_id = 5,
                 female_servo_id = 6,
                 real_machine = False,
                 unlock_servo_angle_male = 7000,
                 lock_servo_angle_male = 8800,
                 unlock_servo_angle_female = 11000,
                 lock_servo_angle_female = 5600,
                 leader = 'beetle2',
                 leader_id = 2,
                 attach_dir = -1):
        smach.State.__init__(self, outcomes=['done','emergency'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.male_servo_id = male_servo_id
        self.female_servo_id = female_servo_id
        self.real_machine = real_machine
        self.unlock_servo_angle_male = unlock_servo_angle_male
        self.lock_servo_angle_male = lock_servo_angle_male
        self.unlock_servo_angle_female = unlock_servo_angle_female
        self.lock_servo_angle_female = lock_servo_angle_female
        self.leader = leader
        self.leader_id = leader_id
        self.attach_dir = attach_dir

        # flags
        self.emergency_flag = False

        # publisher
        self.follower_docking_pub = rospy.Publisher(self.robot_name+"/docking_cmd", Bool, queue_size=10)
        self.follower_nav_pub = rospy.Publisher(self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.leader_nav_pub = rospy.Publisher(self.leader+"/uav/nav", FlightNav, queue_size=10)
        if(self.attach_dir < 0):
            self.kondo_servo = KondoControl(self.robot_name,self.robot_id,self.male_servo_id,self.real_machine)
        else:
            self.kondo_servo = KondoControl(self.leader,self.leader_id,self.male_servo_id,self.real_machine)
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
        if self.real_machine:
            self.kondo_servo.sendTargetAngle(self.lock_servo_angle_male)
        time.sleep(2.0)
        self.nav_msg.pos_xy_nav_mode= 6
        self.follower_nav_pub.publish(self.nav_msg)
        self.leader_nav_pub.publish(self.nav_msg)
        rospy.sleep(5.0)
        self.flag_msg.key = str(self.robot_id)
        self.flag_msg.value = '1'
        self.flag_pub.publish(self.flag_msg)
        self.flag_msg.key = str(self.leader_id)
        self.flag_msg.value = '1'
        self.flag_pub_leader.publish(self.flag_msg)
        return 'done'

    def emergencyCb(self,msg):
        nav_msg_follower = FlightNav()
        nav_msg_follower.target = 0
        nav_msg_follower.control_frame = 0
        nav_msg_follower.pos_xy_nav_mode=2
        nav_msg_follower.target_pos_x = -1.5
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
