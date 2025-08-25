#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math,time,logging
from std_msgs.msg import Empty, String, Bool
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import ServoControlCmd
from diagnostic_msgs.msg import KeyValue
from ninja.dynamixel_control_api import DynamixelControl
import numpy as np
import tf


#### state classes ####

"""""""""""""""""""""""""""
Switch -> Separate -> Stop
"""""""""""""""""""""""""""

class Filter(logging.Filter):
    def filter(self, record):
        return 'State machine transitioning' not in record.msg

class SwitchState(smach.State):
    # release docking mechanism and switch control mode
    def __init__(self,
                 robot_name = 'ninja1',
                 robot_id = 1,
                 male_servo_id = 8,
                 real_machine = False,
                 unlock_servo_angle_male = 2300,
                 default_servo_angle_male = 4050,
                 neighboring = 'ninja2',
                 neighboring_id = 2,
                 female_servo_id = 6,
                 separate_dir = -1):
        smach.State.__init__(self, outcomes=['done'])

        self.robot_name = robot_name 
        self.robot_id = robot_id 
        self.male_servo_id = male_servo_id
        self.real_machine = real_machine
        self.unlock_servo_angle_male = unlock_servo_angle_male
        self.default_servo_angle_male = default_servo_angle_male
        self.neighboring = neighboring
        self.neighboring_id = neighboring_id
        self.female_servo_id = female_servo_id
        self.separate_dir = separate_dir

        if(separate_dir > 0):
            # self.dynamixel_servo = DynamixelControl(self.robot_name,self.robot_id,self.female_servo_id,self.real_machine)
            self.dynamixel_servo_neighboring = DynamixelControl(self.neighboring,self.neighboring_id,self.male_servo_id,self.real_machine)
        else:
            self.dynamixel_servo = DynamixelControl(self.robot_name,self.robot_id,self.male_servo_id,self.real_machine)
            # self.dynamixel_servo_neighboring = DynamixelControl(self.neighboring,self.neighboring_id,self.female_servo_id,self.real_machine)
            
        self.flag_pub = rospy.Publisher('/' + self.robot_name + '/assembly_flag', KeyValue, queue_size = 1)
        self.flag_pub_neighboring = rospy.Publisher('/' + self.neighboring + '/assembly_flag', KeyValue, queue_size = 1)

        if(separate_dir > 0):
            self.docking_pub = rospy.Publisher('/' + self.neighboring  + '/docking_cmd', Bool, queue_size = 1)
        else:
            self.docking_pub = rospy.Publisher('/' + self.robot_name  + '/docking_cmd', Bool, queue_size = 1)

        #messeges
        self.flag_msg = KeyValue()
        self.docking_msg = Bool()
        time.sleep(0.5)

    def execute(self, userdata):
        self.flag_msg.key = str(self.robot_id)
        self.flag_msg.value = '0'
        self.flag_pub.publish(self.flag_msg)
        if self.real_machine:
            if(self.separate_dir > 0):
                self.dynamixel_servo_neighboring.sendTargetAngle(self.unlock_servo_angle_male)
            else:
                self.dynamixel_servo.sendTargetAngle(self.unlock_servo_angle_male)
        else:
            self.docking_msg.data = False
            self.docking_pub.publish(self.docking_msg)
        time.sleep(3.0)
        return 'done'

class SeparateState(smach.State):
    # keep away target robot from leader
    def __init__(self,
                 robot_name = 'ninja1',
                 robot_id = 1,
                 separate_dir = -1,
                 separate_vel = 0.2,
                 neighboring = 'ninja2',
                 target_dist_from_neighboring = 0.5):

        smach.State.__init__(self, outcomes=['done','in_process'])

        self.robot_name = robot_name
        self.robot_id = robot_id
        self.separate_dir = separate_dir
        self.separate_vel = separate_vel * self.separate_dir
        self.neighboring = neighboring
        self.target_dist_from_neighboring = target_dist_from_neighboring

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()

        # publisher
        self.nav_pub = rospy.Publisher('/' + self.robot_name+"/uav/nav", FlightNav, queue_size=10)
        self.nav_pub_neighboring = rospy.Publisher('/' + self.neighboring+"/uav/nav", FlightNav, queue_size=10)

        #messages
        self.nav_msg = FlightNav()
        
    def execute(self, userdata):
        x_dist = 0
        try:
            if self.separate_dir < 0:
                tf_from_neighboring = self.listener.lookupTransform('/' + self.neighboring+'/pitch_connect_point', '/' + self.robot_name+'/yaw_connect_point', rospy.Time(0))
                x_dist = math.fabs(tf_from_neighboring[0][0])
            else:
                tf_from_neighboring = self.listener.lookupTransform('/' + self.neighboring+'/yaw_connect_point', '/' + self.robot_name+'/pitch_connect_point', rospy.Time(0))
                x_dist = math.fabs(tf_from_neighboring[0][0])    
            rospy.loginfo("tf is fine")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x_dist = 0
            rospy.loginfo('out')
            return 'in_process'
        if x_dist <= self.target_dist_from_neighboring:
            self.nav_msg.target = 1
            if self.separate_dir < 0:
                self.nav_msg.control_frame = 3 #RIGHT_DOCK frame
            else:
                self.nav_msg.control_frame = 2 #LEFT_DOCK frame
            self.nav_msg.pos_xy_nav_mode= 1
            self.nav_msg.target_vel_x = self.separate_vel
            self.nav_pub.publish(self.nav_msg)
            return 'in_process'
        else:
            self.nav_msg.pos_xy_nav_mode= 6
            self.nav_pub.publish(self.nav_msg)
            return 'done'


#### main class ####
class DisassembleDemo():
    def __init__(self):
        rospy.init_node("disassemble_demo")

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            smach.StateMachine.add('SwitchState', SwitchState(), transitions={'done':'SeparateState'})

            smach.StateMachine.add('SeparateState', SeparateState(), transitions={'done':'succeeded','in_process':'SeparateState'})
 
        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    try:
        logging.getLogger('rosout').addFilter(Filter())
        disassemble_demo = DisassembleDemo();
        disassemble_demo.main()
    except rospy.ROSInterruptException: pass
