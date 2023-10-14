#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math,time,logging
from std_msgs.msg import Empty, String, Bool
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from spinal.msg import ServoControlCmd
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import KeyValue
from beetle.kondo_control import KondoControl
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
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])

        #TODO: change these into pamameters
        self.robot_name = 'beetle1'
        self.robot_id = 1
        self.servo_id = 1
        self.real_machine = False
        self.unlock_servo_angle_male = 7000
        self.lock_servo_angle_male = 3700
        self.unlock_servo_angle_female = 7000 #todo
        self.lock_servo_angle_female = 3700 #todo
        self.neighboring = 'beetle2'
        self.neighboring_id = 2

        self.kondo_servo = KondoControl(self.robot_name,self.robot_id,self.servo_id,self.real_machine)

        self.flag_pub = rospy.Publisher('/' + self.robot_name + '/assembly_flag', KeyValue, queue_size = 1)
        self.flag_pub_neighboring = rospy.Publisher('/' + self.neighboring + '/assembly_flag', KeyValue, queue_size = 1)
        self.docking_pub = rospy.Publisher('/' + self.robot_name  + '/docking_cmd', Bool, queue_size = 1)
        #messeges
        self.flag_msg = KeyValue()
        self.docking_msg = Bool()
        time.sleep(0.5)

    def execute(self, userdata):
        self.flag_msg.key = str(self.robot_id)
        self.flag_msg.value = '0'
        self.flag_pub.publish(self.flag_msg)
        self.flag_msg.key = str(self.neighboring_id)
        self.flag_msg.value = '0'
        self.flag_pub_neighboring.publish(self.flag_msg)
        if self.real_machine:
            self.kondo_servo.sendTargetAngle(self.unlock_servo_angle_male)
        else:
            self.docking_msg.data = False
            self.docking_pub.publish(self.docking_msg)
        time.sleep(3.0)
        return 'done'

class SeparateState(smach.State):
    # keep away target robot from leader
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','in_process'])

        #TODO: change these into pamameters
        self.robot_name = 'beetle1'
        self.robot_id = 1
        self.separate_vel = -0.2
        self.neighboring = 'beetle2'
        self.target_dist_from_neighboring = 1.5

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
            tf_from_neighboring = self.listener.lookupTransform('/' + self.neighboring+'/root', '/' + self.robot_name+'/root', rospy.Time(0))
            x_dist = math.fabs(tf_from_neighboring[0][0])
            rospy.loginfo("tf is fine")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            x_dist = 0
            rospy.loginfo('out')
            return 'in_process'
        if x_dist <= self.target_dist_from_neighboring:
            self.nav_msg.target = 1
            self.nav_msg.control_frame = 1 #local frame
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
