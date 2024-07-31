#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
from std_msgs.msg import Empty, String, Bool
from aerial_robot_msgs.msg import FlightNav
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

#### state classes ####

"""
Contact -> Standby -> Approach -> Assembly
             ^            |          |
             | _ _ _ _ _ _|_ _ _ _ _ |      
"""
class ContactState(smach.State):
    def __init__(self, edge_module):
        smach.State.__init__(self, outcomes=['done','in_process'])
        self.edge_module = edge_module

        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        # publisher
        self.edge_nav_pub = rospy.Publisher(self.edge_module+"/uav/nav", FlightNav, queue_size=10)
        self.edge_att_pub = rospy.Publisher(self.edge_module+"/final_target_baselink_rot", DesireCoord, queue_size=10)
        # parameters
        self.target_x = 3.3
        self.target_y = 1.5
        self.target_z = 1.0
        self.target_roll = 0
        self.target_pitch = 0.5
        self.target_yaw = 0
        self.x_tol = 0.23
        self.y_tol = 0.23
        self.z_tol = 0.01
        self.roll_tol = 0.08
        self.pich_tol = 0.08
        self.yaw_tol = 0.08
        self.target_pos = np.array([self.target_x, self.target_y, self.target_z])
        self.target_att = np.array([self.target_roll, self.target_pitch, self.target_yaw])
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol]) # position error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pich_tol, self.yaw_tol]) # attitude error torelance

    def execute(self, userdata):
        try:
            baselink_from_world = self.listener.lookupTransform('/world', self.edge_module+'/root', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'
        try:
            edge_from_world = self.listener.lookupTransform('/world', self.edge_module+'/contact_point', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'
        pos_error = np.array(self.target_pos - edge_from_world[0])
        rospy.loginfo(pos_error)
        att_error = np.array(self.target_att)-tf.transformations.euler_from_quaternion(baselink_from_world[1])

        if np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol)):
            return 'done'
        else:
            # x,y,z and yaw
            nav_msg = FlightNav()
            nav_msg.target = 2
            nav_msg.control_frame = 0
            nav_msg.pos_xy_nav_mode=2
            nav_msg.pos_z_nav_mode=2
            nav_msg.yaw_nav_mode = 2
            nav_msg.target_pos_x = self.target_pos[0]
            nav_msg.target_pos_y = self.target_pos[1]
            nav_msg.target_pos_z = self.target_pos[2]
            nav_msg.target_yaw = self.target_att[2]
            self.edge_nav_pub.publish(nav_msg)

            # roll and pitch
            link_rot = DesireCoord()
            link_rot.roll = self.target_att[0]
            link_rot.pitch = self.target_att[1]
            self.edge_att_pub.publish(link_rot)

            return 'in_process'

class StandbyState(smach.State):
    def __init__(self, leader, follower):
        smach.State.__init__(self, outcomes=['done','in_process'])
        #rosparams
        self.leader =  leader
        self.follower =  follower
        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        # publisher
        self.follower_nav_pub = rospy.Publisher(self.follower+"/uav/nav", FlightNav, queue_size=10)
        self.follower_att_pub = rospy.Publisher(self.follower+"/final_target_baselink_rot", DesireCoord, queue_size=10)
        # parameters
        self.frame_size = 0.52
        self.x_offset = 0.02
        self.y_offset = 0
        self.z_offset = 0
        self.x_tol = 0.005
        self.y_tol = 0.005
        self.z_tol = 0.005
        self.roll_tol = 0.08
        self.pich_tol = 0.08
        self.yaw_tol = 0.08
        self.root_fc_dis = 0.129947 #set from urdf
        self.target_offset = np.array([-(self.frame_size + self.x_offset), self.y_offset, self.z_offset]) # position offset while StandbyState
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol]) # position error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pich_tol, self.yaw_tol]) # attitude error torelance

    def execute(self, userdata):
        # rospy.loginfo('Executing state STATE1')
        # rospy.sleep(2.0)

        # calculate now follower position in leader coordinate
        #TODO: determine leader namespace dynamically
        try:
            leader_from_world = self.listener.lookupTransform('/world', self.leader+'/root', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 'in_process'
        try:
            follower_from_leader = self.listener.lookupTransform(self.leader+'/root', self.follower+'/root', rospy.Time(0))
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

        #check if pos and att error are within the torrelance
        if np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol)):
            return 'done'
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
            self.follower_att_pub.publish(link_rot_follower)

            return 'in_process'

class ApproachState(smach.State):
    def __init__(self,leader,follower):
        smach.State.__init__(self, outcomes=['done','in_process','fail'])
        # rosparam
        self.leader =  leader
        self.follower =  follower
        # tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        # publisher
        self.follower_nav_pub = rospy.Publisher(self.follower+"/uav/nav", FlightNav, queue_size=10)
        self.follower_att_pub = rospy.Publisher(self.follower+"/final_target_baselink_rot", DesireCoord, queue_size=10)
        # parameters
        self.frame_size = 0.52
        self.x_offset = 0
        self.y_offset = 0
        self.z_offset = 0
        self.x_tol = 0.02
        self.y_tol = 0.1
        self.z_tol = 0.1
        self.roll_tol = 0.5
        self.pich_tol = 0.5
        self.yaw_tol = 0.5
        self.root_fc_dis = 0.129947
        self.x_danger_thre = 0.02
        self.y_danger_thre = 0.05
        self.z_danger_thre = 0.05
        self.roll_danger_thre = 0.35
        self.pich_danger_thre = 0.35
        self.yaw_danger_thre = 0.35
        self.target_offset = np.array([-(self.frame_size + self.x_offset), self.y_offset, self.z_offset]) # position offset while ApproachState
        self.pos_error_tol = np.array([self.x_tol, self.y_tol, self.z_tol]) # position error torelance
        self.att_error_tol = np.array([self.roll_tol, self.pich_tol, self.yaw_tol]) # attitude error torelance
        self.pos_danger_thre = np.array([self.x_danger_thre, self.y_danger_thre, self.z_danger_thre]) # position danger threshold
        self.att_danger_thre = np.array([self.roll_danger_thre, self.pich_danger_thre, self.yaw_danger_thre]) # attitude danger threshold

    def execute(self, userdata):
        # rospy.loginfo('Executing state STATE1')
        # rospy.sleep(2.0)

        # calculate now follower position in leader coordinate
        #TODO: determine leader namespace dynamically
        try:
            follower_from_leader = self.listener.lookupTransform(self.leader+'/root', self.follower+'/root', rospy.Time(0))
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

        #check if pos and att error are within the torrelance
        if np.all(np.less(np.abs(pos_error),self.pos_error_tol)) and np.all(np.less(np.abs(att_error),self.att_error_tol)):
            return 'done'
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
            self.follower_att_pub.publish(link_rot_follower)

            return 'in_process'

class AssemblyState(smach.State):
    def __init__(self,leader,follower):
        smach.State.__init__(self, outcomes=['done'])
        # rosparam
        self.leader =  leader
        self.follower =  follower
        # publisher
        self.follower_peg_pub = rospy.Publisher(self.follower+"/peg_insertion", Bool, queue_size=10)
        self.follower_docking_pub = rospy.Publisher(self.follower+"/docking_cmd", Bool, queue_size=10)
        self.follower_nav_pub = rospy.Publisher(self.follower+"/uav/nav", FlightNav, queue_size=10)

    def execute(self, userdata):
        rospy.loginfo('assembled!')
        self.follower_peg_pub.publish(True)
        self.follower_docking_pub.publish(True)
        rospy.sleep(2.0)
        return 'done'

#### main class ####
class AssembleDemo():
    def __init__(self):
        rospy.init_node("assemble_demo")

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            # contact
            smach.StateMachine.add('ContactState', ContactState(edge_module='beetle4'), transitions={'done':'StandbyState_3_4', 'in_process':'ContactState'})

            # module3 -> module4
            smach.StateMachine.add('StandbyState_3_4', StandbyState(leader='beetle4',follower='beetle3'), transitions={'done':'ApproachState_3_4', 'in_process':'StandbyState_3_4'})
            smach.StateMachine.add('ApproachState_3_4', ApproachState(leader='beetle4',follower='beetle3'), transitions={'done':'AssemblyState_3_4', 'in_process':'ApproachState_3_4', 'fail':'StandbyState_3_4'})
            smach.StateMachine.add('AssemblyState_3_4', AssemblyState('beetle4','beetle3'), transitions={'done':'StandbyState_2_3'})

            # module2 -> module3
            smach.StateMachine.add('StandbyState_2_3', StandbyState(leader='beetle3',follower='beetle2'), transitions={'done':'ApproachState_2_3', 'in_process':'StandbyState_2_3'})
            smach.StateMachine.add('ApproachState_2_3', ApproachState(leader='beetle3',follower='beetle2'), transitions={'done':'AssemblyState_2_3', 'in_process':'ApproachState_2_3', 'fail':'StandbyState_2_3'})
            smach.StateMachine.add('AssemblyState_2_3', AssemblyState('beetle3','beetle2'), transitions={'done':'StandbyState_1_2'})

            # module1 -> module2
            smach.StateMachine.add('StandbyState_1_2', StandbyState(leader='beetle2',follower='beetle1'), transitions={'done':'ApproachState_1_2', 'in_process':'StandbyState_1_2'})
            smach.StateMachine.add('ApproachState_1_2', ApproachState(leader='beetle2',follower='beetle1'), transitions={'done':'AssemblyState_1_2', 'in_process':'ApproachState_1_2', 'fail':'StandbyState_1_2'})
            smach.StateMachine.add('AssemblyState_1_2', AssemblyState('beetle2','beetle1'), transitions={'done':'succeeded'})            
 
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
