#!/usr/bin/env python

from http import server
import queue
import sndhdr
import sys
import time
from tkinter import SE
import rospy
import math
import copy
import select, termios, tty
from std_msgs.msg import UInt8, Bool, Empty
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from spinal.msg import DesireCoord
from dynamixel_workbench_msgs.srv import *
from geometry_msgs.msg import WrenchStamped

class avatar_control():

    def __init__(self):

        self.debug = rospy.get_param("~debug", False)
        self.gripper = rospy.get_param("~gripper", False)
        self.robot_name = rospy.get_param("~robot_ns", "hydrus_xi")
        
        topic_name = '/'+self.robot_name+'/joints_ctrl'
        if self.debug:
            topic_name = '/'+self.robot_name+'/joint_states'

        self.joint_servo_pub = rospy.Publisher('/dynamixel_workbench/joint_trajectory', JointTrajectory, queue_size=10)
        self.joint_control_pub = rospy.Publisher(topic_name, JointState, queue_size=10)
        self.ref_joint_angles_pub = rospy.Publisher('/'+self.robot_name+'/ref_joint_angles', JointState, queue_size=10)
        self.des_wrench_pub = rospy.Publisher('/'+self.robot_name+'/desire_wrench', WrenchStamped, queue_size=10)
        self.avatar_sub = rospy.Subscriber('/dynamixel_workbench/joint_states', JointState, self.avatarCb)
        self.flight_state_sub = rospy.Subscriber('/'+self.robot_name+'/flight_state', UInt8, self.flight_stateCb)
        self.land_sub = rospy.Subscriber('/'+self.robot_name+'/teleop_command/land', Empty, self.landCb)
        self.static_thrust_sub = rospy.Subscriber('/'+self.robot_name+'/static_thrust_available', Bool, self.static_thrustCb)

        self.raw_servo_position = None
        self.desire_joint = JointState()
        self.Hovering = False
        self.Landing = False
        self.servo_Switch_state = True
        self.danger_config = False
        self.staric_thrust_available = True
        self.angle_limit = rospy.get_param("angle_limit", 1.535) # the limitation of the joint
        self.min_yaw_angle = rospy.get_param("min_yaw_angle", 0.3)
        self.yaw_sum_threshold = rospy.get_param("yaw_sum_threshold", 1.0)
        self.servo_init_time = 0.5

    def servo_Switch(self,Switch,servo_number):
        rospy.wait_for_service('/dynamixel_workbench/dynamixel_command')
        try:
            client = rospy.ServiceProxy('/dynamixel_workbench/dynamixel_command', DynamixelCommand)
            resp = client('', servo_number, 'Torque_Enable', Switch)
        except rospy.ServiceException as e:
            print("Service call failed: {}".format(e))
        self.servo_Switch_state = Switch

    def servo_number_to_name_converter(self,servo_number):
        joint_name_list = []
        for i in range(len(servo_number)):
            if servo_number[i] == 1:
                joint_name_list.append("joint1_pitch")
            if servo_number[i] == 2:
                joint_name_list.append("joint1_yaw")
            if servo_number[i] == 3:
                joint_name_list.append("joint2_pitch")
            if servo_number[i] == 4:
                joint_name_list.append("joint2_yaw")
            if servo_number[i] == 5:
                joint_name_list.append("joint3_pitch")
            if servo_number[i] == 6:
                joint_name_list.append("joint3_yaw")
        return joint_name_list
        
    def move_servo(self,Servo_number,desire_angle):
        for i in range(len(Servo_number)):
            self.servo_Switch(Switch=True, servo_number=Servo_number[i])
        self.desire_servo_position = JointTrajectory()
        self.desire_servo_position.joint_names = self.servo_number_to_name_converter(Servo_number)
        self.desire_servo_position.points = [JointTrajectoryPoint()]
        self.desire_servo_position.points[0].positions = desire_angle
        #self.desire_servo_position.points[0].velocities = [0.1]       
        self.desire_servo_position.points[0].time_from_start = rospy.Time(1.0)
        self.joint_servo_pub.publish(self.desire_servo_position)
        rospy.sleep(3.0)
        for i in range(len(Servo_number)):
            self.servo_Switch(Switch=False, servo_number=Servo_number[i])
        rospy.loginfo("move done")


    def set_servo_init(self,msg):
        # set the joint servo to havering
        self.servo_init_position = JointTrajectory()
        self.servo_init_position.joint_names = ["joint1_pitch", "joint1_yaw", "joint2_pitch", "joint2_yaw", "joint3_pitch", "joint3_yaw"]
        self.servo_init_position.points = [JointTrajectoryPoint()]
        self.servo_init_position.points[0].positions = [0.0, 1.5, 0.0, 1.5, 0.0, 1.5]
        self.servo_init_position.points[0].time_from_start = rospy.Time(self.servo_init_time)
        self.joint_servo_pub.publish(self.servo_init_position)
        rospy.sleep(self.servo_init_time * 10)
        rospy.loginfo("joint servo init DONE!")

    def flight_stateCb(self,msg):
        self.flight_state = msg.data
        if self.flight_state == 5:
            self.Hovering = True 
        if self.flight_state == 2:
            self.Landing = False

    def landCb(self,msg):
        self.Landing = True


    def static_thrustCb(self,msg):
        self.staric_thrust_available = msg.data

    def avatarCb(self,msg):
        
        #init
        if self.raw_servo_position is None:
            # set the joint servo on
            for i , n in enumerate(msg.name):
                self.servo_Switch(Switch=True, servo_number=i+1)
            # set the joint servo to havering
            self.set_servo_init(msg)

        # set the joint servo off
        
        if self.debug==False and self.Hovering==True and self.servo_Switch_state==True and self.danger_config==False:
        #if self.debug==False and self.servo_Switch_state==True and self.danger_config==False:
            for i , n in enumerate(msg.name):
                if 'yaw' in n:
                    self.servo_Switch(Switch=False, servo_number=i+1)
                if self.robot_name == "dragon":
                    self.servo_Switch(Switch=False, servo_number=i+1)
                if self.gripper==True:
                    self.servo_Switch(Switch=True, servo_number=4)
                    self.servo_Switch_state = False
            rospy.loginfo("servo off")
        if self.debug==True and self.servo_Switch_state==True and self.danger_config==False:
            for i , n in enumerate(msg.name):
                if 'yaw' in n:
                    self.servo_Switch(Switch=False, servo_number=i+1)
                if self.robot_name == "dragon":
                    self.servo_Switch(Switch=False, servo_number=i+1)
                if self.gripper==True:
                    self.servo_Switch(Switch=True, servo_number=4)
                    self.servo_Switch_state = False
            rospy.loginfo("servo off")

        self.raw_servo_position = msg
        self.raw_servo_position.name = list(self.raw_servo_position.name)
        self.raw_servo_position.position = list(self.raw_servo_position.position)


    def main(self):

        r = rospy.Rate(40)
        while not rospy.is_shutdown():

            if self.raw_servo_position is None:
                continue
            # for hydrus
            if self.robot_name == "hydrus_xi":
                self.desire_joint = copy.deepcopy(self.raw_servo_position)
                #rospy.loginfo(self.desire_joint)
                self.desire_joint.name = ["joint1", "joint2","joint3"]
                del self.desire_joint.position[0::2]

            # for hydrus_gripper
            if self.gripper==True:
                self.desire_joint = copy.deepcopy(self.raw_servo_position)
                del self.desire_joint.position[2:5]
                del self.desire_joint.position[0]
                self.desire_joint.name = ["joint1", "joint3"]

            # for dragon
            if self.robot_name == "dragon":
                self.desire_joint = copy.deepcopy(self.raw_servo_position)

            #avoid over angle 
            for i in range(len(self.desire_joint.position)):
                if self.desire_joint.position[i] > self.angle_limit:
                    self.desire_joint.position[i] = self.angle_limit
                if self.desire_joint.position[i] < -self.angle_limit:
                    self.desire_joint.position[i] = -self.angle_limit

            # avoid stright line configuration
            if self.gripper==False:
                '''
                sum = 0.0
                #for i in range(len(desire_joint.position)):
                    #sum += desire_joint.position[i]
                sum = self.desire_joint.position[0] + self.desire_joint.position[1]
                if sum <= self.yaw_sum_threshold and self.desire_joint.position[2]<=1.5:
                    rospy.loginfo("caution")
                    self.danger_config=True
                    self.servo_Switch(Switch=True,servo_number=2)
                    self.servo_Switch(Switch=True,servo_number=4)
                    self.desire_joint.position[0] = self.yaw_sum_threshold/2
                    self.desire_joint.position[1] = self.yaw_sum_threshold/2
                else:
                    self.danger_config=False
                

                if self.staric_thrust_available == False:
                    self.danger_config = True
                    self.desire_joint.position[2] = 0.0
                    self.desire_joint.position[1] = 1.57
                    self.joint_control_pub.publish(self.desire_joint)
                    #self.servo_Switch(Switch=True,servo_number=6)
                    self.move_servo(Servo_number=[4,6],desire_angle=[1.57,0.0])
                    #self.servo_Switch(Switch=False,servo_number=6)
                    self.danger_config = False
                '''


            if self.gripper==True:
                if self.staric_thrust_available == False:
                    self.danger_config = True
                    if self.desire_joint.position[0] <= self.desire_joint.position[1]:
                        self.desire_joint.position[0] = 0.0
                        self.joint_control_pub.publish(self.desire_joint)
                        self.servo_Switch(Switch=True,servo_number=6)
                        self.move_servo(Servo_number=[2],desire_angle=[0.0])
                        self.servo_Switch(Switch=False,servo_number=6)
                        self.danger_config = False
                    if self.desire_joint.position[0] > self.desire_joint.position[1]:
                        self.desire_joint.position[1] = 0.0
                        self.joint_control_pub.publish(self.desire_joint)
                        self.servo_Switch(Switch=True,servo_number=2)
                        self.move_servo(Servo_number=[6],desire_angle=[0.0])
                        self.servo_Switch(Switch=False,servo_number=2)
                        self.danger_config = False
            '''
            # add gimbal angle if necessary
            if self.debug:
                desire_joint.name.extend(['gimbal1_roll', 'gimbal1_pitch', 'gimbal2_roll'opt_joint_an, 'gimbal2_pitch', 'gimbal3_roll', 'gimbal3_pitch', 'gimbal4_roll', 'gimbal4_pitch'])
                desire_joint.position.extend([0] * 8)
            '''
            desire_wrench_msg = WrenchStamped()
            desire_wrench_msg.wrench.force.x = 1.0
            desire_wrench_msg.wrench.force.y = 0.0
            desire_wrench_msg.wrench.force.z = 0.0
            #self.des_wrench_pub.publish(desire_wrench_msg)

            # send joint angles
            if self.Landing == False:
                self.joint_control_pub.publish(self.desire_joint)
            #self.ref_joint_angles_pub.publish(self.desire_joint)

            r.sleep()

if __name__ == "__main__":
  rospy.init_node("avatar_control")
  Tracker = avatar_control()
  Tracker.main()


