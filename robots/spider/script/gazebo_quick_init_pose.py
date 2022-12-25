#!/usr/bin/env python

import sys
import time
import rospy
import numpy as np
from gazebo_msgs.srv import ApplyJointEffort, JointRequest, SetModelConfiguration, SetModelState
from gazebo_msgs.msg import ModelState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from control_msgs.msg import JointControllerState

class InitPose(object):
    def __init__(self):
        joint_controller_params = rospy.get_param("servo_controller/joints")
        init_pos = rospy.get_param("~init_pos", [0,0,0.3])
        self.robot_name = rospy.get_namespace().replace('/', '')

        self.joint_num = len([k for k in joint_controller_params.keys() if 'controller' in k])
        self.joint_names = [joint_controller_params['controller' + '{:0>2}'.format(i+1)]['name'] for i in range(self.joint_num)]

        self.init_angles = np.array([joint_controller_params['controller' + '{:0>2}'.format(i+1)]['simulation']['init_value'] \
                                     if 'simulation' in joint_controller_params['controller' + '{:0>2}'.format(i+1)] else 0 for i in range(self.joint_num)])

        if rospy.has_param("~joint_names"):
            self.joint_names = rospy.get_param("~joint_names")

        if rospy.has_param("~init_angles"):
            self.init_angles = rospy.get_param("~init_angles")

        self.servo_sub = rospy.Subscriber("servo_controller/joints/controller" + '{:0>2}'.format(self.joint_num) + "/simulation/state", JointControllerState, self.servoCb)

        self.last_servo_state = None

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.last_servo_state is not None:
                break
            r.sleep()

        try:
            change_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            state = ModelState()
            state.model_name = self.robot_name
            state.pose = Pose(Point(init_pos[0], init_pos[1], init_pos[2]),
                              Quaternion(0, 0, 0, 1))
            change_pose(state)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        try:
            change_joint_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
            change_joint_configuration(self.robot_name,
                                       rospy.get_namespace() + 'robot_description',
                                       self.joint_names,
                                       self.init_angles)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def servoCb(self, msg):
        self.last_servo_state = msg
        self.servo_sub.unregister()


if __name__ == "__main__":

    rospy.init_node("spider_init_pose")
    pose_controller = InitPose()


