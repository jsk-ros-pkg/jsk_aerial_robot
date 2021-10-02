#!/usr/bin/env python

import sys
import time
import rospy
import numpy as np
from gazebo_msgs.srv import ApplyJointEffort, JointRequest, SetModelConfiguration
from sensor_msgs.msg import JointState

class InitPose(object):
    def __init__(self):
        joint_state_sub = rospy.Subscriber("joint_states", JointState, self.jointCb)

        joint_controller_params = rospy.get_param("servo_controller/joints")
        self.robot_name = rospy.get_namespace().replace('/', '')
        self.angle_thresh = rospy.get_param("simulation/init_pose_thesh", 0.01) # ~ 1 deg
        self.joint_torque = rospy.get_param("simulation/joint_torque", 1000) # Nm

        self.joint_num = len([k for k in joint_controller_params.keys() if 'controller' in k])
        self.joint_names = [joint_controller_params['controller' + str(i+1)]['name'] for i in range(self.joint_num)]

        self.init_angles = np.array([joint_controller_params['controller' + str(i+1)]['simulation']['init_value'] \
                                     if 'simulation' in joint_controller_params['controller' + str(i+1)] else 0 for i in range(self.joint_num)])

        self.curr_angles = None

    def jointCb(self, msg):
        #self.curr_angles = np.array([msg.position[i] for i in range(len(msg.name)) if msg.name[i] in self.joint_names])
        self.curr_angles = np.array([msg.position[msg.name.index(n)] for n in self.joint_names]) # ensure the order of joint names

    def main(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.curr_angles is None:
                r.sleep()
                continue

            diff_angles =  self.init_angles - self.curr_angles
            max_diff_angle = np.max(np.abs(diff_angles))
            rospy.loginfo("max diff angle: {}".format(max_diff_angle))

            joint_list = []
            # option1: change from jonit torque
            # still slow
            # for i, diff in enumerate(diff_angles):
            #     if np.max(diff) > self.angle_thresh:
            #         joint_list.append(self.joint_names[i])
            #         try:
            #             add_joint_torque = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
            #             add_joint_torque(joint_list[-1], self.joint_torque, rospy.Time(), rospy.Time(1.0))
            #         except rospy.ServiceException, e:
            #             print "Service call failed: %s"%e

            # option2: change directly the model configuration in gazebo
            try:
                change_joint_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
                change_joint_configuration(self.robot_name,
                                           rospy.get_namespace() + 'robot_description',
                                           self.joint_names,
                                           self.init_angles)
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e


            if  max_diff_angle < self.angle_thresh:
                rospy.loginfo("the init pose of robot reach the target init pose!!")
                for j in joint_list:
                    try:
                        clear_joint_torque = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
                        clear_joint_torque(j)
                    except rospy.ServiceException, e:
                        print "Service call failed: %s"%e

                break

            r.sleep()


if __name__ == "__main__":

    rospy.init_node("dragon_init_pose")

    pose_controller = InitPose()
    pose_controller.main()

