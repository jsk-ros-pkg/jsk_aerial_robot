#!/usr/bin/env python

import time
import sys
import rospy
import math
import tf
import numpy as np
from geometry_msgs.msg import Twist, Quaternion, PointStamped, Vector3Stamped
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from controller_manager_msgs.srv import *

shutdown_timeout = 30.0

class GazeboControlBridge:

    def init(self):
        rospy.init_node('gazebo_control_bridge', anonymous=True)

        controller_manager_name = rospy.get_namespace() + "controller_manager/load_controller"

        # joint
        self.joint_num = rospy.get_param("joint_num", 1)
        self.init_joint_angle = []
        self.joint_command_pubs = []
        self.controller_names = []

        for i in range(0, self.joint_num):
            # joint init angle
            self.joint_command_pubs.append(rospy.Publisher(rospy.get_namespace() + "joint" + str(i+1) + "_position_controller/command", Float64, queue_size = 1))
            self.init_joint_angle.append(rospy.get_param("joint" + str(i+1) + "/init_angle", 0))

            # controller
            rospy.wait_for_service(controller_manager_name)
            try:
                controller_loader = rospy.ServiceProxy(controller_manager_name, LoadController)
                controller_loader.wait_for_service(timeout=shutdown_timeout)
                self.controller_names.append("joint" + str(i+1) + "_position_controller")

                res = controller_loader(self.controller_names[i])

                if res.ok:
                    rospy.loginfo("loaded: %s",  self.controller_names[i])
                else:
                    rospy.logerr("can not load: %s",  self.controller_names[i])
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        # gimbal
        self.gimbal_num = rospy.get_param("gimbal_num", 0)
        self.init_gimbal_angle = []
        self.gimbal_command_pubs = []

        for i in range(0, self.gimbal_num):
            # gimbal init angle
            self.gimbal_command_pubs.append(rospy.Publisher(rospy.get_namespace() + "gimbal" + str(i+1) + "_position_controller/command", Float64, queue_size = 1))
            self.init_gimbal_angle.append(rospy.get_param("gimbal" + str(i+1) + "/init_angle", 0))

            # controller
            rospy.wait_for_service(controller_manager_name)
            try:
                controller_loader = rospy.ServiceProxy(controller_manager_name, LoadController)
                controller_loader.wait_for_service(timeout=shutdown_timeout)
                self.controller_names.append("gimbal" + str(i+1) + "_position_controller")

                res = controller_loader(self.controller_names[i + self.joint_num])

                if res.ok:
                    rospy.loginfo("loaded: %s",  self.controller_names[i + self.joint_num])
                else:
                    rospy.logerr("can not load: %s",  self.controller_names[i + self.joint_num])
            except rospy.ServiceException, e:
                print "Service call failed: %s"%e

        # start all controllers
        try:
            controller_switcher = rospy.ServiceProxy(rospy.get_namespace() + "controller_manager/switch_controller", SwitchController)
            controller_switcher.wait_for_service(timeout=shutdown_timeout)

            res = controller_switcher(self.controller_names, [], SwitchControllerRequest.STRICT)

            if res.ok:
                rospy.loginfo("start controllers")
            else:
                rospy.logerr("can not start controller")
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # slepp
        time.sleep(1)
        # send init pose
        for i in range(0, self.joint_num):
            self.joint_command_pubs[i].publish(self.init_joint_angle[i])
        for i in range(0, self.gimbal_num):
            self.gimbal_command_pubs[i].publish(self.init_gimbal_angle[i])

        self.joint_ctrl_sub_topic_name_ = rospy.get_param("~joint_ctrl_sub_topic_name", "joints_ctrl")
        self.joint_ctrl_sub_ = rospy.Subscriber(self.joint_ctrl_sub_topic_name_, JointState, self.jointCtrlCallback)

        self.gimbal_ctrl_sub_topic_name_ = rospy.get_param("~gimbal_ctrl_sub_topic_name", "gimbals_ctrl")
        self.gimbal_ctrl_sub_ = rospy.Subscriber(self.gimbal_ctrl_sub_topic_name_, JointState, self.gimbalCtrlCallback)

    def jointCtrlCallback(self, msg):
        if len(msg.position) != self.joint_num:
            rospy.err("the motor num is not equal with joint control number")
            return

        for i in range(0, self.joint_num):
            self.joint_command_pubs[i].publish(msg.position[i])

    def gimbalCtrlCallback(self, msg):
        if len(msg.position) != self.gimbal_num:
            rospy.err("the motor num is not equal with joint control number")
            return

        for i in range(0, self.gimbal_num):
            self.gimbal_command_pubs[i].publish(msg.position[i])

if __name__ == '__main__':
    try:
        control_bridge = GazeboControlBridge()
        control_bridge.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

