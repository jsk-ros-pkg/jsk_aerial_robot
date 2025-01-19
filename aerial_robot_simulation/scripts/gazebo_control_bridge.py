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

shutdown_timeout = 0.5 #30.0

class GazeboControlBridge:

    def init(self):
        rospy.init_node('gazebo_control_bridge', anonymous=True)

        # controller
        controller_manager_name = rospy.get_namespace() + "controller_manager/load_controller"
        rospy.wait_for_service(controller_manager_name)
        try:
            controller_loader = rospy.ServiceProxy(controller_manager_name, LoadController)
            controller_loader.wait_for_service(timeout=shutdown_timeout)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        if not rospy.has_param("actuator_controller"):
            rospy.logerr("you have to define the actuator controller (e.g. joint)")
            return

        controllers = []
        init_values = {}
        self.actuator_command_pubs = {}

        self.controller_groups = rospy.get_param("actuator_controller")
        for group_name, group_param in self.controller_groups.iteritems():
            subgroup_init_values = []
            actuator_command_pubs = []

            # rosparam in rospy is not ordered (e.g. controller3, controller2, controller1, ...)
            for i in range(0, len([key for key in group_param.keys() if key.find("controller") == 0])):

                param = group_param["controller" + str(i+1)]
                actuactor_name = param["joint"]

                if "init_value" in param.keys():
                    subgroup_init_values.append(param["init_value"])
                else:
                    subgroup_init_values.append(group_param["init_value"])

                controllers.append("actuator_controller/" + group_name + "/controller" + str(i+1))
                if not "pid" in param.keys():
                    rospy.set_param(controllers[-1] + "/pid", group_param["pid"])
                if not "type" in param.keys():
                    rospy.set_param(controllers[-1] + "/type", group_param["type"])

                res = controller_loader(controllers[-1])
                if res.ok:
                    rospy.loginfo("loaded: %s",  controllers[-1])
                else:
                    rospy.logerr("can not load: %s",  controllers[-1])

                # setup the command publisher
                actuator_command_pubs.append(rospy.Publisher(rospy.get_namespace() + controllers[-1] + "/command", Float64, queue_size = 1))

            init_values[group_name] = subgroup_init_values
            self.actuator_command_pubs[group_name] = actuator_command_pubs

            # setup the control subscriber
            group_ctrl_topic_name = rospy.get_param("actuator_controller/" + group_name + "/ctrl_topic_name", group_name + "_ctrl")
            self.joint_ctrl_sub_ = rospy.Subscriber(group_ctrl_topic_name, JointState, self.actuatorSubGroupCtrlCallback, group_name)

        # start all controllers
        try:
            controller_switcher = rospy.ServiceProxy(rospy.get_namespace() + "controller_manager/switch_controller", SwitchController)
            controller_switcher.wait_for_service(timeout=shutdown_timeout)

            res = controller_switcher(controllers, [], SwitchControllerRequest.STRICT)

            if res.ok:
                rospy.loginfo("start controllers")
            else:
                rospy.logerr("can not start controller")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # send init pose
        time.sleep(1)
        for group_name in self.controller_groups.keys():
            for i in range(0, len(self.actuator_command_pubs[group_name])):
                self.actuator_command_pubs[group_name][i].publish(init_values[group_name][i])

    def actuatorSubGroupCtrlCallback(self, msg, group_name):
        for i in range(0, len(msg.position)):
            self.actuator_command_pubs[group_name][i].publish(msg.position[i])

if __name__ == '__main__':
    try:
        control_bridge = GazeboControlBridge()
        control_bridge.init()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

