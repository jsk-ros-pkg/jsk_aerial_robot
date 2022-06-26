#!/usr/bin/env python

import sys
import time
import rospy
import math
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest, BodyRequestRequest
from geometry_msgs.msg import Point, Vector3, Wrench

if __name__ == "__main__":

    rospy.init_node("external_wrench_test")

    reference_frame = rospy.get_param("~reference_frame", "inter_joint2")
    offset = rospy.get_param("~offset", [0, 0, 0])
    force = rospy.get_param("~force", [0, 0, 0])
    torque = rospy.get_param("~torque", [0, 0, 0])
    clear = rospy.get_param("~clear", False)
    gazebo_offset = offset
    if rospy.has_param("~gazebo_offset"):
        gazebo_offset = rospy.get_param("~gazebo_offset")

    wrench = Wrench(Vector3(force[0], force[1], force[2]), Vector3(torque[0], torque[1], torque[2]))

    if clear:
        robot_clear_wrench_name = 'clear_external_wrench'
        rospy.wait_for_service(robot_clear_wrench_name, 1.0)

        gazebo_clear_wrench_name = '/gazebo/clear_body_wrenches'
        rospy.wait_for_service(gazebo_clear_wrench_name, 1.0)

        # to robot
        try:
            req = BodyRequestRequest()
            req.body_name = 'force1'
            clear_wrench_to_robot = rospy.ServiceProxy(robot_clear_wrench_name, BodyRequest)
            resp1 = clear_wrench_to_robot(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # to gazebo
        try:
            req = BodyRequestRequest()
            req.body_name = rospy.get_namespace().strip('/') +'::' + reference_frame
            clear_wrench_to_gazebo = rospy.ServiceProxy(gazebo_clear_wrench_name, BodyRequest)
            resp1 = clear_wrench_to_gazebo(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    else:
        robot_apply_wrench_name = 'apply_external_wrench'
        rospy.wait_for_service(robot_apply_wrench_name, 1.0)

        gazebo_apply_wrench_name = '/gazebo/apply_body_wrench'
        rospy.wait_for_service(gazebo_apply_wrench_name, 1.0)

        # to robot
        try:
            req = ApplyBodyWrenchRequest()
            req.body_name = 'force1'
            req.reference_frame = reference_frame
            req.reference_point = Point(offset[0], offset[1], offset[2])
            req.wrench = wrench
            apply_wrench_to_robot = rospy.ServiceProxy(robot_apply_wrench_name, ApplyBodyWrench)
            resp1 = apply_wrench_to_robot(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # to gazebo
        try:
            req = ApplyBodyWrenchRequest()
            req.body_name = rospy.get_namespace().strip('/') +'::' + reference_frame
            req.reference_point = Point(gazebo_offset[0], gazebo_offset[1], gazebo_offset[2])
            req.wrench = wrench
            req.duration.secs = -1
            apply_wrench_to_gazebo = rospy.ServiceProxy(gazebo_apply_wrench_name, ApplyBodyWrench)
            resp1 = apply_wrench_to_gazebo(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

