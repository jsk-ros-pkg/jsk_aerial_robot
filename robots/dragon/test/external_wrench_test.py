#!/usr/bin/env python

import sys
import time
import rospy
import math
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest, BodyRequest, BodyRequestRequest
from aerial_robot_msgs.msg import ApplyWrench
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3, Wrench

if __name__ == "__main__":

    rospy.init_node("external_wrench_test")

    reference_frame = rospy.get_param("~reference_frame", "link4")
    # NOTE2: default reference point in gazebo apply wrench is the CoG of reference frame (link).
    # So, please assign the CoG of link for applying compesation wrench for flight control, but do not add offset in gazebo
    # C.F.:
    # $ rosrun dragon external_wrench_test.py _clear:=false _force:=[10,0,0] _reference_frame:=link4 _offset:=[0.2,0,-0.0207] _gazebo_offset:=[0,0,0]
    # $ rosrun dragon external_wrench_test.py _clear:=false _force:=[10,0,0] _reference_frame:=link1 _offset:=[0.2271,-0.0067,-0.0272] _gazebo_offset:=[0,0,0]
    offset = rospy.get_param("~offset", [0, 0, 0])
    force = rospy.get_param("~force", [1, 0, 0])
    torque = rospy.get_param("~torque", [0, 0, 0])
    clear = rospy.get_param("~clear", False)
    gazebo_offset = offset
    if rospy.has_param("~gazebo_offset"):
        gazebo_offset = rospy.get_param("~gazebo_offset")

    apply_pub = rospy.Publisher('/dragon/apply_external_wrench', ApplyWrench, queue_size=1)
    clear_pub = rospy.Publisher('/dragon/clear_external_wrench', String, queue_size=1)

    time.sleep(0.5)

    if clear:


        rospy.wait_for_service('/gazebo/clear_body_wrenches', 1.0)
        # to gazebo
        try:
            req = BodyRequestRequest()
            if reference_frame == 'link1':
                reference_frame = 'root'
            req.body_name = 'dragon::' + reference_frame
            # NOTE2: do not assing any variable to req.refrence_frame
            # Please check https://github.com/ros-simulation/gazebo_ros_pkgs/blob/noetic-devel/gazebo_ros/src/gazebo_ros_api_plugin.cpp#L1926-L2071
            clear_wrench_to_gazebo = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
            resp = clear_wrench_to_gazebo(req)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e


        # to robot
        clear_pub.publish(String('wrench1'))

    else:
        gazebo_apply_wrench_name = '/gazebo/apply_body_wrench'
        rospy.wait_for_service(gazebo_apply_wrench_name, 1.0)

        # to gazebo
        try:
            req = ApplyBodyWrenchRequest()
            req.body_name = 'dragon::' + reference_frame if reference_frame != 'link1' else 'root'
            req.reference_point = Point(gazebo_offset[0], gazebo_offset[1], gazebo_offset[2])
            req.wrench = Wrench(Vector3(force[0], force[1], force[2]), Vector3(torque[0], torque[1], torque[2]))
            req.duration.secs = -1
            apply_wrench_to_gazebo = rospy.ServiceProxy(gazebo_apply_wrench_name, ApplyBodyWrench)
            resp1 = apply_wrench_to_gazebo(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # to robot
        msg = ApplyWrench()
        msg.name = 'wrench1'
        msg.reference_frame = reference_frame
        msg.reference_point = Point(offset[0], offset[1], offset[2])
        msg.wrench = Wrench(Vector3(-force[0], -force[1], -force[2]),
                            Vector3(-torque[0], -torque[1], -torque[2]))
        apply_pub.publish(msg)


