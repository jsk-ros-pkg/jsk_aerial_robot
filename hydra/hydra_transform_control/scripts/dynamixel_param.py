#!/usr/bin/env python

import sys
import rospy
from dynamixel_controllers.srv import *

def set_compliance_params(servo_id, margin_value, slope_value, punch_value):
    servo_controller_name = '/j' + servo_id + '_controller/'

    service_name =  servo_controller_name + 'set_compliance_margin'
    rospy.wait_for_service(service_name)

    try:
        set_compliance_margin = rospy.ServiceProxy(service_name, SetComplianceMargin)
        set_compliance_margin(margin_value)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    service_name =  servo_controller_name + 'set_compliance_slope'
    rospy.wait_for_service(service_name)
    try:
        set_compliance_slope = rospy.ServiceProxy(service_name, SetComplianceSlope)
        res1 = set_compliance_slope(slope_value)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    service_name =  servo_controller_name + 'set_compliance_punch'
    rospy.wait_for_service(service_name)
    try:
        set_compliance_punch = rospy.ServiceProxy(service_name, SetCompliancePunch)
        res1 = set_compliance_punch(punch_value)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == "__main__":

    rospy.init_node("servo_compliance_params")
    rospy.loginfo("servo compliance params")

    servo_id = rospy.get_param('~servo_id','2')
    margin_value = int(rospy.get_param('~margin_value','5'))
    margin_slope = int(rospy.get_param('~margin_slope','0'))
    margin_punch = int(rospy.get_param('~margin_punch','100'))

    set_compliance_params(servo_id, margin_value, margin_slope, margin_punch)
