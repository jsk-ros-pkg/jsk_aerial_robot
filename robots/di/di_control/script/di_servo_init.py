#!/usr/bin/env python


import sys
import rospy
from dynamixel_controllers.srv import *
from std_msgs.msg import *
import math

def set_compliance_params(servo_max_id, margin_value, slope_value, punch_value):
    for i in range(1, int(servo_max_id) + 1):
        servo_controller_name = '/j' + str(i) + '_controller/'

        service_name =  servo_controller_name + 'torque_enable'
        rospy.wait_for_service(service_name)
        try:
            set_torque_enable = rospy.ServiceProxy(service_name, TorqueEnable)
            set_torque_enable(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    

        '''
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
        '''


def set_init_angles(servo_max_id):
    if int(servo_max_id) == 8:
        i = int(servo_max_id)


if __name__ == "__main__":

    rospy.init_node("servo_compliance_params")
    rospy.loginfo("servo compliance params")

    servo_max_id = rospy.get_param('~servo_max_id','8')
    init_angle_pubs = []
    for i in range(1,  int(servo_max_id) + 1):
        topic = '/j' + str(i) + '_controller/command'
        init_angle_pub = rospy.Publisher(topic, Float64, queue_size=1 )
        init_angle_pubs.append(init_angle_pub)

    margin_value = int(rospy.get_param('~margin_value','5'))
    margin_slope = int(rospy.get_param('~margin_slope','0'))
    margin_punch = int(rospy.get_param('~margin_punch','100'))

    set_compliance_params(servo_max_id, margin_value, margin_slope, margin_punch)


    for i in range(int(servo_max_id)):
        comm = Float64()
        
        if i == 0:
            comm.data = 3.16
        if i == 1:
            comm.data = math.pi
        if i == 2:
            comm.data = 3.2
        if i == 3:
            comm.data = math.pi
        if i == 4:
            comm.data = 3.142
        if i == 5:
            comm.data = math.pi
        if i == 6:
            comm.data = 3.13
        if i == 7:
            comm.data = math.pi

        init_angle_pubs[i].publish(comm)

