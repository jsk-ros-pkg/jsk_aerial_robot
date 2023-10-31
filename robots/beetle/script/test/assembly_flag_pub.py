#!/usr/bin/env python

import rospy
import time
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool
from beetle.kondo_control import KondoControl

if __name__=="__main__":
    rospy.init_node("assembly_flag_pub")
    robot_name = 'beetle1'
    robot_id = 1
    male_servo_id = 5
    real_machine = True
    unlock_servo_angle_male = 7000
    lock_servo_angle_male = 8800
    unlock_servo_angle_female = 11000 #todo
    lock_servo_angle_female = 5000 #todo
    leader = 'beetle2'
    leader_id = 2
    female_servo_id = 6
    kondo_servo = KondoControl(robot_name,robot_id,male_servo_id,real_machine)
    kondo_servo_leader = KondoControl(leader,leader_id,female_servo_id,real_machine)
    kondo_servo.sendTargetAngle(lock_servo_angle_male)
    kondo_servo_leader.sendTargetAngle(lock_servo_angle_female)
    r = rospy.Rate(1)
    flag_pub = rospy.Publisher('/beetle1/assembly_flag', KeyValue, queue_size = 1)
    flag_pub2 = rospy.Publisher('/beetle2/assembly_flag', KeyValue, queue_size = 1)
    docking_pub = rospy.Publisher('/beetle1/docking_cmd', Bool, queue_size = 1)

    flag_msg = KeyValue()
    docking_msg = Bool()
    time.sleep(0.5)

    flag_msg.key = '1'
    flag_msg.value = '1'
    flag_pub.publish(flag_msg)
    # time.sleep(0.5)

    flag_msg.key = '2'
    flag_msg.value = '1'
    flag_pub2.publish(flag_msg)
    # time.sleep(0.5)

    docking_msg.data = True
    docking_pub.publish(docking_msg)

