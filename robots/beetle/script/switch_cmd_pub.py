#!/usr/bin/env python

import rospy
import time
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Bool
from beetle.kondo_control import KondoControl

if __name__=="__main__":
    rospy.init_node("switch_cmd_pub")
    real_machine = rospy.get_param("real_machine",False)
    switch_type = rospy.get_param("/switch_type",1) # 1 -> assembly, 2->disassembly
    print(switch_type)

    male_servo_id = rospy.get_param("male_serve_id",5)
    female_servo_id = rospy.get_param("female_serve_id",6)
    unlock_servo_angle_male = rospy.get_param("unlock_servo_angle_male",7000)
    lock_servo_angle_male = rospy.get_param("lock_servo_angle_male",8300)
    unlock_servo_angle_female = rospy.get_param("unlock_servo_angle_female",5600)
    lock_servo_angle_female = rospy.get_param("unlock_servo_angle_female",11000)

    left_edge_id = rospy.get_param("left_edge_id",1)
    right_edge_id = rospy.get_param("right_edge_id",3)
    
    robot_ids = list(range(left_edge_id, right_edge_id + 1))
    print(robot_ids)
    flag_pubs = []
    docking_pubs =[]
    male_servo_handlers = []
    female_servo_handlers = []

    flag_msg = KeyValue()
    docking_msg = Bool()

    for index, id in enumerate(robot_ids):
        robot_name = 'beetle' + str(id)
        flag_pubs.append(rospy.Publisher('/' +robot_name  + '/assembly_flag', KeyValue, queue_size=1))
        docking_pubs.append(rospy.Publisher('/' +robot_name  + '/docking_cmd', Bool, queue_size=1))
        male_servo_handlers.append(KondoControl(robot_name,id,male_servo_id,real_machine))
        female_servo_handlers.append(KondoControl(robot_name,id,female_servo_id,real_machine))

    for index, id in enumerate(robot_ids):
        flag_msg.key = str(id)
        if switch_type:
            flag_msg.value = '1'
        else:
            flag_msg.value = '0'
        flag_pubs[index].publish(flag_msg)
        if real_machine:
            if id == left_edge_id:
                if switch_type:
                    male_servo_handlers[index].sendTargetAngle(lock_servo_angle_male)
                else:
                    male_servo_handlers[index].sendTargetAngle(unlock_servo_angle_male)
            elif id == right_edge_id:
                if switch_type:
                    female_servo_handlers[index].sendTargetAngle(lock_servo_angle_female)
                else:
                    female_servo_handlers[index].sendTargetAngle(unlock_servo_angle_female)
            else:
                if switch_type:
                    male_servo_handlers[index].sendTargetAngle(lock_servo_angle_male)
                    female_servo_handlers[index].sendTargetAngle(lock_servo_angle_female)
                else:
                    male_servo_handlers[index].sendTargetAngle(unlock_servo_angle_male)
                    female_servo_handlers[index].sendTargetAngle(unlock_servo_angle_female)
        else:
            if switch_type:
                docking_msg.data = True
            else:
                docking_msg.data = False
            docking_pubs[index].publish(docking_msg)

