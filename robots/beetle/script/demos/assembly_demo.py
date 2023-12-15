#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
from beetle.assembly import * 

class AssemblyDemo():
    def __init__(self):
        rospy.init_node("assembly_demo")

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded','interupted'])
        with sm_top:
            sm_sub1 = smach.StateMachine(outcomes=['succeeded_1_2','interupted_1_2']) # module1 depart from  module2
            sm_sub2 = smach.StateMachine(outcomes=['succeeded_2_3','interupted_2_3']) # module2 depart from  module3

            with sm_sub1:
                smach.StateMachine.add('StandbyState1_2',
                                       StandbyState(robot_name = 'beetle1', robot_id = 1, leader = 'beetle2', leader_id = 2, attach_dir = -1.0),
                                       transitions={'done':'ApproachState1_2', 'in_process': 'StandbyState1_2', 'emergency':'interupted_1_2'})

                smach.StateMachine.add('ApproachState1_2',
                                       ApproachState(robot_name = 'beetle1', robot_id = 1, leader = 'beetle2', leader_id = 2, attach_dir = -1.0),
                                       transitions={'done':'AssemblyState1_2', 'in_process':'ApproachState1_2', 'fail':'StandbyState1_2', 'emergency':'interupted_1_2'})

                smach.StateMachine.add('AssemblyState1_2', AssemblyState(robot_name = 'beetle1', robot_id = 1, leader = 'beetle2', leader_id = 2, attach_dir = -1.0),
                                       transitions={'done':'succeeded_1_2', 'emergency':'interupted_1_2'})

            with sm_sub2:
                smach.StateMachine.add('StandbyState2_3',
                                       StandbyState(robot_name = 'beetle3', robot_id = 3, leader = 'beetle2', leader_id = 2, attach_dir = 1.0),
                                       transitions={'done':'ApproachState2_3', 'in_process': 'StandbyState2_3', 'emergency':'interupted_2_3'})

                smach.StateMachine.add('ApproachState2_3',
                                       ApproachState(robot_name = 'beetle3', robot_id = 3, leader = 'beetle2', leader_id = 2, attach_dir = 1.0),
                                       transitions={'done':'AssemblyState2_3', 'in_process':'ApproachState2_3', 'fail':'StandbyState2_3', 'emergency':'interupted_2_3'})

                smach.StateMachine.add('AssemblyState2_3', AssemblyState(robot_name = 'beetle3', robot_id = 3, leader = 'beetle2', leader_id = 2, attach_dir = 1.0),
                                       transitions={'done':'succeeded_2_3', 'emergency':'interupted_2_3'})

            smach.StateMachine.add('SUB1',
                                   sm_sub1,
                                   transitions={'succeeded_1_2':'SUB2', 'interupted_1_2':'interupted'})

            smach.StateMachine.add('SUB2',
                                   sm_sub2,
                                   transitions={'succeeded_2_3':'succeeded', 'interupted_2_3':'interupted'})

        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()
if __name__ == '__main__':
    try:
        demo = AssemblyDemo();
        demo.main()
    except rospy.ROSInterruptException: pass
