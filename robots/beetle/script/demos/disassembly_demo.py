#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
from beetle.disassembly import * 

class DisassemblyDemo():
    def __init__(self):
        rospy.init_node("disassembly_demo")

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            sm_sub2 = smach.StateMachine(outcomes=['succeeded_2_3']) # module2 depart from  module3
            sm_sub1 = smach.StateMachine(outcomes=['succeeded_1_2']) # module1 depart from  module2

            with sm_sub1:
                smach.StateMachine.add('SwitchState1_2',
                                       SwitchState(robot_name = 'beetle1', robot_id = 1, neighboring = 'beetle2', neighboring_id = 2, separate_dir = -1),
                                       transitions={'done':'SeparateState1_2'})

                smach.StateMachine.add('SeparateState1_2',
                                       SeparateState(robot_name = 'beetle1', robot_id = 1, neighboring = 'beetle2'),
                                       transitions={'done':'succeeded_1_2','in_process':'SeparateState1_2'})

            with sm_sub2:
                smach.StateMachine.add('SwitchState2_3',
                                       SwitchState(robot_name = 'beetle3', robot_id = 3, neighboring = 'beetle2', neighboring_id = 2, separate_dir = 1), 
                                       transitions={'done':'SeparateState2_3'})

                smach.StateMachine.add('SeparateState2_3',
                                       SeparateState(robot_name = 'beetle3', robot_id = 3, neighboring = 'beetle2',separate_vel = 0.12),
                                       transitions={'done':'succeeded_2_3','in_process':'SeparateState2_3'})

            smach.StateMachine.add('SUB2',
                                   sm_sub2,
                                   transitions={'succeeded_2_3':'SUB1'})
            
            smach.StateMachine.add('SUB1',
                                   sm_sub1,
                                   transitions={'succeeded_1_2':'succeeded'})


        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()
if __name__ == '__main__':
    try:
        logging.getLogger('rosout').addFilter(Filter())
        demo = DisassemblyDemo();
        demo.main()
    except rospy.ROSInterruptException: pass
