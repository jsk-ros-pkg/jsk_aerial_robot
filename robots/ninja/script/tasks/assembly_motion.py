#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
from ninja.assembly_api import * 

class AssemblyDemo():
    def __init__(self,module_ids,real_machine):
        self.module_ids = module_ids
        self.real_machine = real_machine
        rospy.loginfo(self.module_ids)
    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded','interupted'])
        with sm_top:
            for i,item in enumerate(self.module_ids):
                if i == 0:
                    continue
                target_follower_id = self.module_ids[i]
                target_leader_id = self.module_ids[i-1]
                motion_prefix = str(target_follower_id) + '_' + str(target_leader_id)
                sub_sm = smach.StateMachine(outcomes=['succeeded_'+motion_prefix,'interupted_'+motion_prefix])
                direction = 1 if target_follower_id > target_leader_id else -1
                with sub_sm:
                    smach.StateMachine.add('StandbyState'+ motion_prefix,
                                           StandbyState(robot_name = 'ninja'+str(target_follower_id), robot_id = target_follower_id, leader = 'ninja'+str(target_leader_id), leader_id = target_leader_id, attach_dir = direction, real_machine = self.real_machine),
                                           transitions={'done':'ApproachState' + motion_prefix, 'in_process': 'StandbyState'+motion_prefix, 'emergency':'interupted_'+motion_prefix})

                    smach.StateMachine.add('ApproachState' + motion_prefix,
                                           ApproachState(robot_name = 'ninja'+str(target_follower_id), robot_id = target_follower_id, leader = 'ninja'+str(target_leader_id), leader_id = target_leader_id, attach_dir = direction, real_machine = self.real_machine),
                                           transitions={'done':'AssemblyState'+ motion_prefix, 'in_process':'ApproachState'+motion_prefix, 'fail':'StandbyState'+motion_prefix, 'emergency':'interupted_'+motion_prefix})

                    smach.StateMachine.add('AssemblyState'+motion_prefix, AssemblyState(robot_name = 'ninja'+str(target_follower_id), robot_id = target_follower_id, leader = 'ninja'+str(target_leader_id), leader_id = target_leader_id, attach_dir = direction, real_machine = self.real_machine),
                                           transitions={'done':'succeeded_'+motion_prefix, 'emergency':'interupted_'+motion_prefix})
                if(i == len(self.module_ids)-1):
                    smach.StateMachine.add('SUB'+str(i),
                                           sub_sm,
                                           transitions={'succeeded_'+motion_prefix:'succeeded', 'interupted_'+motion_prefix:'interupted'})
                else:
                    smach.StateMachine.add('SUB'+str(i),
                                           sub_sm,
                                           transitions={'succeeded_'+motion_prefix:'SUB1'+str(i+1), 'interupted_'+motion_prefix:'interupted'})

        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()
if __name__ == '__main__':
    rospy.init_node("assembly_motion")
    modules_str = rospy.get_param("module_ids", default="")
    rm = rospy.get_param("real_machine", default=True)
    modules = []
    if(modules_str):
        modules = [int(x) for x in modules_str.split(',')]
    else:
        rospy.loginfo("No module ID is designated!")
    try:
        assembly_motion = AssemblyDemo(module_ids = modules, real_machine=rm);
        assembly_motion.main()
    except rospy.ROSInterruptException: pass
