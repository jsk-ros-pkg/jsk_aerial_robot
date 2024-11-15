#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
import numpy as np
from ninja.disassembly_api import * 

class DisassemblyDemo():
    def __init__(self,module_ids,real_machine):
        self.module_ids = module_ids
        self.real_machine = real_machine
        rospy.loginfo(self.module_ids)
    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded','interupted'])
        with sm_top:
            for i,item in enumerate(self.module_ids):
                if(i == len(self.module_ids)-1):
                    continue
                target_follower_id = self.module_ids[i]
                target_leader_id = self.module_ids[i+1]
                motion_prefix = str(target_follower_id) + '_' + str(target_leader_id)
                sub_sm = smach.StateMachine(outcomes=['succeeded_'+motion_prefix])
                direction = 1 if target_follower_id > target_leader_id else -1
                with sub_sm:
                    smach.StateMachine.add('SwitchState'+  motion_prefix,
                                           SwitchState(robot_name = 'ninja'+str(target_follower_id), robot_id = target_follower_id,real_machine=self.real_machine, neighboring = 'ninja'+str(target_leader_id), neighboring_id = target_leader_id, separate_dir = direction),
                                           transitions={'done':'SeparateState'+motion_prefix})

                    smach.StateMachine.add('SeparateState'+motion_prefix,
                                           SeparateState(robot_name = 'ninja'+str(target_follower_id), robot_id = target_follower_id, neighboring = 'ninja'+str(target_leader_id)),
                                           transitions={'done':'succeeded_'+motion_prefix,'in_process':'SeparateState'+motion_prefix})
                if(i == len(self.module_ids)-2):
                    smach.StateMachine.add('SUB'+str(i),
                                           sub_sm,
                                           transitions={'succeeded_'+motion_prefix:'succeeded'})
                else:
                    smach.StateMachine.add('SUB'+str(i),
                                           sub_sm,
                                           transitions={'succeeded_'+motion_prefix:'SUB'+str(i+1)})

        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()
if __name__ == '__main__':
    rospy.init_node("disassembly_motion")
    modules_str = rospy.get_param("module_ids", default="")
    rm = rospy.get_param("real_machine", default=True)
    modules = []
    if(modules_str):
        modules = [int(x) for x in modules_str.split(',')]
    else:
        rospy.loginfo("No module ID is designated!")
    try:
        disassembly_motion = DisassemblyDemo(module_ids = modules, real_machine=rm);
        disassembly_motion.main()
    except rospy.ROSInterruptException: pass
