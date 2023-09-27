#!/usr/bin/env python

import rospy
import smach
import smach_ros
import math
from std_msgs.msg import Empty, String, Bool
from aerial_robot_msgs.msg import FlightNav
from aerial_robot_msgs.msg import FlightNav
from spinal.msg import DesireCoord
from spinal.msg import ServoControlCmd
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf

#### state classes ####

"""""""""""""""""""""""""""
Switch -> Separate -> Stop
"""""""""""""""""""""""""""

class SwitchState(smach.State):
    # release docking mechanism and switch control mode
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','in_process'])
        self.servo_msg = ServoControlCmd()

    def execute(self, userdata):
        if 1:
            return 'in_process'
        else:
            return 'done'

    def maleMechRelease(self):
        
        
        

class SeparateState(smach.State):
    # keep away target robot from leader
    def __init__(self):
        smach.State.__init__(self, outcomes=['done','in_process','fail'])
    def execute(self, userdata):
        if 1:
            return 'in_process'
        else:
            return 'done'

class StopState(smach.State):
    # stop and hovering
    def __init__(self):
        smach.State.__init__(self, outcomes=['done'])
    def execute(self, userdata):
        return 'done'

#### main class ####
class DisassembleDemo():
    def __init__(self):
        rospy.init_node("disassemble_demo")

    def main(self):
        sm_top = smach.StateMachine(outcomes=['succeeded'])
        with sm_top:
            smach.StateMachine.add('StandbyState', StandbyState(), transitions={'done':'ApproachState', 'in_process':'StandbyState'})
            smach.StateMachine.add('ApproachState', ApproachState(), transitions={'done':'AssemblyState', 'in_process':'ApproachState', 'fail':'StandbyState'})
            smach.StateMachine.add('AssemblyState', AssemblyState(), transitions={'done':'succeeded'})
 
        sis = smach_ros.IntrospectionServer('smach_server', sm_top, '/SM_ROOT')
        sis.start()
        outcome = sm_top.execute()
        rospy.spin()
        sis.stop()

if __name__ == '__main__':
    try:
        disassemble_demo = DisassembleDemo();
        disassemble_demo.main()
    except rospy.ROSInterruptException: pass
