#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2025, DRAGON Laboratory, The University of Tokyo
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
import math
import copy

from aerial_robot_base.robot_interface import RobotInterface
from aerial_robot_base.state_machine import *


# Simple Hydrus demo
class HydrusDemo():
    def __init__(self):
        self.robot = RobotInterface()
        rospy.sleep(1.0) # wait for joint updated
        joint_names = self.robot.getJointState().name
        joint_num = len(joint_names)
        nominal_angle = 2 * math.pi / (joint_num + 1)
        init_angles = [nominal_angle] * joint_num
        trajectory = [init_angles]

        self.sm_top = smach.StateMachine(outcomes=['succeeded', 'preempted'])
        self.sm_top.userdata.flags = {}
        self.sm_top.userdata.extra = {}

        with self.sm_top:
            smach.StateMachine.add('Start', Start(self.robot),
                                   transitions={'succeeded':'FormCheck',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})


            smach.StateMachine.add('FormCheck', FormCheck(self.robot, 'init_form', \
                                                          joint_names = joint_names, \
                                                          joint_angles = init_angles, \
                                                          thresh = 0.05),
                                   transitions={'succeeded':'Arm',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Arm', Arm(self.robot),
                                   transitions={'succeeded':'Takeoff',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Takeoff', Takeoff(self.robot),
                                   transitions={'succeeded':'Transform',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            # create a loop motion
            loop = rospy.get_param('~loop', 1)
            for i in range(loop):
                for j in range(2 * joint_num):
                    j = j % joint_num
                    angles = copy.deepcopy(trajectory[-1])
                    angles[j] *= -1
                    trajectory.append(angles)

            smach.StateMachine.add('Transform', Transform(self.robot, 'transfrom', \
                                                          joint_names, trajectory, \
                                                          thresh = 0.05, timeout = 20.0, \
                                                          hold_time = 0.0),
                                   transitions={'succeeded':'Land',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Land', Land(self.robot),
                                   transitions={'succeeded':'succeeded',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            self.sis = smach_ros.IntrospectionServer('task_smach_server', self.sm_top, '/SM_ROOT')

        self.sis.start()
        outcome = self.sm_top.execute()

        self.sis.stop()


if __name__ == '__main__':

    rospy.init_node('hydrus_demo')

    demo = HydrusDemo()
