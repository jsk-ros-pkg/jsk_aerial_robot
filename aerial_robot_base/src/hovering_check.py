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

import sys
import unittest

import rospy
import rostest
from aerial_robot_base.state_machine import *
from std_msgs.msg import Empty


PKG = 'rostest'

# Template for simple demo
class HoverMotion():
    def __init__(self):
        self.robot = RobotInterface()

        self.sm_top = smach.StateMachine(outcomes=['succeeded', 'preempted'])
        self.sm_top.userdata.flags = {}
        self.sm_top.userdata.extra = {}

        with self.sm_top:
            smach.StateMachine.add('Start', Start(self.robot),
                                   transitions={'succeeded':'Arm',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})


            smach.StateMachine.add('Arm', Arm(self.robot),
                                   transitions={'succeeded':'Takeoff',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Takeoff', Takeoff(self.robot),
                                   transitions={'succeeded':'WayPoint',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})


            smach.StateMachine.add('WayPoint', WayPoint(self.robot),
                                   transitions={'succeeded':'Land',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Land', Land(self.robot),
                                   transitions={'succeeded':'succeeded',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            self.sis = smach_ros.IntrospectionServer('task_smach_server', self.sm_top, '/SM_ROOT')


    def startMotion(self):
        self.sis.start()
        outcome = self.sm_top.execute()

        self.sis.stop()

        if outcome == 'succeeded':
            return True
        else:
            return False


class HoveringTest(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node("hovering_test")

    def test_hovering(self):
        checker = HoverMotion()
        self.assertTrue(checker.startMotion())

if __name__ == '__main__':
    print("start check hovering")
    try:
        rostest.run(PKG, 'hovering_check', HoveringTest, sys.argv)
    except KeyboardInterrupt:
        pass

    print("exiting")
