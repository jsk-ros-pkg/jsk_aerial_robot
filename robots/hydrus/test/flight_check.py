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
import math

import rospy
import rostest
from aerial_robot_base.state_machine import *
from aerial_robot_msgs.msg import PoseControlPid

PKG = 'rostest'

class FlightMotion():
    def __init__(self):
        self.robot = RobotInterface()

        self.control_term_sub = rospy.Subscriber('debug/pose/pid', PoseControlPid, self.controlTermCallback)
        self.divergent_flag = False
        self.pose_tresh = rospy.get_param('~stable_check/pose_thresh', [0.05, 0.05, 0.05]) # xy, z, yaw
        self.max_err_xy = 0
        self.max_err_x = 0
        self.max_err_y = 0
        self.max_err_z = 0
        self.max_err_yaw = 0

        joint_names = self.robot.getJointState().name

        self.sm_top = smach.StateMachine(outcomes=['succeeded', 'preempted'])
        self.sm_top.userdata.flags = {}
        self.sm_top.userdata.extra = {}

        with self.sm_top:
            smach.StateMachine.add('Start', Start(self.robot),
                                   transitions={'succeeded':'FormCheck',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})


            smach.StateMachine.add('FormCheck', FormCheck(self.robot, 'init_form'),
                                   transitions={'succeeded':'Arm',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Arm', Arm(self.robot),
                                   transitions={'succeeded':'Takeoff',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Takeoff', Takeoff(self.robot),
                                   transitions={'succeeded':'Motion',
                                                'preempted':'preempted'},
                                   remapping={'flags':'flags', 'extra':'extra'})

            smach.StateMachine.add('Motion', Transform(self.robot),
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

        rospy.loginfo("max pose errors in hovering is [xy, z, yaw] are [%f (%f, %f), %f, %f]", self.max_err_xy, self.max_err_x, self.max_err_y, self.max_err_z, self.max_err_yaw)
        if self.divergent_flag:
            rospy.logerr("because of devergence, the test is filaed")
            outcome = 'premmpted'

        if outcome == 'succeeded':
            return True
        else:
            return False


    def controlTermCallback(self, msg):

        if self.robot.getFlightState() != self.robot.HOVER_STATE:
            return

        if self.divergent_flag:
            return

        err_x = msg.x.err_p
        err_y = msg.y.err_p
        err_z = msg.z.err_p
        err_yaw = msg.yaw.err_p
        err_xy = math.sqrt(err_x * err_x + err_y * err_y)

        if err_xy > self.max_err_xy:
            self.max_err_xy = err_xy
            self.max_err_x = err_x
            self.max_err_y = err_y
        if math.fabs(err_z) > math.fabs(self.max_err_z):
            self.max_err_z = err_z
        if math.fabs(err_yaw) > math.fabs(self.max_err_yaw):
            self.max_err_yaw = err_yaw

        # check the control stability
        if err_xy > self.pose_tresh[0] or math.fabs(err_z) > self.pose_tresh[1] or math.fabs(err_yaw) > self.pose_tresh[2]:
            rospy.logerr("devergence in [xy, z, yaw]: [%f (%f, %f), %f, %f]", err_xy, err_x, err_y, err_z, err_yaw)
            self.divergent_flag = True


class FlightTest(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node("flight_test")

    def test_flight(self):
        checker = FlightMotion()
        try:
            self.assertTrue(checker.startMotion())
        except ValueError as e:
            print("e")
            self.assertTrue(False)

if __name__ == '__main__':
    print("start check flight")
    try:
        rostest.run(PKG, 'flight_check', FlightTest, sys.argv)
    except KeyboardInterrupt:
        pass

    print("exiting")
