#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2019, JSK Robotics Laboratory, The University of Tokyo
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

import os
import sys
import rospy
import unittest
import rostest
import operator
from functools import reduce
import shlex
import subprocess
import math
from aerial_robot_base.hovering_check import HoveringCheck
from sensor_msgs.msg import JointState

PKG = 'rostest'

class InitFormCheck():
    def __init__(self, name="init_form_check"):
        self.init_joint_names = rospy.get_param('~init_joint_names', [])
        self.init_joint_angles = rospy.get_param('~init_joint_angles', [])
        self.convergence_duration = rospy.get_param('~init_form_duration', 10.0)
        self.angle_threshold = rospy.get_param('~init_angle_threshold', 0.02) # rad

        self.joint_angle_sub = rospy.Subscriber('joint_states', JointState, self._jointCallback)
        self.joint_msg = None
        self.joint_map = []

    def initFormCheck(self):
        deadline = rospy.Time.now() + rospy.Duration(self.convergence_duration)
        while not rospy.Time.now() > deadline:
            if self.joint_msg is not None:
                joint_angles = [self.joint_msg.position[i] for i in self.joint_map]
                rospy.loginfo_throttle(1, '%s: %s' %  (self.init_joint_names, joint_angles))

            rospy.sleep(1.0)

        # check convergence
        joint_angles = [self.joint_msg.position[i] for i in self.joint_map]

        angles_diff = [math.fabs(x - y) < self.angle_threshold for (x, y) in zip(self.init_joint_angles, joint_angles)]
        rospy.loginfo('joint angles convergence: %s: %s' %  (self.init_joint_names, angles_diff))
        if reduce(operator.mul, angles_diff):
            return True
        else:
            # cannot be convergent
            return False

    def _jointCallback(self, msg):
        if self.joint_msg == None:
            self.joint_map = [msg.name.index(i) for i in self.init_joint_names]

        self.joint_msg = msg

class TransformCheck(InitFormCheck, HoveringCheck):
    def __init__(self, name="init_form_check"):
        InitFormCheck.__init__(self)
        HoveringCheck.__init__(self)

        # list of the transform task
        self.params = rospy.get_param('~tasks', [])

        self.joint_ctrl_sub = rospy.Subscriber('joints_ctrl', JointState, self._jointCtrlCallback)
        self.joint_ctrl_pub = rospy.Publisher('joints_ctrl', JointState, queue_size=1)

        self.target_joint_angles = self.init_joint_angles

    def transformCheck(self):
        rospy.sleep(0.5)

        for param in self.params:
            index = self.params.index(param) + 1
            print("start task%d" % index)
            task = {'timeout': 10.0, 'threshold': [0.01, 0.01, 0.01], 'angle_threshold': 0.01, 'reset': None, 'reset_duration': 10.0}

            task.update(param)

            print(task)

            deadline = rospy.Time.now() + rospy.Duration(task['timeout'])

            node_pid = None
            if isinstance(task['command'], list):
                print("joint ctrl")
                # the target joint angles
                assert len(task['command']) == len(self.init_joint_angles), 'the length of target joint angles from init_joint_anlges is wrong'

                self.target_joint_angles = task['command']
                joint_ctrl_msg = JointState()
                joint_ctrl_msg.header.stamp = rospy.Time.now()
                joint_ctrl_msg.position = self.target_joint_angles
                self.joint_ctrl_pub.publish(joint_ctrl_msg)
            else:
                print("string command")
                # the rosrun command (TODO, the roslaunch)
                node_command = shlex.split(task['command'])
                assert node_command[0] == 'rosrun' or node_command[0] == 'rostopic', 'please use rosrun command'
                # start the rosnode
                node_pid = subprocess.Popen(node_command)

            max_error_xy = 0
            max_error_x = 0
            max_error_y = 0
            max_error_z = 0
            max_error_yaw = 0
            while not rospy.Time.now() > deadline:
                rospy.sleep(0.1)

                if self.joint_msg is None or self.control_msg is None:
                    continue

                err_x = self.control_msg.x.err_p
                err_y = self.control_msg.y.err_p
                err_z = self.control_msg.z.err_p
                err_yaw = self.control_msg.yaw.err_p
                err_xy = math.sqrt(err_x * err_x + err_y * err_y)

                joint_angles = [self.joint_msg.position[i] for i in self.joint_map]
                rospy.loginfo_throttle(1, "errors in [xy, z, yaw]: [{} ({}, {}), {}, {}]; joint angles: {}".format(err_xy, err_x, err_y, err_z, err_yaw, joint_angles))

                if err_xy > max_error_xy:
                    max_error_xy = err_xy
                    max_error_x = err_x
                    max_error_y = err_y
                if math.fabs(err_z) > math.fabs(max_error_z):
                    max_error_z = err_z
                if math.fabs(err_yaw) > math.fabs(max_error_yaw):
                    max_error_yaw = err_yaw

                # check the control stability
                if err_xy > task['threshold'][0] or math.fabs(err_z) > task['threshold'][1] or math.fabs(err_yaw) > task['threshold'][2]:
                    rospy.logwarn("devergence in [xy, z, yaw]: [%f (%f, %f), %f, %f]", err_xy, err_x, err_y, err_z, err_yaw)
                    if node_pid:
                        node_pid.kill()
                    return False

            if node_pid:
                node_pid.kill()

            rospy.loginfo("max errors in [xy, z, yaw] are [%f (%f, %f), %f, %f]", max_error_xy, max_error_x, max_error_y, max_error_z, max_error_yaw)

            # check joint angles convergence
            joint_angles = [self.joint_msg.position[i] for i in self.joint_map]
            angles_diff = [math.fabs(x - y) < task['angle_threshold'] for (x, y) in zip(self.target_joint_angles, joint_angles)]
            if not reduce(operator.mul, angles_diff):
                rospy.logwarn('angles diff is devergent: %s: %s' %  (self.init_joint_names, [(x - y) for (x,y) in zip(self.target_joint_angles, joint_angles)]))
                # cannot be convergent
                return False

            node_pid = None
            # reset the form
            if isinstance(task['reset'], list):
                # the target joint angles
                assert len(task['reset']) == len(self.init_joint_angles), 'the length of target joint angles from init_joint_anlges is wrong'

                self.target_joint_angles = task['reset']
                joint_ctrl_msg = JointState()
                joint_ctrl_msg.header.stamp = rospy.Time.now()
                joint_ctrl_msg.position = self.target_joint_angles
                self.joint_ctrl_pub.publish(joint_ctrl_msg)

                rospy.sleep(task['reset_duration'])
                rospy.loginfo("reset the form")

            elif isinstance(task['reset'], str):
                # the rosrun command (TODO, the roslaunch)
                node_command = shlex.split(task['reset'])
                assert node_command[0] == 'rosrun', 'please use rosrun command'
                # start the rosnode
                node_pid = subprocess.Popen(node_command)

                rospy.sleep(task['reset_duration'])
                rospy.loginfo("reset the form")

                node_pid.kill()

        return True

    def _jointCtrlCallback(self, msg):
        if len(msg.position) == len(self.target_joint_angles):
            self.target_joint_angles = msg.position
        else:
            for i, postion in enumerate(msg.position):
                index = self.init_joint_names.index(msg.name[i])
                self.target_joint_angles[index] = msg.position[i]


class ControlTest(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node("control_test")

    def test_control(self):
        # step1: check the init form convergence
        init_form_checker = InitFormCheck()
        self.assertTrue(init_form_checker.initFormCheck(), msg = 'Cannot reach convergence for initial form')

        # steup2: check hovering
        hovering_checker = HoveringCheck()
        self.assertTrue(hovering_checker.hoveringCheck(), msg = 'Cannot reach convergence for hovering')

        # steup3: check transformation
        transform_checker = TransformCheck()
        self.assertTrue(transform_checker.transformCheck(), msg = 'Cannot reach convergence for transforming hovering')


if __name__ == '__main__':
    print("start check flight control for hydrus")
    try:
        rostest.run(PKG, 'control_check', ControlTest, sys.argv)
    except KeyboardInterrupt:
        pass

    print("exiting")

