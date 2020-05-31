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

from __future__ import print_function

import sys
import time
import math
import unittest

import rospy
import rostest
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import PoseControlPid

PKG = 'rostest'

class HoveringCheck():
    def __init__(self, name="hovering_check"):
        self.hovering_duration = rospy.get_param('~hovering_duration', 0.0)
        self.convergence_thresholds = rospy.get_param('~convergence_thresholds', [0.01, 0.01, 0.01]) # [xy,z, theta]
        self.controller_sub = rospy.Subscriber('debug/pose/pid', PoseControlPid, self._controlCallback)
        self.control_msg = None

    def hoveringCheck(self):

        # start motor arming
        start_pub = rospy.Publisher('teleop_command/start', Empty, queue_size=1)
        takeoff_pub = rospy.Publisher('teleop_command/takeoff', Empty, queue_size=1)

        time.sleep(0.5) # wait for publisher initialization
        start_pub.publish(Empty())

        time.sleep(1.0) # second

        # start takeoff
        takeoff_pub.publish(Empty())

        deadline = rospy.Time.now() + rospy.Duration(self.hovering_duration)
        while not rospy.Time.now() > deadline:
            if self.control_msg is not None:
                err_x = self.control_msg.x.err_p;
                err_y = self.control_msg.y.err_p;
                err_z = self.control_msg.z.err_p;
                err_yaw = self.control_msg.yaw.err_p;
                err_xy = math.sqrt(err_x * err_x + err_y * err_y);
                rospy.loginfo_throttle(1, 'errors in [xy, z, yaw]: [%f (%f, %f), %f, %f]' %  (err_xy, err_x, err_y, err_z, err_yaw))
            rospy.sleep(1.0)

        err_x = self.control_msg.x.err_p;
        err_y = self.control_msg.y.err_p;
        err_z = self.control_msg.z.err_p;
        err_yaw = self.control_msg.yaw.err_p;
        err_xy = math.sqrt(err_x * err_x + err_y * err_y);

        # check convergence
        if err_xy < self.convergence_thresholds[0] and math.fabs(err_z) < self.convergence_thresholds[1] and math.fabs(err_yaw) < self.convergence_thresholds[2]:
            return True

        # cannot be convergent
        rospy.logerr('errors in [xy, z, yaw]: [%f (%f, %f), %f, %f]' %  (err_xy, err_x, err_y, err_z, err_yaw))

        return False

    def _controlCallback(self, msg):
        self.control_msg = msg

        err_x = self.control_msg.x.err_p;
        err_y = self.control_msg.y.err_p;
        err_z = self.control_msg.z.err_p;
        err_yaw = self.control_msg.yaw.err_p;
        err_xy = math.sqrt(err_x * err_x + err_y * err_y);
        rospy.logdebug_throttle(1, 'errors in [xy, z, yaw]: [%f (%f, %f), %f, %f]' %  (err_xy, err_x, err_y, err_z, err_yaw))

class HoveringTest(unittest.TestCase):
    def __init__(self, *args):
        super(self.__class__, self).__init__(*args)
        rospy.init_node("hovering_test")
        self.hovering_delay = rospy.get_param('~hovering_delay', 1.0)

    def test_hovering(self):
        rospy.sleep(self.hovering_delay)
        checker = HoveringCheck()
        assert checker.hoveringCheck(), 'Cannot reach convergence'

if __name__ == '__main__':
    print("start check hovering")
    try:
        rostest.run(PKG, 'hovering_check', HoveringTest, sys.argv)
    except KeyboardInterrupt:
        pass

    print("exiting")
