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
import smach
import smach_ros
import functools
import numpy as np
from aerial_robot_base.robot_interface import RobotInterface
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy

class BaseState(smach.State):
    def __init__(self, robot, outcomes=[], input_keys=[], output_keys=[], io_keys=['flags', 'extra']):
        smach.State.__init__(self, outcomes, input_keys, output_keys, io_keys)
        self.robot = robot

    def hold(self, t, flags):

        if 'interfere_mode' not in flags:
            rospy.sleep(t)
        else:
            interfere_mode = flags['interfere_mode']

            if not interfere_mode:
                rospy.sleep(t)
            else:
                message = '\n\n Please press "L1" and "R1" at the same time to proceed the state \n'
                rospy.loginfo('\033[32m'+ message +'\033[0m')

                while not rospy.is_shutdown():

                    if flags['interfering']:
                        flags['interfering'] = False # reset for subsequent state
                        break

                    rospy.sleep(0.1)



# trigger to start state machine
class Start(BaseState):
    def __init__(self, robot):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.flags = {'interfere_mode': False, 'interfering': False}

        self.task_start = False
        self.task_start_sub = rospy.Subscriber('/task_start', Empty, self.taskStartCallback)
        self.joy_sub = rospy.Subscriber(self.robot.robot_ns + '/joy', Joy, self.joyCallback)

    def taskStartCallback(self, msg):
        self.task_start = True

    def joyCallback(self, msg):

        # Check interfering from joy
        interfere_flag = False

        # This is PS4 Joy Controller
        if len(msg.axes) == 14 and len(msg.buttons) == 14:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:

                interfere_flag = True

        # This is RoG1 Controller
        if len(msg.axes) == 8 and len(msg.buttons) == 11:
            if msg.buttons[4] == 1 and msg.buttons[5] == 1:

                interfere_flag = True

        if interfere_flag:
            if not self.flags['interfere_mode']:
                rospy.loginfo("Enter interfere mode")
                self.flags['interfere_mode'] = True

        self.flags['interfering'] = interfere_flag

    def execute(self, userdata):

        message = \
        '\n\n  Please run following command to start the state machine: \n' + \
        '  "$ rostopic pub -1 /task_start std_msgs/Empty" \n \n' + \
        '  Or you can press "L1" and "R1" at the same time to enter interfere mode, \n' + \
        '  which need to use controller like PS4 to manually proceed the state. \n'
        rospy.loginfo('\033[32m'+ message +'\033[0m')

        userdata.flags = self.flags

        while not self.task_start:
            rospy.sleep(0.1)
            rospy.logdebug_throttle(1.0, "wait to start task")

            if userdata.flags['interfere_mode']:
                rospy.loginfo(self.__class__.__name__ + ': enter the inteferece mode')
                userdata.flags['interfering'] = False # reset for subsequent state
                break

            if rospy.is_shutdown():
                print("shutdown ros")
                return 'preempted'

            pass

        rospy.loginfo("start task")
        return 'succeeded'

# the template state for arm, takeoff, land
class SingleCommandState(BaseState):
    def __init__(self, robot, prefix, func, start_flight_state, target_flight_state, timeout, hold_time):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.prefix = prefix
        self.func = func
        self.start_flight_state = start_flight_state
        self.target_flight_state = target_flight_state
        self.hold_time = rospy.get_param('~' + self.prefix + '/hold_time', hold_time)
        self.timeout = rospy.get_param('~' + self.prefix + '/timeout', timeout)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.start_flight_state:
            rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at the start_flight_state ({}). preempted!'.format(flight_state, self.start_flight_state))
            return 'preempted'

        rospy.loginfo(self.__class__.__name__ + ': start {}'.format(self.prefix))
        self.func()

        start_t = rospy.get_time()
        while rospy.get_time() < start_t + self.timeout:

            flight_state = self.robot.getFlightState()
            if flight_state == self.target_flight_state:
                rospy.loginfo(self.__class__.__name__ + ': robot succeed to {}!'.format(self.prefix))
                self.hold(self.hold_time, userdata.flags)
                return 'succeeded'

            if rospy.is_shutdown():
                print("shutdown ros")
                return 'preempted'

        rospy.logwarn(self.__class__.__name__ + ': timeout ({}sec). preempted!'.format(self.timeout))
        return 'preempted'


class Arm(SingleCommandState):
    def __init__(self, robot):
        SingleCommandState.__init__(self, robot, 'arm', robot.start, robot.ARM_OFF_STATE, robot.ARM_ON_STATE, 2.0, 2.0)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state == self.robot.HOVER_STATE:
            rospy.loginfo(self.__class__.__name__ + ': robot already hovers, skip')
            return 'succeeded'

        return SingleCommandState.execute(self, userdata)

class Takeoff(SingleCommandState):
    def __init__(self, robot):
        SingleCommandState.__init__(self, robot, 'takeoff', robot.takeoff, robot.ARM_ON_STATE, robot.HOVER_STATE, 30.0, 2.0)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state == self.robot.HOVER_STATE:
            rospy.loginfo(self.__class__.__name__ + ': robot already hovers, skip')
            return 'succeeded'

        return SingleCommandState.execute(self, userdata)

class Land(SingleCommandState):
    def __init__(self, robot):
        SingleCommandState.__init__(self, robot, 'land', robot.land, robot.HOVER_STATE, robot.ARM_OFF_STATE, 20.0, 0)


# waypoint
class WayPoint(BaseState):
    def __init__(self, robot, prefix = 'waypoint', waypoints = [], timeout = 30.0, hold_time = 1.0):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.waypoints = rospy.get_param('~' + prefix + '/waypoints', waypoints)
        self.timeout = rospy.get_param('~' + prefix + '/timeout', timeout)
        self.hold_time = rospy.get_param('~' + prefix + '/hold_time', hold_time)
        self.pos_thresh = rospy.get_param('~' + prefix+ '/pos_thresh', 0.1)
        self.vel_thresh = rospy.get_param('~' + prefix + '/vel_thresh', 0.05)
        self.yaw_thresh = rospy.get_param('~' + prefix + '/yaw_thresh', 0.1)

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.robot.HOVER_STATE:
            rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(flight_state))
            return 'preempted'

        if len(self.waypoints) == 0:
            rospy.logwarn(self.__class__.__name__ + ': the waypoints is empty. preempted')
            return 'preempted'

        for i, waypoint in enumerate(self.waypoints):

            if len(waypoint) == 3:
                # only position
                ret = self.robot.goPos(pos = waypoint, pos_thresh = self.pos_thresh, vel_thresh = self.vel_thresh, timeout = self.timeout)
            elif len(waypoint) == 4:
                # position + yaw
                ret = self.robot.goPosYaw(pos = waypoint[:3], yaw = waypoint[-1], pos_thresh = self.pos_thresh, vel_thresh = self.vel_thresh, yaw_thresh = self.yaw_thresh, timeout = self.timeout)
            else:
                rospy.logwarn(self.__class__.__name__ + ': the format of waypoint {} is not supported, the size should be either 3 and 4. preempted!'.format(waypoint))
                return 'preempted'

            if ret:
                rospy.loginfo(self.__class__.__name__ + ': robot succeeds to reach the {}th waypoint [{}]'.format(i+1, waypoint))
                self.hold(self.hold_time, userdata.flags)
            else:
                rospy.logwarn(self.__class__.__name__ + ': robot cannot reach {}. preempted'.format(waypoint))
                return 'preempted'


        return 'succeeded'


class CircleTrajectory(BaseState):
    def __init__(self, robot, period = 20.0, radius = 1.0, init_theta = 0.0, yaw = True, loop = 1, timeout = 30.0, hold_time = 2.0):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.period = rospy.get_param("~circle/period", period)
        self.radius = rospy.get_param("~circle/radius", radius)
        self.init_theta = rospy.get_param("~circle/init_theta", init_theta)
        self.yaw = rospy.get_param("~circle/yaw", yaw)
        self.loop = rospy.get_param("~circle/loop", loop)
        self.hold_time = rospy.get_param('~circle/hold_time', hold_time)

        self.omega = 2 * np.pi / self.period
        self.velocity = self.omega * self.radius

        self.nav_rate = rospy.get_param("~nav_rate", 20.0) # hz
        self.nav_rate = 1 / self.nav_rate

    def execute(self, userdata):

        current_pos = self.robot.getCogPos()
        center_pos_x = current_pos[0] - np.cos(self.init_theta) * self.radius
        center_pos_y = current_pos[1] - np.sin(self.init_theta) * self.radius
        center_pos_z = current_pos[2]

        init_yaw = self.robot.getCogRPY()[2]

        rospy.loginfo("the center position is [%f, %f]", center_pos_x, center_pos_y)

        loop = 0
        cnt = 0

        while loop < self.loop:

            if self.robot.flight_state != self.robot.HOVER_STATE:
                rospy.logerr("[Circle Motion] the robot is not hovering, preempt!")
                return 'preempted'

            theta = self.init_theta + cnt * self.nav_rate * self.omega

            pos_x = center_pos_x + np.cos(theta) * self.radius
            pos_y = center_pos_y + np.sin(theta) * self.radius
            pos = [pos_x, pos_y, center_pos_z]
            vel_x = -np.sin(theta) * self.velocity
            vel_y = np.cos(theta) * self.velocity
            vel = [vel_x, vel_y, 0]

            if self.yaw:
                yaw = init_yaw + cnt * self.nav_rate * self.omega
                rot = [0, 0, yaw]
                ang_vel = [0, 0, self.omega]
                self.robot.navigate(pos = pos, rot = rot, lin_vel = vel, ang_vel = ang_vel)
            else:
                self.robot.navigate(pos = pos, lin_vel = vel)

            cnt += 1

            if cnt == self.period // self.nav_rate:
                loop += 1
                cnt = 0

            rospy.sleep(self.nav_rate)

        # stop
        self.robot.navigate(lin_vel = [0,0,0], ang_vel = [0,0,0])
        self.hold(self.hold_time, userdata.flags)

        return 'succeeded'


# Joint
class FormCheck(BaseState):
    def __init__(self, robot, prefix = 'form_check', joint_names = [], joint_angles = [], thresh = 0.02, timeout = 10.0):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.target_joint_names = rospy.get_param('~' + prefix + '/joint_names', joint_names)
        self.target_joint_angles = rospy.get_param('~' + prefix + '/joint_angles', joint_angles)
        self.timeout = rospy.get_param('~' + prefix + '/timeout', timeout)
        self.thresh = rospy.get_param('~' + prefix + '/angle_thresh', thresh)

    def execute(self, userdata):

        # check convergence
        ret = self.robot.jointConvergenceCheck(self.timeout, self.target_joint_names, self.target_joint_angles, self.thresh)

        if ret:
            rospy.loginfo(self.__class__.__name__ + ': robot succeed to convernge to the target joints {}:{}!'.format(self.target_joint_names, self.target_joint_angles))
            return 'succeeded'
        else:
            rospy.logwarn(self.__class__.__name__ + ': timeout ({}sec). preempted!'.format(self.timeout))
            return 'preempted'


class Transform(BaseState):
    def __init__(self, robot, prefix = 'transform', joint_names = [], joint_trajectory = [], thresh = 0.05, timeout = 10.0, hold_time = 2.0):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.target_joint_names = rospy.get_param('~' + prefix + '/joint_names', joint_names)
        self.target_joint_trajectory = rospy.get_param('~' + prefix + '/joint_trajectory', joint_trajectory)
        self.timeout = rospy.get_param('~' + prefix + '/timeout', timeout)
        self.hold_time = rospy.get_param('~' + prefix + '/hold_time', hold_time)
        self.thresh = rospy.get_param('~' + prefix + '/angle_thresh', thresh)


    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.robot.HOVER_STATE:
            rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(flight_state))
            return 'preempted'

        if len(self.target_joint_trajectory) == 0:
            rospy.logwarn(self.__class__.__name__ + ': the joint trajectory is empty. preempted')
            return 'preempted'

        for i, target_angles in enumerate(self.target_joint_trajectory):
            ret = self.robot.setJointAngle(self.target_joint_names, target_angles, self.thresh, self.timeout)

            if ret:
                rospy.loginfo(self.__class__.__name__ + ': robot succeed to convernge to the {} th target joints {}:{}!'.format(i + 1, self.target_joint_names, target_angles))
                self.hold(self.hold_time, userdata.flags)
            else:
                rospy.logwarn(self.__class__.__name__ + ': timeout ({}sec), fail to reach the {} th target joints. preempted!'.format(i+1, self.timeout))
                return 'preempted'


        return 'succeeded'

# Joint + Pose
class TransformWithPose(BaseState):
    def __init__(self, robot, prefix = 'motion', joint_names = [], joint_trajectory = [], pos_trajectory = [], rot_trajectory = [], joint_thresh = 0.05, pos_thresh = 0.1, rot_thresh = 0.1, timeout = 10.0, hold_time = 2.0, rotate_cog = False):
        BaseState.__init__(self, robot, outcomes=['succeeded', 'preempted'])

        self.target_joint_names = rospy.get_param('~' + prefix + '/joint_names', joint_names)
        self.target_joint_trajectory = rospy.get_param('~' + prefix + '/joint_trajectory', joint_trajectory)
        self.target_pos_trajectory = rospy.get_param('~' + prefix + '/pos_trajectory', pos_trajectory)
        self.target_rot_trajectory = rospy.get_param('~' + prefix + '/rot_trajectory', rot_trajectory)
        self.timeout = rospy.get_param('~' + prefix + '/timeout', timeout)
        self.hold_time = rospy.get_param('~' + prefix + '/hold_time', hold_time)
        self.joint_thresh = rospy.get_param('~' + prefix + '/joint_thresh', joint_thresh)
        self.pos_thresh = rospy.get_param('~' + prefix + '/pos_thresh', pos_thresh)
        self.rot_thresh = rospy.get_param('~' + prefix + '/rot_thresh', rot_thresh)
        self.rotate_cog = rospy.get_param('~' + prefix + '/rotate_cog', rotate_cog) # speical rotation for CoG

    def execute(self, userdata):

        flight_state = self.robot.getFlightState()

        if flight_state != self.robot.HOVER_STATE:
            rospy.logwarn(self.__class__.__name__ + ': the robot state ({}) is not at HOVER_STATE. preempted!'.format(flight_state))
            return 'preempted'

        if len(self.target_joint_trajectory) == 0:
            rospy.logwarn(self.__class__.__name__ + ': the joint trajectory is empty. preempted')
            return 'preempted'

        for i, target_angles in enumerate(self.target_joint_trajectory):

            # send target position
            target_pos = None
            if len(self.target_pos_trajectory) > 0:

                if len(self.target_pos_trajectory) != len(self.target_joint_trajectory):
                    rospy.logwarn(self.__class__.__name__ + ': the size of joint trajectory and that of pos is not equal. preempted!')
                    return 'preempted'

                target_pos = self.target_pos_trajectory[i]
                self.robot.goPos(target_pos, timeout = 0)

            # send target rotation
            target_rot = None
            if len(self.target_rot_trajectory) > 0:
                if len(self.target_rot_trajectory) != len(self.target_joint_trajectory):
                    rospy.logwarn(self.__class__.__name__ + ': the size of joint trajectory and that of rot is not equal. preempted!')
                    return 'preempted'

                target_rot = self.target_rot_trajectory[i]

                if self.rotate_cog:

                    if len(target_rot) > 3 or len(target_rot) < 2:
                        rospy.logwarn(self.__class__.__name__ + ': the size of target rot should be 2 or 3 for special cog ratate mode. preempted!')
                        return 'preempted'

                    self.robot.rotateCog(target_rot[0], target_rot[1])

                    if len(target_rot) == 3:
                        self.robot.rotateyaw(target_rot[2], timeout = 0)
                    else:
                        # fill the yaw element for subsequent pose convergence check
                        curr_yaw = self.robot.getCogRPY()[2]
                        target_rot.append(curr_yaw)

                else:
                    self.robot.rotate(target_rot, timeout = 0)

            start_time = rospy.get_time()

            # send joint angles
            rospy.loginfo(self.__class__.__name__ + ': start to change the joint motion {} with pose of [{}, {}]'.format(target_angles, target_pos, target_rot))
            ret = self.robot.setJointAngle(self.target_joint_names, target_angles, self.joint_thresh, self.timeout)

            if ret:
                rospy.loginfo(self.__class__.__name__ + ': robot succeed to convernge to the {} th target joints {}:{}!'.format(i + 1, self.target_joint_names, target_angles))
            else:
                rospy.logwarn(self.__class__.__name__ + ': timeout ({}sec), fail to reach the {} th target joints. preempted!'.format(self.timeout, i+1))
                return 'preempted'


            # reset
            timeout = start_time + self.timeout - rospy.get_time()
            start_time = rospy.get_time()

            # check the pose convergence
            ret = self.robot.poseConvergenceCheck(timeout, target_pos, target_rot, self.pos_thresh, rot_thresh = self.rot_thresh)

            if not ret:
                rospy.logwarn(self.__class__.__name__ + ': timeout ({}sec), fail to reach the {} th target pose. preempted!'.format(i+1, self.timeout))
                return 'preempted'

            rospy.loginfo(self.__class__.__name__ + ': robot succeed to convernge to the {} th target pose {}: [{}, {}]!'.format(i + 1, self.target_joint_names, target_pos, target_rot))

            self.hold(self.hold_time, userdata.flags)

        return 'succeeded'
