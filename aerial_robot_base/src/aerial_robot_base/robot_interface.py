#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)

# Copyright (c) 2024, DRAGON Laboratory, The University of Tokyo
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

import math
import numpy as np
import rospy

import ros_numpy as ros_np
import tf2_ros
from tf.transformations import *
import rosgraph

from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from geometry_msgs.msg import PoseStamped, Wrench, Vector3, Vector3Stamped, WrenchStamped, Quaternion, QuaternionStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Empty, Int8, UInt8, String
from std_srvs.srv import SetBool, SetBoolRequest

class RobotInterface(object):
    def __init__(self, robot_ns = "", debug_view = False, init_node = False):


        # flight states:
        # TODO: get from header file
        self.ARM_OFF_STATE = 0
        self.START_STATE = 1
        self.ARM_ON_STATE = 2
        self.TAKEOFF_STATE = 3
        self.LAND_STATE = 4
        self.HOVER_STATE = 5
        self.STOP_STATE = 6

        self.debug_view = debug_view
        self.joint_state = JointState()
        self.cog_odom = None
        self.base_odom = None
        self.flight_state = None
        self.target_pos = np.array([0,0,0])

        if init_node:
            rospy.init_node("rotor_interface")

        self.default_pos_thresh = rospy.get_param('~default_pos_thresh', [0.1, 0.1, 0.1]) # m
        self.default_rot_thresh = rospy.get_param('~default_rot_thresh', [0.05, 0.05, 0.05]) # rad
        self.default_vel_thresh = rospy.get_param('~default_vel_thresh', [0.05, 0.05, 0.05]) # rad

        self.robot_ns = robot_ns
        if not self.robot_ns:
            master = rosgraph.Master('/rostopic')
            try:
                _, subs, _ = master.getSystemState()
            except socket.error:
                raise ROSTopicIOException("Unable to communicate with master!")

            teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
            if len(teleop_topics) == 1:
                self.robot_ns = teleop_topics[0].split('/teleop')[0]

        # Teleop
        self.start_pub = rospy.Publisher(self.robot_ns + '/teleop_command/start', Empty, queue_size = 1)
        self.takeoff_pub = rospy.Publisher(self.robot_ns + '/teleop_command/takeoff', Empty, queue_size = 1)
        self.land_pub = rospy.Publisher(self.robot_ns + '/teleop_command/land', Empty, queue_size = 1)
        self.force_landing_pub = rospy.Publisher(self.robot_ns + '/teleop_command/force_landing', Empty, queue_size = 1)
        self.halt_pub = rospy.Publisher(self.robot_ns + '/teleop_command/halt', Empty, queue_size = 1)

        # Odometry&Control
        self.cog_odom_sub = rospy.Subscriber(self.robot_ns + '/uav/cog/odom', Odometry, self.cogOdomCallback)
        self.base_odom_sub = rospy.Subscriber(self.robot_ns + '/uav/baselink/odom', Odometry, self.baseOdomCallback)
        self.control_pid_sub = rospy.Subscriber(self.robot_ns + '/debug/pose/pid', PoseControlPid, self.controlPidCallback)

        # Navigatoin
        self.flight_state_sub = rospy.Subscriber(self.robot_ns + '/flight_state', UInt8, self.flightStateCallback)
        self.traj_nav_pub = rospy.Publisher(self.robot_ns + '/target_pose', PoseStamped, queue_size = 1)
        self.direct_nav_pub = rospy.Publisher(self.robot_ns + '/uav/nav', FlightNav, queue_size = 1)
        self.final_rot_pub = rospy.Publisher(self.robot_ns + '/final_target_baselink_rpy', Vector3Stamped, queue_size = 1)

        # Joint
        self.joint_state_sub = rospy.Subscriber(self.robot_ns + '/joint_states', JointState, self.jointStateCallback)
        self.joint_ctrl_pub = rospy.Publisher(self.robot_ns + '/joints_ctrl', JointState, queue_size = 1)
        self.set_joint_torque_client = rospy.ServiceProxy(self.robot_ns + '/joints/torque_enable', SetBool)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        start_time = rospy.get_time()
        while not rospy.is_shutdown():
            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > 5.0:
                rospy.logerr("cannot connect to {}".format(self.robot_ns))
                break

            if self.base_odom is not None:
                rospy.loginfo("connect to {}!".format(self.robot_ns))
                break

            rospy.sleep(0.1)

    def convergenceCheck(self, timeout, func, *args, **kwargs):

        # if timeout is -1, it means no time constraint
        if timeout < 0:
            return True

        # check convergence
        start_time = rospy.get_time()

        while not rospy.is_shutdown():

            if self.flight_state != self.HOVER_STATE and self.flight_state != self.ARM_OFF_STATE:
                rospy.logwarn("[{}]: preempt because current flight state({}) allows no more motion".format(func.__name__, self.flight_state))
                return False

            ret = func(*args, **kwargs)

            if ret:
                rospy.loginfo("[{}]: convergence".format(func.__name__))
                return True

            elapsed_time = rospy.get_time() - start_time
            if elapsed_time > timeout:
                rospy.logwarn("[{}]: timeout, cannot convergence".format(func.__name__))
                return False

            rospy.sleep(0.1)

        return False

    def start(self, sleep = 1.0):
        self.start_pub.publish()
        rospy.sleep(sleep)

    def takeoff(self):
        self.takeoff_pub.publish()

    def land(self):
        self.land_pub.publish()

    def forceLanding(self):
        self.force_landing_pub.publish()

    def halt(self):
        self.halt_pub.publish()

    def baseOdomCallback(self, msg):
        self.base_odom = msg

    def cogOdomCallback(self, msg):
        self.cog_odom = msg

    def flightStateCallback(self, msg):
        self.flight_state = msg.data

    def controlPidCallback(self, msg):
        self.control_pid = msg

    def getControlPid(self):
        return self.control_pid

    def getBaseOdom(self):
        return self.base_odom

    def getCogOdom(self):
        return self.cog_odom

    def getBasePos(self):
        return ros_np.numpify(self.base_odom.pose.pose.position)

    def getBaseRot(self):
        return ros_np.numpify(self.base_odom.pose.pose.orientation)

    def getBaseRPY(self):
        return euler_from_quaternion(self.getBaseRot())

    def getBaseLinearVel(self):
        return ros_np.numpify(self.base_odom.twist.twist.linear)

    def getBaseAngularVel(self):
        return ros_np.numpify(self.base_odom.twist.twist.angular)

    def getCogPos(self):
        return ros_np.numpify(self.cog_odom.pose.pose.position)

    def getCogRot(self):
        return ros_np.numpify(self.cog_odom.pose.pose.orientation)

    def getCogRPY(self):
        return euler_from_quaternion(self.getCogRot())

    def getCogLinVel(self):
        return ros_np.numpify(self.cog_odom.twist.twist.linear)

    def getCogAngVel(self):
        return ros_np.numpify(self.cog_odom.twist.twist.angular)

    def getFlightState(self):
        return self.flight_state

    def getTargetPos(self):
        return self.target_pos

    def goPos(self, pos, pos_thresh = 0.1, vel_thresh = 0.05, timeout = 30):

        return self.navigate(pos = pos, pos_thresh = pos_thresh, vel_thresh = vel_thresh, timeout = timeout)

    def rotateYaw(self, yaw, yaw_thresh = 0.1, timeout = 30):

        rot = quaternion_from_euler(0, 0, yaw)
        return self.rotate(rot, yaw_thresh, timeout)

    def rotate(self, rot, rot_thresh = 0.1, timeout = 30):

        return self.navigate(rot = rot, rot_thresh = rot_thresh, timeout = timeout)

    def goPosYaw(self, pos, yaw, pos_thresh = 0.1, vel_thresh = 0.05, yaw_thresh = 0.1, timeout = 30):

        rot = quaternion_from_euler(0, 0, yaw)
        return self.goPose(pos, rot, pos_thresh, vel_thresh, yaw_thresh, timeout)

    def goPose(self, pos, rot, pos_thresh = 0.1, vel_thresh = 0.05, rot_thresh = 0.1, timeout = 30):

        return self.navigate(pos = pos, rot = rot, pos_thresh = pos_thresh, vel_thresh = vel_thresh, rot_thresh = rot_thresh, timeout = timeout)

    def goVel(self, vel):
        return self.navigate(lin_vel = vel)

    def rotateVel(self, vel):
        return self.navigate(ang_vel = vel)

    def trajectoryNavigate(self, pos, rot):

        if self.flight_state != self.HOVER_STATE:
            rospy.logerr("[Navigate] the current flight state({}) does not allow navigation".format(self.flight_state))
            return

        if pos is None:
            pos = self.getCogPos()
        if rot is None:
            rot = self.getCogRot()

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]
        self.traj_nav_pub.publish(msg)


    def directNavigate(self, pos, rot, lin_vel, ang_vel):

        if self.flight_state != self.HOVER_STATE:
            rospy.logerr("[Navigate] the current flight state({}) does not allow navigation".format(self.flight_state))
            return

        msg = FlightNav()
        msg.header.stamp = rospy.Time.now()

        msg.control_frame = FlightNav.WORLD_FRAME
        msg.target = FlightNav.COG

        pos_mode = FlightNav.NO_NAVIGATION
        if pos is None:
            if lin_vel is not None:
                pos_mode = FlightNav.VEL_MODE
        else:
            if lin_vel is None:
                pos_mode = FlightNav.POS_MODE
            else:
                pos_mode = FlightNav.POS_VEL_MODE

        rot_mode = FlightNav.NO_NAVIGATION
        if rot is None:
            if ang_vel is not None:
                rot_mode = FlightNav.VEL_MODE
        else:
            if ang_vel is None:
                rot_mode = FlightNav.POS_MODE
            else:
                rot_mode = FlightNav.POS_VEL_MODE


        if pos is None:
            pos = self.getCogPos()

        if rot is None:
            rot = self.getCogRot()
        _, _, yaw = euler_from_quaternion(rot)

        if lin_vel is None:
            lin_vel = np.array([0, 0, 0])

        if ang_vel is None:
            ang_vel = np.array([0, 0, 0])

        msg.pos_xy_nav_mode = pos_mode
        msg.target_pos_x = pos[0]
        msg.target_pos_y = pos[1]
        msg.target_vel_x = lin_vel[0]
        msg.target_vel_y = lin_vel[1]
        msg.pos_z_nav_mode = pos_mode
        msg.target_pos_z = pos[2]
        msg.target_vel_z = lin_vel[2]
        msg.yaw_nav_mode = rot_mode
        msg.target_yaw = yaw
        msg.target_omega_z = ang_vel[2]

        self.direct_nav_pub.publish(msg)

    def navigate(self, pos = None, rot = None, lin_vel = None, ang_vel = None, pos_thresh = 0.1, vel_thresh = 0, rot_thresh = 0, timeout = -1):

        if self.flight_state != self.HOVER_STATE:
            rospy.logerr("[Navigate] the current flight state({}) does not allow navigation".format(self.flight_state))
            return False

        if rot is not None:

            # change rpy to quaternion
            if len(rot) == 3:
                rot = quaternion_from_euler(*rot)

        # check whether the pose is final one
        if lin_vel is None and ang_vel is None:
            self.trajectoryNavigate(pos, rot)
        else:
            self.directNavigate(pos, rot, lin_vel, ang_vel)

        return self.poseConvergenceCheck(timeout, target_pos = pos, target_rot = rot, pos_thresh = pos_thresh, vel_thresh = vel_thresh, rot_thresh = rot_thresh)

    def poseConvergenceCheck(self, timeout, target_pos = None, target_rot = None, pos_thresh = None, vel_thresh = None, rot_thresh = None):

        return self.convergenceCheck(timeout, self.poseThresholdCheck, target_pos, target_rot, pos_thresh, vel_thresh, rot_thresh)

    def poseThresholdCheck(self, target_pos = None, target_rot = None, pos_thresh = None, vel_thresh = None, rot_thresh = None):

        # threshold
        if pos_thresh is None:
            pos_thresh = self.default_pos_thresh

        if isinstance(pos_thresh, float):
            pos_thresh = [pos_thresh] * 3

        if rot_thresh is None:
            rot_thresh = self.default_rot_thresh

        if isinstance(rot_thresh, float):
            rot_thresh = [rot_thresh] * 3

        if vel_thresh is None:
            vel_thresh = self.default_vel_thresh

        if isinstance(vel_thresh, float):
            vel_thresh = [vel_thresh] * 3

        # target pose
        if target_pos is None:
            target_pos = self.getCogPos()
            pos_thresh = np.array([1e6] * 3)
            vel_thresh = np.array([1e6] * 3)

        if target_rot is None:
            target_rot = self.getBaseRot() # assume the coordinate axes of baselink are identical to those of CoG
            rot_thresh = np.array([1e6] * 3)
        if len(target_rot) == 3:
            target_rot = quaternion_from_euler(*target_rot)


        # current pose
        current_pos = self.getCogPos()
        current_vel = self.getCogLinVel()
        current_rot = self.getBaseRot() # assume the coordinate axes of baselink are identical to those of CoG

        # delta_pos
        delta_pos = target_pos - current_pos
        delta_vel = current_vel
        delta_rot = quaternion_multiply(quaternion_inverse(current_rot), \
                                        target_rot)
        delta_rot = euler_from_quaternion(delta_rot)

        rospy.loginfo_throttle(1.0, 'pose diff: {}, rot: {}, vel: {};  target pose: pos: {}, rot: {}; current pose: \n  pos: {}, rot: {}, vel: {}'.format(delta_pos, delta_rot, delta_vel, target_pos, target_rot, current_pos, current_rot, current_vel))

        if np.all(np.abs(delta_pos) < pos_thresh) and \
           np.all(np.abs(delta_rot) < rot_thresh) and \
           np.all(np.abs(delta_vel) < vel_thresh):
            return True
        else:
            return False

    # speical rorataion function for DRAGON like robot
    def rotateCog(self, roll, pitch):
        # send the target roll and pitch angles
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.vector.x = roll
        msg.vector.y = pitch
        self.final_rot_pub.publish(msg)


    def getTF(self, frame_id, wait=0.5, parent_frame_id='world'):
        trans = self.tf_buffer.lookup_transform(parent_frame_id, frame_id, rospy.Time.now(), rospy.Duration(wait))
        return trans


    def jointStateCallback(self, msg):
        joint_state = JointState()

        # only extract joint, exclude other component like gimbal
        for n, j in zip(msg.name, msg.position):
            if 'joint' not in n:
                continue
            joint_state.name.append(n)
            joint_state.position.append(j)
        self.joint_state = joint_state


    def getJointState(self):
        return self.joint_state

    def setJointAngle(self, target_joint_names, target_joint_angles, thresh = 0.05, timeout = -1):

        if self.flight_state != self.HOVER_STATE and self.flight_state != self.ARM_OFF_STATE:
            rospy.logerr("[Send Joint] the current flight state({}) does not allow joint motion ".format(self.flight_state))
            return False


        if len(target_joint_names) != len(target_joint_angles):
            rospy.logerr("[Send Joint] the size of joint names {} and angles {} are not same".format(len(target_joint_names), len(target_joint_angles)))
            return False

        # check whether the joint exists
        for name in target_joint_names:
            if name not in self.joint_state.name:
                rospy.logerr("set jont angle: cannot find {}".format(name))
                return False

        # send joint angles
        target_joint_state = JointState()
        target_joint_state.name = target_joint_names
        target_joint_state.position = target_joint_angles
        self.joint_ctrl_pub.publish(target_joint_state)

        return self.jointConvergenceCheck(timeout, target_joint_names, target_joint_angles, thresh)

    def jointConvergenceCheck(self, timeout, target_joint_names, target_joint_angles, thresh):

        return self.convergenceCheck(timeout, self.jointThresholdCheck, target_joint_names, target_joint_angles, thresh)

    def jointThresholdCheck(self, target_joint_names, target_joint_angles, thresh):

        if len(target_joint_names) != len(target_joint_angles):
            rospy.logerr("[Send Joint] the sizes of joint names {} and angles {} are not same".format(len(target_joint_names), len(target_joint_angles)))
            return False

        # check whether the joint exists, and create index map
        index_map = []
        for i, name in enumerate(target_joint_names):
            try:
                j = self.joint_state.name.index(name)
            except ValueError:
                rospy.logerr("set jont angle: cannot find {}".format(name))
                return False

            index_map.append(j)

        delta_ang = []
        for index, target_ang in zip(index_map, target_joint_angles):
            current_ang = self.joint_state.position[index]
            delta = target_ang - current_ang
            delta_ang.append(delta)

        rospy.loginfo_throttle(1.0, "delta angle: {}".format(delta_ang))

        if np.all(np.abs(delta_ang) < thresh):
            return True
        else:
            return False

    def setJointTorque(self, state):
        req = SetBoolRequest()
        req.data = state
        try:
            self.set_joint_torque_client(req)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

if __name__ == "__main__":
      from IPython import embed
      ri = RobotInterface(init_node=True)
      embed()
