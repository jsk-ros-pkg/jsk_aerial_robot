#!/usr/bin/env python3

import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
import tf
from aerial_robot_msgs.msg import ControlInput
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from spinal.msg import FourAxisCommand, TorqueAllocationMatrixInv
from pid import PI_D
from mujoco_flight_controller import mujocoFlightController
from mujoco_navigation import mujocoNavigator

class MujocoRosInterface:
    def __init__(self):
        rospy.init_node("mujoco_ros_interface", anonymous=True)

        #init mujoco model
        xml_path = rospy.get_param('~model')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        self.joint_names = self.joint_names[1:] # remove root
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        self.site_names = [self.model.site(i).name for i in range(self.model.nsite)]

        mujoco.mj_step(self.model, self.data)

        self.cog = None
        self.q_mat = np.zeros((4, len(self.site_names)))
        self.mass = None

        self.pos = np.zeros(3)
        self.rpy = np.zeros(3)
        self.prev_pos =np.zeros(3)
        self.prev_rpy = np.zeros(3)
        self.prev_time = rospy.get_time()

        self.calcCog()
        print(self.mass)
        print(self.joint_names)
        print(self.actuator_names)
        print(self.site_names)

        self.cnt = 0

        self.control_input = [0] * self.model.nu

        # ros publisher
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.mocap_pub = rospy.Publisher("mocap/pose", PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher("twist", TwistStamped, queue_size=1)

        # ros subscriber
        ctrl_sub = rospy.Subscriber("ctrl_input", ControlInput, self.ctrlCallback)

        # ros timer
        timer100 = rospy.Timer(rospy.Duration(0.01), self.timerCallback)

        self.viewer = viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

        while not rospy.is_shutdown():
            self.cog = self.calcCog()
            for i in range(len(self.control_input)):
                self.data.ctrl[i] = self.control_input[i]
            self.viewer.sync()
            mujoco.mj_step(self.model, self.data)

        self.viewer.close()

    def calcCog(self):
        com_pos = np.array(self.data.xipos)
        mass = np.array(self.data.cinert)[:, -1]
        self.mass = np.sum(mass)
        cog = np.dot(com_pos.transpose(), mass) / self.mass
        return cog

    def calcWrenchMat(self):
        self.cog = self.calcCog()
        rotors_origin_from_cog = self.data.site_xpos - self.cog
        rotors_rot_mat_from_world = self.data.site_xmat.reshape(len(self.site_names), 3, 3) # 4, 3, 3
        baselink_rot_mat_from_world = self.data.xmat[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link2")].reshape(3, 3) # rotation of baselink and cog is same
        self.baselink_rpy = tf.transformations.euler_from_matrix(baselink_rot_mat_from_world)
        rotors_rot_mat_from_cog = np.ones_like(rotors_rot_mat_from_world)
        for i in range(rotors_rot_mat_from_world.shape[0]):
            rotors_rot_mat_from_cog[i] = np.dot(np.linalg.inv(baselink_rot_mat_from_world), rotors_rot_mat_from_world[i])
        rotors_normal_from_cog = rotors_rot_mat_from_cog[:, :, 2]

        for i in range(len(self.site_names)):
            self.q_mat[0, i] = 1 / self.mass
            self.q_mat[1:4, i] = np.cross(rotors_origin_from_cog[i], rotors_normal_from_cog[i]) + (-0.01) * (i % 2 * 2 - 1) * rotors_normal_from_cog[i]

    def ctrlCallback(self, msg):
        for actuator_name, actuator_input in zip(msg.name, msg.input):
            if not (actuator_name in self.actuator_names):
                rospy.logwarn("%s is an invalid actuator name" % (actuator_name))
            else:
                rospy.logwarn("%s is set" %(actuator_name))
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                self.control_input[actuator_id] = actuator_input


    def timerCallback(self, event):
        self.cnt = (self.cnt + 1) % 20
        now = rospy.Time.now()
        now_time = rospy.get_time()
        # print(now_time)
        # print(now)
        # print(self.prev_time)
        # joint state
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        joint_pos = self.data.qpos # including root (7 elements in the head of data)
        joint_pos_addr = self.model.jnt_qposadr
        joint_vel = self.data.qvel # including root (6 elements in the head of data)
        joint_vel_addr = self.model.jnt_dofadr
        joint_force = self.data.actuator_force

        for i in range(len(self.joint_names)):
            js.position.append(joint_pos[i + 7])
            js.velocity.append(joint_vel[i + 6])
            # js.position.append(joint_pos[joint_pos_addr[i]])
            # js.velocity.append(joint_vel[joint_vel_addr[i]])
            js.effort.append(joint_force[i])
        self.joint_state_pub.publish(js)

        # mocap (pos and quat)
        ps = PoseStamped()
        ps.header.stamp = now
        baselink_pos  = self.data.xpos[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link2")]
        baselink_quat = self.data.xquat[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "link2")]
        ps.pose.position.x = baselink_pos[0]
        ps.pose.position.y = baselink_pos[1]
        ps.pose.position.z = baselink_pos[2]
        ps.pose.orientation.x = baselink_quat[1]
        ps.pose.orientation.y = baselink_quat[2]
        ps.pose.orientation.z = baselink_quat[3]
        ps.pose.orientation.w = baselink_quat[0]
        self.mocap_pub.publish(ps)

        self.prev_pos = self.pos
        self.pos = baselink_pos

        self.prev_rpy = self.rpy
        self.rpy = tf.transformations.euler_from_quaternion((baselink_quat[1], baselink_quat[2], baselink_quat[3], baselink_quat[0]))

        #twist
        twist = TwistStamped()
        twist.header.stamp = now
        twist.twist.linear.x = (self.pos[0] - self.prev_pos[0])  / (now_time - self.prev_time)
        twist.twist.linear.y = (self.pos[1] - self.prev_pos[1]) / (now_time - self.prev_time)
        twist.twist.linear.z = (self.pos[2] - self.prev_pos[2]) / (now_time - self.prev_time)
        twist.twist.angular.x = (self.rpy[0] - self.prev_rpy[0]) / (now_time - self.prev_time)
        twist.twist.angular.y = (self.rpy[1] - self.prev_rpy[1]) / (now_time - self.prev_time)
        twist.twist.angular.z = (self.rpy[2] - self.prev_rpy[2]) / (now_time - self.prev_time)
        self.twist_pub.publish(twist)

        # print(self.pos[0] , self.prev_pos[0], self.pos[0] - self.prev_pos[0])
        # print(self.rpy[0], self.prev_rpy[0], self.rpy[0] - self.prev_rpy[0])
        # print()
        self.prev_time = now_time

if __name__ == '__main__':
    node = MujocoRosInterface()

    rospy.spin()
