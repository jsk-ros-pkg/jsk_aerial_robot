#!/usr/bin/env python3

import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
import tf
from aerial_robot_msgs.msg import ControlInput, WrenchAllocationMatrix
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from spinal.msg import FourAxisCommand, TorqueAllocationMatrixInv
from pid import PI_D
from mujoco_flight_controller import mujocoFlightController
from mujoco_navigation import mujocoNavigator
import time

class MujocoRosInterface:
    def __init__(self):
        rospy.init_node("mujoco_ros_interface", anonymous=True)

        #init mujoco model
        xml_path = rospy.get_param('~model')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # mujoco model parameter
        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        self.joint_names = self.joint_names[1:] # remove root
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        self.site_names = [self.model.site(i).name for i in range(self.model.nsite)]
        self.control_input = [0] * self.model.nu

        mujoco.mj_step(self.model, self.data)

        # paramter for control
        self.cog = None
        self.mass = None
        self.wrench_allocation_mat = WrenchAllocationMatrix()

        self.pos = np.zeros(3)
        self.rpy = np.zeros(3)
        self.prev_pos =np.zeros(3)
        self.prev_rpy = np.zeros(3)
        self.prev_time = rospy.get_time()

        self.calcCog()

        print("mass=", self.mass)
        print()
        print("joint list=", self.joint_names)
        print()
        print("actuator list=", self.actuator_names)
        print()
        print("rotor list=", self.site_names)

        self.cnt = 0

        # ros publisher
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.mocap_pub = rospy.Publisher("mocap/pose", PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher("twist", TwistStamped, queue_size=1)
        self.wrench_mat_pub = rospy.Publisher("wrench_allocation_mat", WrenchAllocationMatrix, queue_size=1)

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
            time.sleep(0.01)
        self.viewer.close()

    def calcCog(self):
        com_pos = np.array(self.data.xipos)         # (nbody, 3)
        mass = np.array(self.data.cinert)[:, -1]    # (nbody, )
        self.mass = np.sum(mass)
        cog = np.dot(com_pos.transpose(), mass) / self.mass
        return cog

    def calcWrenchMat(self):
        self.cog = self.calcCog()
        qpos = self.data.qpos
        rootlink_quat_from_world = np.zeros(4)
        rootlink_quat_from_world[0] = qpos[4]
        rootlink_quat_from_world[1] = qpos[5]
        rootlink_quat_from_world[2] = qpos[6]
        rootlink_quat_from_world[3] = qpos[3]
        rootlink_rot_mat_from_world = tf.transformations.quaternion_matrix((rootlink_quat_from_world[0], rootlink_quat_from_world[1], rootlink_quat_from_world[2], rootlink_quat_from_world[3]))[0:3, 0:3] # rotation of rootlink and cog is same
        rotors_origin_from_cog_in_world = self.data.site_xpos - self.cog  # (4, 3)
        rotors_origin_from_cog_in_cog = np.dot(rootlink_rot_mat_from_world.T, rotors_origin_from_cog_in_world.T).T
        rotors_rot_mat_from_world = self.data.site_xmat.reshape(len(self.site_names), 3, 3) # 4, 3, 3
        rotors_rot_mat_from_cog = np.ones_like(rotors_rot_mat_from_world)
        for i in range(rotors_rot_mat_from_world.shape[0]):
            rotors_rot_mat_from_cog[i] = np.dot(np.linalg.inv(rootlink_rot_mat_from_world), rotors_rot_mat_from_world[i])
        rotors_normal_in_cog = rotors_rot_mat_from_cog[:, :, 2]

        wrench_allocation_mat = WrenchAllocationMatrix()
        for i in range(len(self.site_names)):
            wrench_allocation_mat.f_z.append(1)
            rotor_i_torque_allocation_vector = np.cross(rotors_origin_from_cog_in_cog[i], rotors_normal_in_cog[i]) + 0.02 * (i % 2 * 2 - 1) * rotors_normal_in_cog[i]
            wrench_allocation_mat.t_x.append(rotor_i_torque_allocation_vector[0])
            wrench_allocation_mat.t_y.append(rotor_i_torque_allocation_vector[1])
            wrench_allocation_mat.t_z.append(rotor_i_torque_allocation_vector[2])
            self.wrench_allocation_mat = wrench_allocation_mat

    def ctrlCallback(self, msg):
        for actuator_name, actuator_input in zip(msg.name, msg.input):
            if not (actuator_name in self.actuator_names):
                rospy.logwarn("%s is an invalid actuator name" % (actuator_name))
            else:
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
        qpos = self.data.qpos
        quat = [0] * 4
        quat[0] = qpos[4]
        quat[1] = qpos[5]
        quat[2] = qpos[6]
        quat[3] = qpos[3]
        ps.pose.position.x = qpos[0]
        ps.pose.position.y = qpos[1]
        ps.pose.position.z = qpos[2]
        ps.pose.orientation.x = qpos[4]
        ps.pose.orientation.y = qpos[5]
        ps.pose.orientation.z = qpos[6]
        ps.pose.orientation.w = qpos[3]
        self.mocap_pub.publish(ps)

        #twist
        twist = TwistStamped()
        qvel = self.data.qvel
        twist.header.stamp = now
        twist.twist.linear.x = qvel[0]
        twist.twist.linear.y = qvel[1]
        twist.twist.linear.z = qvel[2]
        twist.twist.angular.x = qvel[3]
        twist.twist.angular.y = qvel[4]
        twist.twist.angular.z = qvel[5]
        self.twist_pub.publish(twist)

        # wrench allocation matrix
        self.calcWrenchMat()
        self.wrench_mat_pub.publish(self.wrench_allocation_mat)

        self.prev_pos = self.pos
        self.prev_rpy = self.rpy
        self.prev_time = now_time


if __name__ == '__main__':
    node = MujocoRosInterface()

    rospy.spin()
