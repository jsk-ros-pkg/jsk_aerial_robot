#!/usr/bin/env python3

import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
import quaternion
import tf
from aerial_robot_msgs.msg import ControlInput
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import JointState
from spinal.msg import FourAxisCommand, TorqueAllocationMatrixInv, Imu
from pid import PI_D
import time
from rosgraph_msgs.msg import Clock

class MujocoRosInterface:
    def __init__(self):
        #init mujoco model
        xml_path = rospy.get_param('~model')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)

        # mujoco model parameter
        self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        self.joint_names = self.joint_names[1:] # remove root
        self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        # self.site_names = [self.model.site(i).name for i in range(self.model.nsite)]
        self.control_input = [0] * self.model.nu
        self.rotor_list = rospy.get_param('rotor_list')
        print(self.rotor_list)

        mujoco.mj_step(self.model, self.data)

        # paramter for control
        self.mass = None

        self.mass = np.sum(np.array(self.data.cinert)[:, -1])
        print("mass=", self.mass)
        print()
        print("joint list=", self.joint_names)
        print()
        print("actuator list=", self.actuator_names)
        print()

        # ros publisher
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
        self.mocap_pub = rospy.Publisher("mocap/pose", PoseStamped, queue_size=1)
        self.twist_pub = rospy.Publisher("twist", TwistStamped, queue_size=1)
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1)
        self.clock_pub = rospy.Publisher('/clock',Clock, queue_size=10)

        # ros subscriber
        base_thrust_sub = rospy.Subscriber("four_axes/command", FourAxisCommand, self.baseThrustCallback)
        ctrl_sub = rospy.Subscriber("mujoco/ctrl_input", ControlInput, self.ctrlCallback)

        # ros timer
        timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)

        # ros time
        self.sim_clock = Clock()
        self.zero_time = rospy.get_time()
        clock = rospy.Timer(rospy.Duration(0.001), self.clockCallback)

        self.viewer = viewer.launch_passive(self.model, self.data)
        self.viewer.sync()

        while not rospy.is_shutdown():
            for i in range(len(self.control_input)):
                self.data.ctrl[i] = self.control_input[i]
            self.viewer.sync()
            mujoco.mj_step(self.model, self.data)
            time.sleep(0.01)
        self.viewer.close()

    def baseThrustCallback(self, msg):
        if len(self.rotor_list) != len(msg.base_thrust):
            rospy.logwarn("size of rotor name list and input list is not same")
            return
        for i in range(len(self.rotor_list)):
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, self.rotor_list[i])
            self.control_input[actuator_id] = msg.base_thrust[i]


    def ctrlCallback(self, msg):
        if len(msg.name) != len(msg.input):
            rospy.logwarn("size of joint name list and input list is not same")
            return
        for actuator_name, actuator_input in zip(msg.name, msg.input):
            if not (actuator_name in self.actuator_names):
                rospy.logwarn("%s is an invalid actuator name" % (actuator_name))
            else:
                actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
                self.control_input[actuator_id] = actuator_input


    def timerCallback(self, event):
        now = self.getNowTime()

        # joint state
        js = JointState()
        js.header.stamp = self.getNowTime()
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
        fc_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE ,"fc")
        fc_pos = self.data.site_xpos[fc_id]
        fc_rot = self.data.site_xmat[fc_id].reshape(3, 3)
        fc_quat = quaternion.from_rotation_matrix(fc_rot)
        ps = PoseStamped()
        ps.header.stamp = self.getNowTime()
        ps.pose.position.x = fc_pos[0]
        ps.pose.position.y = fc_pos[1]
        ps.pose.position.z = fc_pos[2]
        ps.pose.orientation.x = fc_quat.x
        ps.pose.orientation.y = fc_quat.y
        ps.pose.orientation.z = fc_quat.z
        ps.pose.orientation.w = fc_quat.w
        self.mocap_pub.publish(ps)

        # imu
        rpy = tf.transformations.euler_from_quaternion([fc_quat.x, fc_quat.y, fc_quat.z, fc_quat.w])
        imu = Imu()
        imu.stamp = self.getNowTime()
        for i in range(self.model.nsensor):
            if self.model.sensor(i).name == "acc":
                imu.acc_data = self.data.sensordata[self.model.sensor(i).adr[0]:self.model.sensor(i).adr[0]+self.model.sensor(i).dim[0]]
            if self.model.sensor(i).name == "gyro":
                imu.gyro_data = self.data.sensordata[self.model.sensor(i).adr[0]:self.model.sensor(i).adr[0]+self.model.sensor(i).dim[0]]
            if self.model.sensor(i).name == "mag":
                imu.mag_data = self.data.sensordata[self.model.sensor(i).adr[0]:self.model.sensor(i).adr[0]+self.model.sensor(i).dim[0]]
        imu.angles = rpy
        self.imu_pub.publish(imu)

    def clockCallback(self, event):
        # print("clock")
        self.sim_clock = rospy.Time.from_sec(rospy.get_time() - self.zero_time)
        self.clock_pub.publish(self.sim_clock)

    def getNowTime(self):
        return rospy.Time.from_sec(rospy.get_time() - self.zero_time)

if __name__ == '__main__':
    rospy.init_node("mujoco_ros_interface", anonymous=True)
    node = MujocoRosInterface()

    rospy.spin()
