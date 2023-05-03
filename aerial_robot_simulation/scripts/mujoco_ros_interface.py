#!/usr/bin/env python3
import rospy
import mujoco
from mujoco import viewer
import os
import numpy as np
# from aerial_robot_msgs.msg import ControlInput

class MujocoRosInterface:
    def __init__(self):
        rospy.init_node("mujoco_ros_interface", anonymous=True)

        #init mujoco model
        xml_path = rospy.get_param('~model')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        viewer.launch(self.model, self.data)

        # self.joint_names = [self.model.joint(i).name for i in range(self.model.njnt)]
        # self.joint_names = self.joint_names[1:] # remove root
        # print(self.joint_names)

        # self.actuator_names = [self.model.actuator(i).name for i in range(self.model.nu)]
        # print(self.actuator_names)

        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "rotor1_thrust")
        print(actuator_id)

        # ros subscriber
        # ctrl_sub = rospy.Subscriber("~ctrl_input", ControlInput , self.ctrlCallback)

        self.control_input = [0] * self.model.nu

        while not rospy.is_shutdown():
            mujoco.mj_step(self.model, self.data)


    # def ctrlCallback(self, msg):
    #     for actuator_name, actuator_input in zip(msg.name, msg.input):
    #         if not (actuator_name in self.actuator_names):
    #             rospy.logwarn("%s is an invalid actuator name" % (actuator_name))
    #         else:
    #             actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
    #             self.control_input[actuator_id] = actuator_input

if __name__ == '__main__':
    node = MujocoRosInterface()

    rospy.spin()
