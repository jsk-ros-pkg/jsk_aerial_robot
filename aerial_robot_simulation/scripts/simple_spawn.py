#!/usr/bin/env python3
import rospy
import mujoco
from mujoco import viewer
import os

class SimpleSpawn:
    def __init__(self):
        rospy.init_node("simple_spawn", anonymous=True)

        #init mujoco model
        xml_path = rospy.get_param('~model')
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        viewer.launch(self.model, self.data)

        while not rospy.is_shutdown():
            mujoco.mj_step(self.model, self.data)

if __name__ == '__main__':
    node = SimpleSpawn()

    rospy.spin()
