#!/usr/bin/env python3
import rospy
import mujoco
import os

class SimpleSpawn:
    def __init__(self):
        rospy.init_node("simple_spawn", anonymous=True)

        #init mujoco model
        mj_path = mujoco_py.utils.discover_mujoco()
        xml_path = rospy.get_param('~model')
        model = mujoco_py.load_model_from_path(xml_path)
        self.sim = mujoco_py.MjSim(model)

        viewer = mujoco_py.MjViewer(self.sim)

        while not rospy.is_shutdown():
            self.sim.step()
            viewer.render()

if __name__ == '__main__':
    node = SimpleSpawn()

    rospy.spin()
