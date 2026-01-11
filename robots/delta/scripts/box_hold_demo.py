#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from aerial_robot_model.srv import AddExtraModule, AddExtraModuleRequest


class BoxHoldDemoNode:
    def __init__(self):
        self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        self.update_hz = 50
        self.position = 1.0

        self.srv_name = "add_extra_module"
        rospy.loginfo(f"Waiting for service: {self.srv_name}")
        rospy.wait_for_service(self.srv_name)
        self.add_extra_module = rospy.ServiceProxy(self.srv_name, AddExtraModule)

    def call_add_extra_module(self) -> bool:
        req = AddExtraModuleRequest()
        req.action = 1
        req.module_name = "box"
        req.parent_link_name = "soft_link3"

        # transform
        req.transform.translation.x = 0.1175
        req.transform.translation.y = 0.0
        req.transform.translation.z = 0.0
        req.transform.rotation.x = 0.0
        req.transform.rotation.y = 0.0
        req.transform.rotation.z = 0.0
        req.transform.rotation.w = 1.0

        # inertia
        req.inertia.m = 0.25
        req.inertia.com.x = 0.0
        req.inertia.com.y = 0.0
        req.inertia.com.z = 0.0
        req.inertia.ixx = 0.0001
        req.inertia.ixy = 0.0
        req.inertia.ixz = 0.0
        req.inertia.iyy = 0.0001
        req.inertia.iyz = 0.0
        req.inertia.izz = 0.0001

        try:
            resp = self.add_extra_module(req)
            rospy.loginfo(f"add_extra_module status: {resp.status}")
            return bool(resp.status)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def run(self):
        rospy.sleep(0.5)

        r = rospy.Rate(self.update_hz)
        while not rospy.is_shutdown():
            joint_msg = JointState()
            joint_msg.name = ["joint1", "joint3"]
            joint_msg.position = [self.position, self.position]
            joint_msg.header.stamp = rospy.Time.now()
            self.joint_states_pub.publish(joint_msg)
            print("current position:", self.position)
            self.position += 0.5

            if self.position > 1.6:
                rospy.sleep(1.0)
                # self.call_add_extra_module()
                break
            
            if self.position < 1.15:
                # rospy.sleep(5.0)
                user_input = input("Press Enter to continue...")
                # if user_input != "":
                #     rospy.loginfo("Demo interrupted by user.")
                #     return
             
            # rospy.sleep(0.5)
            r.sleep()



def main():
    rospy.init_node("box_hold_demo_node")
    node = BoxHoldDemoNode()
    node.run()

if __name__ == "__main__":
    main()
