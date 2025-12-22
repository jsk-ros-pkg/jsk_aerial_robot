#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class PubSoftLinkJointStatesNode:
    def __init__(self):
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_cb, queue_size=1)
        if self.is_simulation():
            self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        else:
            self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

    
    def is_simulation(self) -> bool:
        if rospy.get_param('/use_sim_time', False):
            return True
        try:
            m = rosgraph.Master('probe')
            pubs, subs, srvs = m.getSystemState()
            pub_topics = {t for t, _ in pubs}
            return ('/gazebo/model_states' in pub_topics) or ('/clock' in pub_topics)
        except Exception:
            return False


    def joint_states_cb(self, msg: JointState):
        rigid_joints = [None, None, None]  # joint1, joint2, joint3
        
        for n, j in zip(msg.name, msg.position):
            if n == 'joint1':
                rigid_joints[0] = j
            elif n == 'joint2':
                rigid_joints[1] = j
            elif n == 'joint3':
                rigid_joints[2] = j

        if None in rigid_joints:
            rospy.logwarn("Not all rigid joints found in joint states")
            print(rigid_joints)
            return
        
        print("rigid joints all found")
        
        soft_joint_msg = JointState()
        soft_joint_msg.name = ["soft_joint2", "soft_joint3", "soft_joint5", "soft_joint6"]

        soft_joint_msg.position = [0.0] * 4
        soft_joint_msg.position[0] = -2.48 * rigid_joints[1] + 3.8
        soft_joint_msg.position[3] = -2.48 * rigid_joints[1] + 3.8
        soft_joint_angle = (3.14*2 - (rigid_joints[0] + rigid_joints[1] + rigid_joints[2] + soft_joint_msg.position[0] + soft_joint_msg.position[3])) / 2
        soft_joint_msg.position[1] = soft_joint_angle
        soft_joint_msg.position[2] = soft_joint_angle
        print("rigid joints:", rigid_joints)
        print("soft joint angles:", soft_joint_msg.position)
        soft_joint_msg.header.stamp = rospy.Time.now()
        self.joint_states_pub.publish(soft_joint_msg)

def main():
    rospy.init_node("pub_soft_link_joint_states")
    PubSoftLinkJointStatesNode()
    rospy.spin()

if __name__ == "__main__":
    main()
