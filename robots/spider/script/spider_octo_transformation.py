#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import argparse
import numpy as np

class SpiderOctoTransformation:
    def __init__(self):
        rospy.init_node('spider_octo_transformation')
        self.joint_state = JointState()
        self.pub = rospy.Publisher('/spider/joints_ctrl', JointState, queue_size = 1)
        self.sub = rospy.Subscriber('/spider/joint_states', JointState, self.jointStateCallback)

        rospy.sleep(0.5)

    def jointStateCallback(self, msg):
        self.joint_state = msg
        self.sub.unregister()

    def publishJointMsg(self, target_joint_position, execute_time, frequency):

        while not self.joint_state.name:
            pass

        joint_seq_len = int(execute_time * frequency / 1000.0)
        joint_seq = []
        joint_name = ['joint' + str(i/2+1) + ('_yaw' if i % 2 == 0 else '_pitch')  for i in range(16)]

        if joint_seq_len > 1:
            for position, name in zip(target_joint_position, joint_name):
                current_position = self.joint_state.position[self.joint_state.name.index(name)]
                joint_seq.append(np.linspace(current_position, float(position), num = joint_seq_len, endpoint = True))
            joint_seq = np.stack(joint_seq).transpose()
        else:
            joint_seq = [target_joint_position]

        joint_msg = JointState()
        joint_msg.name = joint_name

        for joint_pos in joint_seq:
            joint_msg.header.stamp = rospy.Time.now()
            joint_msg.position = joint_pos
            self.pub.publish(joint_msg)
            rospy.sleep(1.0 / args.f)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-q', nargs='+', help='joint angle [rad]*16')
    parser.add_argument('-t', type=int, help='execute time [ms]')
    parser.add_argument('-f', type=float, help='joint publish frequency [Hz]', default='20.0')
    args = parser.parse_args()

    if len(args.q) != 16 and len(args.q) != 4:
        print("[Error] incorrect number of joint angle, please assign angle with order of (joint_yaw, joint_pitch)")
        exit(0)

    if len(args.q) == 4:
        args.q = args.q * 4

    print("taget joint angles are {}".format(args.q))

    node = SpiderOctoTransformation()
    node.publishJointMsg(args.q, args.t, args.f)
