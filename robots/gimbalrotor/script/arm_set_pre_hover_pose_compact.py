#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState


def main():
    rospy.init_node("publish_arm_commands_once", anonymous=True)

    move_time = rospy.get_param("~move_time", 1.5)

    print(move_time)

    joints_pub = rospy.Publisher(
        "/gimbalrotor/joints_ctrl",
        JointState,
        queue_size=1
    )

    # Publisher接続待ち
    rospy.sleep(0.5)

    joint_msg = JointState()

    joint_msg.header.seq = 0
    joint_msg.header.stamp = rospy.Time(0)
    joint_msg.header.frame_id = ""

    joint_msg.name = [
        "arm_joint3_joint1",
        "arm_joint4_joint1",
        "arm_joint5_joint1",
    ]

    joint_msg.position = [0.3, 0.0, -3.0]
    joint_msg.velocity = [0.0, 0.0, 0.0]
    joint_msg.effort = []
    joints_pub.publish(joint_msg)

    joint_msg.position = [-0.5, -0.2, -3.0]
    joint_msg.header.stamp = rospy.Time(0)
    joints_pub.publish(joint_msg)
    rospy.sleep(move_time)

    joint_msg.position = [0.5, -0.2, -3.0]
    joint_msg.header.stamp = rospy.Time(0)
    joints_pub.publish(joint_msg)
    rospy.sleep(move_time)

    joint_msg.position = [0.0, 0.0, -3.0]
    joint_msg.header.stamp = rospy.Time(0)
    joints_pub.publish(joint_msg)
    rospy.sleep(move_time)

if __name__ == "__main__":
    main()
