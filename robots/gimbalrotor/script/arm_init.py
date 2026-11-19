#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState
from spinal.msg import ServoControlCmd


def main():
    rospy.init_node("publish_arm_commands_once", anonymous=True)

    joints_pub = rospy.Publisher(
        "/gimbalrotor/joints_ctrl",
        JointState,
        queue_size=1
    )

    servo_pub = rospy.Publisher(
        "/gimbalrotor/servo/target_states",
        ServoControlCmd,
        queue_size=1
    )

    # Publisher接続待ち
    rospy.sleep(0.5)

    # =========================================================
    # 1. /arm/joints_ctrl を publish
    # =========================================================
    joint_msg = JointState()

    joint_msg.header.seq = 0
    joint_msg.header.stamp = rospy.Time(0)
    joint_msg.header.frame_id = ""

    joint_msg.name = [
        "arm_joint1_joint1",
        "arm_joint2_joint1",
        "arm_joint3_joint1",
        "arm_joint4_joint1",
        "arm_joint5_joint1",
    ]

    # joint_msg.position = [0.0, 0.0, 0.0, 0.48, 3.0]
    joint_msg.position = [0.0, 0.0, 0.0, 0.0, 3.0]
    joint_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
    joint_msg.effort = []

    joints_pub.publish(joint_msg)

    # joint_msg.position = [0.0, -0.5, -0.5, 0.4]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)

    # =========================================================
    # 2. /arm/servo/target_states を publish
    # =========================================================

    # servo_msg = ServoControlCmd()
    # servo_msg.index = [8]
    # servo_msg.angles = [4100]
    # servo_pub.publish(servo_msg)

    # publish直後の終了で取りこぼさないよう少し待つ
    # rospy.sleep(1)

    # joint_msg.position = [0.0, 0.0, 0.0, 0.48, -3.0]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(2)

    # joint_msg.position = [0.0, -0.5, -0.5, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(2)

    # joint_msg.position = [0.0, -0.25, -0.25, -0.4]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(2)

    # joint_msg.position = [0.0, -0, -0, -0.4]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(3)

    # joint_msg.position = [0.0, -0, -0, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(3)

    # servo_msg.angles = [1200]
    # servo_pub.publish(servo_msg)
    # rospy.sleep(2)

    # joint_msg.position = [0.0, -0, -0, -0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)
    
    # joint_msg.position = [0.0, -0, -0, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.2, 0.1, 0.1, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.4, 0.2, 0.2, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.6, 0.3, 0.3, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 0.4, 0.4, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 0.75, 0.75, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)
    
    # joint_msg.position = [-0.8, 1.1, 1.1, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 1.25, 1.25, -0.3]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 1.4, 1.4, -0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 1.7, 1.7, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.5, 1.7, 1.7, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.2, 1.7, 1.7, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # servo_msg.angles = [4100]
    # servo_pub.publish(servo_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.5, 1.7, 1.7, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 1.7, 1.7, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 1.4, 1.4, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 1.1, 1.1, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 0.8, 0.8, 0.1]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

    # joint_msg.position = [-0.8, 0.8, 0.8, 0.45]
    # joint_msg.header.stamp = rospy.Time(0)
    # joints_pub.publish(joint_msg)
    # rospy.sleep(1)

if __name__ == "__main__":
    main()
