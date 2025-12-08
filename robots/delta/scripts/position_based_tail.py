#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from spinal.msg import ServoControlCmd
from spinal.msg import PwmTest
import numpy as np
import math
from aerial_robot_base.robot_interface import RobotInterface
from calc_pwm import force_to_pwm
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, JointState


s = 230
d = 5
r_joint_1 = 85 / 2 / math.sqrt(2)
r_joint_2 = 85 / 2
r_wheel = 20

def fk(alpha_1, alpha_2):
    R_0_to_1 = np.array(
        [
            [math.cos(alpha_1), math.sin(alpha_1)],
            [-math.sin(alpha_1), math.cos(alpha_1)],
        ]
    )
    R_2_to_3 = np.array(
        [
            [math.cos(alpha_2), math.sin(alpha_2)],
            [-math.sin(alpha_2), math.cos(alpha_2)],
        ]
    )
    if alpha_1 == 0:
        p_0_to_1 = np.array(
            [
                [0],
                [s + d*2],
            ]
        )
    else:
        r_1 = s / alpha_1
        p_0_to_1 = np.array(
            [
                [r_1 * (1 - math.cos(alpha_1)) + d * math.sin(alpha_1) * 2],
                [r_1 * math.sin(alpha_1) + d * math.cos(alpha_1) * 2],
            ]
        )
    if alpha_2 == 0:
        p_2_to_3 = np.array(
            [
                [0],
                [s + d],
            ]
        )
    else:
        r_3 = s / alpha_2
        p_2_to_3 = np.array(
            [
                [r_3 * (1 - math.cos(alpha_2)) + d * math.sin(alpha_2)],
                [r_3 * math.sin(alpha_2) + d * math.cos(alpha_2)],
            ]
        )
    return (p_0_to_1 + R_0_to_1 @ p_2_to_3)

def solve_ik(alpha_1, alpha_2, p_des):
    for i in range(500):
        print("num of iteration: ", i)
        R_0_to_1 = np.array(
            [
                [math.cos(alpha_1), math.sin(alpha_1)],
                [-math.sin(alpha_1), math.cos(alpha_1)],
            ]
        )
        R_2_to_3 = np.array(
            [
                [math.cos(alpha_2), math.sin(alpha_2)],
                [-math.sin(alpha_2), math.cos(alpha_2)],
            ]
        )

        R_0_to_1_prime = np.array(
            [
                [-math.sin(alpha_1), math.cos(alpha_1)],
                [-math.cos(alpha_1), -math.sin(alpha_1)],
            ]
        )
        R_2_to_3_prime = np.array(
            [
                [-math.sin(alpha_2), math.cos(alpha_2)],
                [-math.cos(alpha_2), -math.sin(alpha_2)],
            ]
        )

        if alpha_1 == 0:
            p_0_to_1 = np.array(
                [
                    [0],
                    [s + d*2],
                ]
            )
            p_0_to_1_prime = np.array(
                [
                    [s / 2 + d*2],
                    [0],
                ]
            )
        else:
            r_1 = s / alpha_1
            p_0_to_1 = np.array(
                [
                    [r_1 * (1 - math.cos(alpha_1)) + d * math.sin(alpha_1) * 2],
                    [r_1 * math.sin(alpha_1) + d * math.cos(alpha_1) * 2],
                ]
            )
            p_0_to_1_prime = np.array(
                [
                    [
                        -s / alpha_1 / alpha_1 * (1 - math.cos(alpha_1))
                        + r_1 * math.sin(alpha_1)
                        + d * math.cos(alpha_1) * 2
                    ],
                    [
                        -s / alpha_1 / alpha_1 * math.sin(alpha_1)
                        + r_1 * math.cos(alpha_1)
                        - d * math.sin(alpha_1) * 2
                    ],
                ]
            )
        if alpha_2 == 0:
            p_2_to_3 = np.array(
                [
                    [0],
                    [s + d],
                ]
            )
            p_2_to_3_prime = np.array(
                [
                    [s / 2 + d],
                    [0],
                ]
            )
        else:
            r_3 = s / alpha_2
            p_2_to_3 = np.array(
                [
                    [r_3 * (1 - math.cos(alpha_2)) + d * math.sin(alpha_2)],
                    [r_3 * math.sin(alpha_2) + d * math.cos(alpha_2)],
                ]
            )
            p_2_to_3_prime = np.array(
                [
                    [
                        -s / alpha_2 / alpha_2 * (1 - math.cos(alpha_2))
                        + r_3 * math.sin(alpha_2)
                        + d * math.cos(alpha_2)
                    ],
                    [
                        -s / alpha_2 / alpha_2 * math.sin(alpha_2)
                        + r_3 * math.cos(alpha_2)
                        - d * math.sin(alpha_2)
                    ],
                ]
            )

        p = (
            p_0_to_1
            + R_0_to_1 @ p_2_to_3
        )

        print("alpha_1: ", math.degrees(alpha_1), "deg")
        print("alpha_2: ", math.degrees(alpha_2), "deg")
        if np.linalg.norm(p_des - p[:1, :]) < 13:
            print("p_des: ")
            print(p_des)
            print("p: ")
            print(p)
            return (alpha_1, alpha_2)

        dp_dalpha_1 = (
            p_0_to_1_prime
            + R_0_to_1_prime @ p_2_to_3
            + R_0_to_1 @ p_2_to_3_prime
        )
        dp_dalpha_2 = (
            R_0_to_1 @ p_2_to_3_prime
        )

        dp_dalpha = np.concatenate((dp_dalpha_1, dp_dalpha_2), axis=1)
        dp_dalpha = dp_dalpha[:1, :] 
        dq = np.dot(np.linalg.pinv(dp_dalpha), (p_des - p[:1, :]) * 0.05)
        # dq = np.dot(np.linalg.pinv(dp_dalpha), (p_des - p)*0.05)

        alpha_1 += dq[0, 0]
        alpha_2 += dq[1, 0]

        if i == 499:
            raise Exception("IK was not solved")


def get_wire_diff(alpha_1, alpha_2):
    divide_num = 8
    def get_plus_pos_wire_length(alpha, r_joint):  # xまたはyが正のワイヤーの長さ
        if alpha == 0:
            return s + d
        r = (s-d*(divide_num/2-1)) / abs(alpha)
        if alpha > 0:
            return divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num) + divide_num/2*d
        else:
            return divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num) + divide_num/2*d

    def get_minus_pos_wire_length(alpha, r_joint):  # xまたはyが負のワイヤーの長さ
        if alpha == 0:
            return s + d
        r = (s-d*(divide_num/2-1)) / abs(alpha)
        if alpha > 0:
            return divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num) + divide_num/2*d
        else:
            return divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num) + divide_num/2*d

    x_plus_long_wire = get_plus_pos_wire_length(alpha_1, r_joint_2) + d + get_plus_pos_wire_length(alpha_2, r_joint_2)
    x_minus_long_wire = get_minus_pos_wire_length(alpha_1, r_joint_2) + d +get_minus_pos_wire_length(alpha_2, r_joint_2)
    x_plus_short_wire = get_plus_pos_wire_length(alpha_1, r_joint_2) + d
    x_minus_short_wire = get_minus_pos_wire_length(alpha_1, r_joint_2) + d

    return (
        x_plus_long_wire - (s + d) * 2 - d,
        x_minus_long_wire - (s + d) * 2- d,
        x_plus_short_wire - (s + d) - d,
        x_minus_short_wire - (s + d) - d,
    )


def get_angle_diff(wire_diff):
    return int(wire_diff / r_wheel / math.pi * 4096)

def is_simulation() -> bool:
    if rospy.get_param('/use_sim_time', False):
        return True
    try:
        m = rosgraph.Master('probe')
        pubs, subs, srvs = m.getSystemState()
        pub_topics = {t for t, _ in pubs}
        return ('/gazebo/model_states' in pub_topics) or ('/clock' in pub_topics)
    except Exception:
        return False


def need_next_input(dest, last_published_target):
    if dest is None:
        return True
    if abs(dest[0] - last_published_target[0]) < math.radians(1.0) and abs(dest[1] - last_published_target[1]) < math.radians(1.0):
        return True
    return False


if __name__ == "__main__":
    rospy.init_node("tail_ik")
    tail_pub = rospy.Publisher("servo/target_states", ServoControlCmd, queue_size=1)
    rotor_pub = rospy.Publisher("pwm_test", PwmTest, queue_size=1)
    soft_joint_pub = rospy.Publisher("joint_states", JointState, queue_size=1)
    servo_controller_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
    joint_angle_pub = rospy.Publisher("soft_airframe_joint_angles", Float32MultiArray, queue_size=1)

    is_simulation = is_simulation()
    print("simulation mode" if is_simulation else "real robot mode")

    dest = None
    last_published_target = None
    rate = rospy.Rate(10)
    
    try:
        while True:
            rospy.sleep(0.5)
            tail_msg = ServoControlCmd()
            tail_msg.index = [8, 7, 9, 10, 12, 11, 13, 14]

            rotor_msg = PwmTest()
            rotor_msg.motor_index = [5]

            soft_joint_msg = JointState()
            soft_joint_msg.name = ["soft_joint2", "soft_joint3", "soft_joint4", "soft_joint5"]
            soft_joint_msg.effort = [0.3, 0.3, 0.3, 0.3]

            if last_published_target is None:
                last_published_target = [0.0, 0.0, 0.0, 0.0]
                last_published_target[0] = math.radians(float(input("last_published alpha_1 (deg): ")))
                last_published_target[1] = math.radians(float(input("last_published alpha_2 (deg): ")))
                last_published_target[2] = math.radians(float(input("last_published alpha_3 (deg): ")))
                last_published_target[3] = math.radians(float(input("last_published alpha_4 (deg): ")))

            if need_next_input(dest, last_published_target):
                dest_alpha_1 = math.radians(float(input("alpha_1 (deg): ")))
                if abs(dest_alpha_1) > math.radians(99):
                    print("stop!!!!")
                    rotor_msg.pwms = [0.5]
                    # rotor_pub.publish(rotor_msg)
                    rospy.sleep(0.5)
                    exit()
                dest_alpha_2 = math.radians(float(input("alpha_2 (deg): ")))
                if abs(dest_alpha_2) > math.radians(99):
                    print("stop!!!!")
                    rotor_msg.pwms = [0.5]
                    # rotor_pub.publish(rotor_msg)
                    rospy.sleep(0.5)
                    exit()
                dest_alpha_3 = math.radians(float(input("alpha_3 (deg): ")))
                if abs(dest_alpha_3) > math.radians(99):
                    print("stop!!!!")
                    rotor_msg.pwms = [0.5]
                    # rotor_pub.publish(rotor_msg)
                    rospy.sleep(0.5)
                    exit()
                dest_alpha_4 = math.radians(float(input("alpha_4 (deg): ")))
                if abs(dest_alpha_4) > math.radians(99):
                    print("stop!!!!")
                    rotor_msg.pwms = [0.5]
                    # rotor_pub.publish(rotor_msg)
                    rospy.sleep(0.5)
                    exit()
                dest = [dest_alpha_1, dest_alpha_2, dest_alpha_3, dest_alpha_4]

            if dest[0] - last_published_target[0] > 0:
                dest_alpha_1 = last_published_target[0] + math.radians(1.0)
            else:
                dest_alpha_1 = last_published_target[0] - math.radians(1.0)
            if dest[1] - last_published_target[1] > 0:
                dest_alpha_2 = last_published_target[1] + math.radians(1.0)
            else:
                dest_alpha_2 = last_published_target[1] - math.radians(1.0)
            if dest[2] - last_published_target[2] > 0:
                dest_alpha_3 = last_published_target[2] + math.radians(1.0)
            else:
                dest_alpha_3 = last_published_target[2] - math.radians(1.0)
            if dest[3] - last_published_target[3] > 0:
                dest_alpha_4 = last_published_target[3] + math.radians(1.0)
            else:
                dest_alpha_4 = last_published_target[3] - math.radians(1.0)
            last_published_target = [dest_alpha_1, dest_alpha_2, dest_alpha_3, dest_alpha_4]

            (
                x_plus_long_wire_soft_link_1,
                x_minus_long_wire_soft_link_1,
                x_plus_short_wire_soft_link_1,
                x_minus_short_wire_soft_link_1,   
            ) = get_wire_diff(dest_alpha_1, dest_alpha_2)

            (
                x_plus_long_wire_soft_link_2,
                x_minus_long_wire_soft_link_2,
                x_plus_short_wire_soft_link_2,
                x_minus_short_wire_soft_link_2,
            ) = get_wire_diff(dest_alpha_3, dest_alpha_4)

            print("dest_alpha_1: ", math.degrees(dest_alpha_1), "deg")
            print("dest_alpha_2: ", math.degrees(dest_alpha_2), "deg")
            print("p: ", fk(dest_alpha_1, dest_alpha_2))
            print("x_plus_long_wire: ", x_plus_long_wire_soft_link_1)
            print("x_minus_long_wire: ", x_minus_long_wire_soft_link_1)
            print("x_plus_short_wire: ", x_plus_short_wire_soft_link_1)
            print("x_minus_short_wire: ", x_minus_short_wire_soft_link_1)
            print("x_plus_long_wire_2: ", x_plus_long_wire_soft_link_2)
            print("x_minus_long_wire_2: ", x_minus_long_wire_soft_link_2)
            print("x_plus_short_wire_2: ", x_plus_short_wire_soft_link_2)
            print("x_minus_short_wire_2: ", x_minus_short_wire_soft_link_2)
            print()

            dest_servo_angles = [
                2047 + 1 * get_angle_diff(x_plus_long_wire_soft_link_1),
                2047 - 1 * get_angle_diff(x_minus_long_wire_soft_link_1),
                2047 + 1 * get_angle_diff(x_plus_short_wire_soft_link_1),
                2047 - 1 * get_angle_diff(x_minus_short_wire_soft_link_1),
                2047 + 1 * get_angle_diff(x_plus_long_wire_soft_link_2),
                2047 - 1 * get_angle_diff(x_minus_long_wire_soft_link_2),
                2047 + 1 * get_angle_diff(x_plus_short_wire_soft_link_2),
                2047 - 1 * get_angle_diff(x_minus_short_wire_soft_link_2),
            ]
            print("dest_servo_angles: ", dest_servo_angles)
            print()

            tail_msg.angles = dest_servo_angles
            tail_pub.publish(tail_msg)

            soft_joint_msg.position = [dest_alpha_1, dest_alpha_2, dest_alpha_3, dest_alpha_4]
            soft_joint_msg.header.stamp = rospy.Time.now()
            if is_simulation:
                servo_controller_pub.publish(soft_joint_msg)
            else:
                soft_joint_pub.publish(soft_joint_msg)


            joint_angle_pub.publish(Float32MultiArray(data=[dest_alpha_1, dest_alpha_2]))
            rospy.sleep(0.01)

    except Exception as e:
        print(repr(e))
        pass
