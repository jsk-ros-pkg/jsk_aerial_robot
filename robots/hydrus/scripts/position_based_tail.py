#!/usr/bin/env python

from __future__ import print_function # for print function in python2
import sys, select, termios, tty

import rospy
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav
import rosgraph
from spinal.msg import ServoControlCmd
import numpy as np
import math
from aerial_robot_base.robot_interface import RobotInterface

s = 230
d = 5
r_joint_1 = 85 / 2 / math.sqrt(2)
r_joint_2 = 85 / 2
r_wheel = 20


def solve_ik(alpha_1, alpha_3, p_des):
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
                [math.cos(alpha_3), math.sin(alpha_3)],
                [-math.sin(alpha_3), math.cos(alpha_3)],
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
                [-math.sin(alpha_3), math.cos(alpha_3)],
                [-math.cos(alpha_3), -math.sin(alpha_3)],
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
        if alpha_3 == 0:
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
            r_3 = s / alpha_3
            p_2_to_3 = np.array(
                [
                    [r_3 * (1 - math.cos(alpha_3)) + d * math.sin(alpha_3)],
                    [r_3 * math.sin(alpha_3) + d * math.cos(alpha_3)],
                ]
            )
            p_2_to_3_prime = np.array(
                [
                    [
                        -s / alpha_3 / alpha_3 * (1 - math.cos(alpha_3))
                        + r_3 * math.sin(alpha_3)
                        + d * math.cos(alpha_3)
                    ],
                    [
                        -s / alpha_3 / alpha_3 * math.sin(alpha_3)
                        + r_3 * math.cos(alpha_3)
                        - d * math.sin(alpha_3)
                    ],
                ]
            )

        p = (
            p_0_to_1
            + R_0_to_1 @ p_2_to_3
        )

        print("alpha_1: ", math.degrees(alpha_1), "deg")
        print("alpha_3: ", math.degrees(alpha_3), "deg")
        if np.linalg.norm(p_des - p[:1, :]) < 13:
            print("p_des: ")
            print(p_des)
            print("p: ")
            print(p)
            return (alpha_1, alpha_3)

        dp_dalpha_1 = (
            p_0_to_1_prime
            + R_0_to_1_prime @ p_2_to_3
            + R_0_to_1 @ p_2_to_3_prime
        )
        dp_dalpha_3 = (
            R_0_to_1 @ p_2_to_3_prime
        )

        dp_dalpha = np.concatenate((dp_dalpha_1, dp_dalpha_3), axis=1)
        dp_dalpha = dp_dalpha[:1, :] 
        dq = np.dot(np.linalg.pinv(dp_dalpha), (p_des - p[:1, :]) * 0.05)
        # dq = np.dot(np.linalg.pinv(dp_dalpha), (p_des - p)*0.05)

        alpha_1 += dq[0, 0]
        alpha_3 += dq[1, 0]

        if i == 499:
            raise Exception("IK was not solved")


def get_wire_diff(alpha_1, alpha_3):
    divide_num = 4
    def get_plus_pos_wire_length(alpha, r_joint):  # xまたはyが正のワイヤーの長さ
        if alpha == 0:
            return s + d
        r = s / abs(alpha)
        if alpha > 0:
            return divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num) + d
        else:
            return divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num) + d

    def get_minus_pos_wire_length(alpha, r_joint):  # xまたはyが負のワイヤーの長さ
        if alpha == 0:
            return s + d
        r = s / abs(alpha)
        if alpha > 0:
            return divide_num * (r + r_joint + 1.5) * math.sin(abs(alpha) / divide_num) + d
        else:
            return divide_num * (r - r_joint - 1.5) * math.sin(abs(alpha) / divide_num) + d

    x_plus_long_wire = get_plus_pos_wire_length(alpha_1, r_joint_2) + d + get_plus_pos_wire_length(alpha_3, r_joint_2)
    x_minus_long_wire = get_minus_pos_wire_length(alpha_1, r_joint_2) + d +get_minus_pos_wire_length(alpha_3, r_joint_2)
    x_plus_short_wire = get_plus_pos_wire_length(alpha_1, r_joint_2) + d

    return (
        x_plus_long_wire - (s + d) * 2 - d,
        x_minus_long_wire - (s + d) * 2- d,
        x_plus_short_wire - (s + d) - d,
    )


def get_angle_diff(wire_diff):
    return int(wire_diff / r_wheel / math.pi * 4096)


if __name__ == "__main__":
    alpha_1 = math.radians(1)
    alpha_3 = math.radians(1)

    rospy.init_node("tail_ik")
    tail_pub = rospy.Publisher("servo/target_states", ServoControlCmd, queue_size=1)

    rate = rospy.Rate(10)

    # ri = RobotInterface()
    # rospy.sleep(1.0)
    # print(ri.getCogPos())
    # ri.trajectoryNavigate(pos=[0,0,0.6], rot=None)
    # rospy.sleep(5)
    
    try:
        while True:
            rospy.sleep(0.5)
            tail_msg = ServoControlCmd()
            tail_msg.index = [7, 6, 4]

            # x = float(input("x (default 0)   "))
            # z = float(input("z"))

            # p_des = np.array(
            #     [
            #         [x],
            #         # [z],
            #     ]
            # )

            # dest_alpha_1, dest_alpha_3 = solve_ik(
            #     alpha_1, alpha_3, p_des
            # )
            dest_alpha_1 = math.radians(float(input("alpha_1 (deg): ")))
            dest_alpha_3 = math.radians(float(input("alpha_3 (deg): ")))

            (
                x_plus_long_wire,
                x_minus_long_wire,
                x_plus_short_wire,      
            ) = get_wire_diff(dest_alpha_1, dest_alpha_3)

            rotor_x = 16.9 * alpha_3 # N
            rotor_z = # (234g分) N
            rotor_angle = math.atan2(rotor_x, rotor_z)
            rotor_force = math.sqrt(rotor_x**2 + rotor_z**2)
            # todo: publish these values

            print("dest_alpha_1: ", math.degrees(dest_alpha_1), "deg")
            print("dest_alpha_3: ", math.degrees(dest_alpha_3), "deg")
            print("x_plus_long_wire: ", x_plus_long_wire)
            print("x_minus_long_wire: ", x_minus_long_wire)
            print("x_plus_short_wire: ", x_plus_short_wire)
            print("rotor_angle: ", math.degrees(rotor_angle), "deg")
            print("rotor_force: ", rotor_force, "N")
            print()
            init_servo_angles = [2047, 2047, 2047]
            dest_servo_angles = [
                2047 + 1 * get_angle_diff(x_plus_long_wire),
                2047 - 1 * get_angle_diff(x_minus_long_wire),
                2047 + 1 * get_angle_diff(x_plus_short_wire),
                2047 + int(rotor_angle / (2 * math.pi) * 4096),
            ]
            print("dest_servo_angles: ", dest_servo_angles)
            print()

            tail_msg.angles = dest_servo_angles
            tail_pub.publish(tail_msg)

            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))
        pass
