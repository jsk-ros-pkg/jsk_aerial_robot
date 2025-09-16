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


# def solve_ik(alpha_1, alpha_2, alpha_3, alpha_4, p_des):
def solve_ik(alpha_1, alpha_3, p_des):
    for i in range(500):
        print("num of iteration: ", i)
        R_0_to_1 = np.array(
            [
                [math.cos(alpha_1), math.sin(alpha_1)],
                [-math.sin(alpha_1), math.cos(alpha_1)],
            ]
        )
        # R_1_to_2 = np.array(
        #     [
        #         [1, 0, 0],
        #         [0, math.cos(alpha_2), math.sin(alpha_2)],
        #         [0, -math.sin(alpha_2), math.cos(alpha_2)],
        #     ]
        # )
        R_2_to_3 = np.array(
            [
                [math.cos(alpha_3), math.sin(alpha_3)],
                [-math.sin(alpha_3), math.cos(alpha_3)],
            ]
        )
        # R_3_to_4 = np.array(
        #     [
        #         [1, 0, 0],
        #         [0, math.cos(alpha_4), math.sin(alpha_4)],
        #         [0, -math.sin(alpha_4), math.cos(alpha_4)],
        #     ]
        # )

        R_0_to_1_prime = np.array(
            [
                [-math.sin(alpha_1), math.cos(alpha_1)],
                [-math.cos(alpha_1), -math.sin(alpha_1)],
            ]
        )
        # R_1_to_2_prime = np.array(
        #     [
        #         [0, 0, 0],
        #         [0, -math.sin(alpha_2), math.cos(alpha_2)],
        #         [0, -math.cos(alpha_2), -math.sin(alpha_2)],
        #     ]
        # )
        R_2_to_3_prime = np.array(
            [
                [-math.sin(alpha_3), math.cos(alpha_3)],
                [-math.cos(alpha_3), -math.sin(alpha_3)],
            ]
        )
        # R_3_to_4_prime = np.array(
        #     [
        #         [0, 0, 0],
        #         [0, -math.sin(alpha_4), math.cos(alpha_4)],
        #         [0, -math.cos(alpha_4), -math.sin(alpha_4)],
        #     ]
        # )

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
        # if alpha_2 == 0:
        #     p_1_to_2 = np.array(
        #         [
        #             [0],
        #             [0],
        #             [s + d],
        #         ]
        #     )
        #     p_1_to_2_prime = np.array(
        #         [
        #             [0],
        #             [s / 2 + d],
        #             [0],
        #         ]
        #     )
        # else:
        #     r_2 = s / alpha_2
        #     p_1_to_2 = np.array(
        #         [
        #             [0],
        #             [r_2 * (1 - math.cos(alpha_2)) + d * math.sin(alpha_2)],
        #             [r_2 * math.sin(alpha_2) + d * math.cos(alpha_2)],
        #         ]
        #     )
        #     p_1_to_2_prime = np.array(
        #         [
        #             [0],
        #             [
        #                 -s / alpha_2 / alpha_2 * (1 - math.cos(alpha_2))
        #                 + r_2 * math.sin(alpha_2)
        #                 + d * math.cos(alpha_2)
        #             ],
        #             [
        #                 -s / alpha_2 / alpha_2 * math.sin(alpha_2)
        #                 + r_2 * math.cos(alpha_2)
        #                 - d * math.sin(alpha_2)
        #             ],
        #         ]
        #     )
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
        # if alpha_4 == 0:
        #     p_3_to_4 = np.array(
        #         [
        #             [0],
        #             [0],
        #             [s + d],
        #         ]
        #     )
        #     p_3_to_4_prime = np.array(
        #         [
        #             [0],
        #             [s / 2 + d],
        #             [0],
        #         ]
        #     )
        # else:
        #     r_4 = s / alpha_4
        #     p_3_to_4 = np.array(
        #         [
        #             [0],
        #             [r_4 * (1 - math.cos(alpha_4)) + d * math.sin(alpha_4)],
        #             [r_4 * math.sin(alpha_4) + d * math.cos(alpha_4)],
        #         ]
        #     )
        #     p_3_to_4_prime = np.array(
        #         [
        #             [0],
        #             [
        #                 -s / alpha_4 / alpha_4 * (1 - math.cos(alpha_4))
        #                 + r_4 * math.sin(alpha_4)
        #                 + d * math.cos(alpha_4)
        #             ],
        #             [
        #                 -s / alpha_4 / alpha_4 * math.sin(alpha_4)
        #                 + r_4 * math.cos(alpha_4)
        #                 - d * math.sin(alpha_4)
        #             ],
        #         ]
        #     )

        p = (
            p_0_to_1
            + R_0_to_1 @ p_2_to_3
        )
        # print("p: ")
        # print(p)

        # if np.linalg.norm(p_des - p[:2, :]) < 13:
        print("alpha_1: ", math.degrees(alpha_1), "deg")
        # print("alpha_2: ", math.degrees(alpha_2), "deg")
        print("alpha_3: ", math.degrees(alpha_3), "deg")
        # print("alpha_4: ", math.degrees(alpha_4), "deg")
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
    
    # dest_poses = [
    #     [0.0, 0.0],
    #     [200.0, 0.0],
    #     [-200.0, 0.0],
    #     [0.0, 0.0],
    #     [0.0, 200.0],
    #     [0.0, -200.0],
    # ]
    # pos_index = 0
    dest_pos_x = 350
    dest_pos_y = 0
    max_x = 300
    min_x = -300


    try:
        while True:
            rospy.sleep(0.5)
            tail_msg = ServoControlCmd()
            # tail_msg.index = [0, 1, 2, 3, 4]
            tail_msg.index = [7, 6, 4]

            # dest_pos_x -= 50
            # if dest_pos_x < min_x:
            #     dest_pos_x = max_x
            #     dest_pos_y -= 50
            # if dest_pos_x < min_x:
            #     exit()
            x = float(input("x (default 0)   "))
            # z = float(input("z (default 475) "))
            # z = float(input("z"))
            # x, y = dest_poses[pos_index]
            # pos_index += 1
            # if pos_index >= len(dest_poses):
            #     pos_index = 0
            # print("x: ", x, " y: ", y)

            p_des = np.array(
                [
                    # [dest_pos_x],
                    # [dest_pos_y],
                    # [dest_pos_z],
                    [x],
                    # [z],
                    # [z],
                ]
            )

            dest_alpha_1, dest_alpha_3 = solve_ik(
                alpha_1, alpha_3, p_des
            )

            (
                x_plus_long_wire,
                x_minus_long_wire,
                x_plus_short_wire,      
            ) = get_wire_diff(dest_alpha_1, dest_alpha_3)

            print("dest_alpha_1: ", math.degrees(dest_alpha_1), "deg")
            print("dest_alpha_3: ", math.degrees(dest_alpha_3), "deg")
            print("x_plus_long_wire: ", x_plus_long_wire)
            print("x_minus_long_wire: ", x_minus_long_wire)
            print("x_plus_short_wire: ", x_plus_short_wire)
            print()
            init_servo_angles = [2047, 2047, 2047]
            dest_servo_angles = [
                2047 + 1 * get_angle_diff(x_plus_long_wire),
                2047 - 1 * get_angle_diff(x_minus_long_wire),
                2047 + 1 * get_angle_diff(x_plus_short_wire),
            ]
            print("dest_servo_angles: ", dest_servo_angles)
            print()

            tail_msg.angles = dest_servo_angles
            tail_pub.publish(tail_msg)

            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))
        pass
