
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
import pulp


def get_g(alpha):
    s = 110 / 1000 # m
    d = 5 / 1000 # m
    print(alpha)
    alpha_1, alpha_2, alpha_3, alpha_4 = alpha
    weight_of_middle = 26 / 1000  # kg
    weight_of_joint = 15 / 1000

    R_0_to_1 = np.array(
        [
            [math.cos(alpha_1), 0, math.sin(alpha_1)],
            [0, 1, 0],
            [-math.sin(alpha_1), 0, math.cos(alpha_1)],
        ]
    )
    R_1_to_2 = np.array(
        [
            [1, 0, 0],
            [0, math.cos(alpha_2), math.sin(alpha_2)],
            [0, -math.sin(alpha_2), math.cos(alpha_2)],
        ]
    )
    R_2_to_3 = np.array(
        [
            [math.cos(alpha_3), 0, math.sin(alpha_3)],
            [0, 1, 0],
            [-math.sin(alpha_3), 0, math.cos(alpha_3)],
        ]
    )
    R_3_to_4 = np.array(
        [
            [1, 0, 0],
            [0, math.cos(alpha_4), math.sin(alpha_4)],
            [0, -math.sin(alpha_4), math.cos(alpha_4)],
        ]
    )

    R_0_to_1_prime = np.array(
        [
            [-math.sin(alpha_1), 0, math.cos(alpha_1)],
            [0, 0, 0],
            [-math.cos(alpha_1), 0, -math.sin(alpha_1)],
        ]
    )
    R_1_to_2_prime = np.array(
        [
            [0, 0, 0],
            [0, -math.sin(alpha_2), math.cos(alpha_2)],
            [0, -math.cos(alpha_2), -math.sin(alpha_2)],
        ]
    )
    R_2_to_3_prime = np.array(
        [
            [-math.sin(alpha_3), 0, math.cos(alpha_3)],
            [0, 0, 0],
            [-math.cos(alpha_3), 0, -math.sin(alpha_3)],
        ]
    )
    R_3_to_4_prime = np.array(
        [
            [0, 0, 0],
            [0, -math.sin(alpha_4), math.cos(alpha_4)],
            [0, -math.cos(alpha_4), -math.sin(alpha_4)],
        ]
    )

    if alpha_1 == 0:
        p_0_to_1 = np.array(
            [
                [0],
                [0],
                [s + d],
            ]
        )
        p_0_to_1_prime = np.array(
            [
                [s / 2 + d],
                [0],
                [0],
            ]
        )
        p_0_to_1_middle = np.array(
            [
                [0],
                [0],
                [s/2],
            ]
        )
        p_0_to_1_middle_prime = np.array(
            [
                [s / 4],
                [0],
                [0],
            ]
        )
    else:
        r_1 = s / alpha_1
        p_0_to_1 = np.array(
            [
                [r_1 * (1 - math.cos(alpha_1)) + d * math.sin(alpha_1)],
                [0],
                [r_1 * math.sin(alpha_1) + d * math.cos(alpha_1)],
            ]
        )
        p_0_to_1_prime = np.array(
            [
                [
                    -s / alpha_1 / alpha_1 * (1 - math.cos(alpha_1))
                    + r_1 * math.sin(alpha_1)
                    + d * math.cos(alpha_1)
                ],
                [0],
                [
                    -s / alpha_1 / alpha_1 * math.sin(alpha_1)
                    + r_1 * math.cos(alpha_1)
                    - d * math.sin(alpha_1)
                ],
            ]
        )
        p_0_to_1_middle = np.array(
            [
                [r_1 * (1 - math.cos(alpha_1 / 2))],
                [0],
                [r_1 * math.sin(alpha_1 / 2)],
            ]
        )
        p_0_to_1_middle_prime = np.array(
            [
                [
                    -s / alpha_1 / alpha_1 * (1 - math.cos(alpha_1 / 2))
                    + r_1 * math.sin(alpha_1 / 2) / 2
                ],
                [0],
                [
                    -s / alpha_1 / alpha_1 * math.sin(alpha_1 / 2)
                    + r_1 * math.cos(alpha_1 / 2) / 2
                ],
            ]
        )
    if alpha_2 == 0:
        p_1_to_2 = np.array(
            [
                [0],
                [0],
                [s + d],
            ]
        )
        p_1_to_2_prime = np.array(
            [
                [0],
                [s / 2 + d],
                [0],
            ]
        )
        p_1_to_2_middle = np.array(
            [
                [0],
                [0],
                [s/2],
            ]
        )
        p_1_to_2_middle_prime = np.array(
            [
                [0],
                [s / 4],
                [0],
            ]
        )
    else:
        r_2 = s / alpha_2
        p_1_to_2 = np.array(
            [
                [0],
                [r_2 * (1 - math.cos(alpha_2)) + d * math.sin(alpha_2)],
                [r_2 * math.sin(alpha_2) + d * math.cos(alpha_2)],
            ]
        )
        p_1_to_2_prime = np.array(
            [
                [0],
                [
                    -s / alpha_2 / alpha_2 * (1 - math.cos(alpha_2))
                    + r_2 * math.sin(alpha_2)
                    + d * math.cos(alpha_2)
                ],
                [
                    -s / alpha_2 / alpha_2 * math.sin(alpha_2)
                    + r_2 * math.cos(alpha_2)
                    - d * math.sin(alpha_2)
                ],
            ]
        )
        p_1_to_2_middle = np.array(
            [
                [0],
                [r_2 * (1 - math.cos(alpha_2 / 2))],
                [r_2 * math.sin(alpha_2 / 2)],
            ]
        )
        p_1_to_2_middle_prime = np.array(
            [
                [0],
                [
                    -s / alpha_2 / alpha_2 * (1 - math.cos(alpha_2 / 2))
                    + r_2 * math.sin(alpha_2 / 2) / 2
                ],
                [
                    -s / alpha_2 / alpha_2 * math.sin(alpha_2 / 2)
                    + r_2 * math.cos(alpha_2 / 2) / 2
                ],
            ]
        )
    if alpha_3 == 0:
        p_2_to_3 = np.array(
            [
                [0],
                [0],
                [s + d],
            ]
        )
        p_2_to_3_prime = np.array(
            [
                [s / 2 + d],
                [0],
                [0],
            ]
        )
        p_2_to_3_middle = np.array(
            [
                [0],
                [0],
                [s/2],
            ]
        )
        p_2_to_3_middle_prime = np.array(
            [
                [s / 4],
                [0],
                [0],
            ]
        )
    else:
        r_3 = s / alpha_3
        p_2_to_3 = np.array(
            [
                [r_3 * (1 - math.cos(alpha_3)) + d * math.sin(alpha_3)],
                [0],
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
                [0],
                [
                    -s / alpha_3 / alpha_3 * math.sin(alpha_3)
                    + r_3 * math.cos(alpha_3)
                    - d * math.sin(alpha_3)
                ],
            ]
        )
        p_2_to_3_middle = np.array(
            [
                [r_3 * (1 - math.cos(alpha_3 / 2))],
                [0],
                [r_3 * math.sin(alpha_3 / 2)],
            ]
        )
        p_2_to_3_middle_prime = np.array(
            [
                [
                    -s / alpha_3 / alpha_3 * (1 - math.cos(alpha_3 / 2))
                    + r_3 * math.sin(alpha_3 / 2) / 2
                ],
                [0],
                [
                    -s / alpha_3 / alpha_3 * math.sin(alpha_3 / 2)
                    + r_3 * math.cos(alpha_3 / 2) / 2
                ],
            ]
        )
    if alpha_4 == 0:
        p_3_to_4 = np.array(
            [
                [0],
                [0],
                [s + d],
            ]
        )
        p_3_to_4_prime = np.array(
            [
                [0],
                [s / 2 + d],
                [0],
            ]
        )
        p_3_to_4_middle = np.array(
            [
                [0],
                [0],
                [s/2],
            ]
        )
        p_3_to_4_middle_prime = np.array(
            [
                [0],
                [s / 4],
                [0],
            ]
        )
    else:
        r_4 = s / alpha_4
        p_3_to_4 = np.array(
            [
                [0],
                [r_4 * (1 - math.cos(alpha_4)) + d * math.sin(alpha_4)],
                [r_4 * math.sin(alpha_4) + d * math.cos(alpha_4)],
            ]
        )
        p_3_to_4_prime = np.array(
            [
                [0],
                [
                    -s / alpha_4 / alpha_4 * (1 - math.cos(alpha_4))
                    + r_4 * math.sin(alpha_4)
                    + d * math.cos(alpha_4)
                ],
                [
                    -s / alpha_4 / alpha_4 * math.sin(alpha_4)
                    + r_4 * math.cos(alpha_4)
                    - d * math.sin(alpha_4)
                ],
            ]
        )
        p_3_to_4_middle = np.array(
            [
                [0],
                [r_4 * (1 - math.cos(alpha_4 / 2))],
                [r_4 * math.sin(alpha_4 / 2)],
            ]
        )
        p_3_to_4_middle_prime = np.array(
            [
                [0],
                [
                    -s / alpha_4 / alpha_4 * (1 - math.cos(alpha_4 / 2))
                    + r_4 * math.sin(alpha_4 / 2) / 2
                ],
                [
                    -s / alpha_4 / alpha_4 * math.sin(alpha_4 / 2)
                    + r_4 * math.cos(alpha_4 / 2) / 2
                ],
            ]
        )

    # p_0_to_2 = p_0_to_1 + R_0_to_1 @ p_1_to_2
    # p_0_to_2_middle = p_0_to_1 + R_0_to_1 @ p_1_to_2_middle
    # p_0_to_3 = p_0_to_2 + R_0_to_1 @ R_1_to_2 @ p_2_to_3
    # p_0_to_3_middle = p_0_to_2 + R_0_to_1 @ R_1_to_2 @ p_2_to_3_middle
    # p_0_to_4 = p_0_to_3 + R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4
    # p_0_to_4_middle = p_0_to_3 + R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4_middle

    g = np.array(
        [
            [0],
            [-9.81],
            [0],
        ]
    )
    g = g.T

    u = weight_of_middle * g @ p_0_to_1_middle + weight_of_joint * g @ p_0_to_1
    u += weight_of_middle * g @ (p_0_to_1 + R_0_to_1 @ p_1_to_2_middle)
    u += weight_of_joint * g @ (p_0_to_1 + R_0_to_1 @ p_1_to_2)
    u += weight_of_middle * g @ (p_0_to_1 + R_0_to_1 @ p_1_to_2 + R_0_to_1 @ R_1_to_2 @ p_2_to_3_middle)
    u += weight_of_joint * g @ (p_0_to_1 + R_0_to_1 @ p_1_to_2 + R_0_to_1 @ R_1_to_2 @ p_2_to_3)
    u += weight_of_middle * g @ (p_0_to_1 + R_0_to_1 @ p_1_to_2 + R_0_to_1 @ R_1_to_2 @ p_2_to_3 + R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4_middle)
    u += weight_of_joint * g @ (p_0_to_1 + R_0_to_1 @ p_1_to_2 + R_0_to_1 @ R_1_to_2 @ p_2_to_3 + R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4)
    print("u:", u)

    du_dalpha_1 = (
        weight_of_middle * g @ p_0_to_1_middle_prime
        + weight_of_joint * g @ p_0_to_1_prime
        + weight_of_middle * g @ (p_0_to_1_prime + R_0_to_1_prime @ p_1_to_2_middle)
        + weight_of_joint * g @ (p_0_to_1_prime + R_0_to_1_prime @ p_1_to_2)
        + weight_of_middle * g @ (p_0_to_1_prime + R_0_to_1_prime @ p_1_to_2 + R_0_to_1_prime @ R_1_to_2 @ p_2_to_3_middle)
        + weight_of_joint * g @ (p_0_to_1_prime + R_0_to_1_prime @ p_1_to_2 + R_0_to_1_prime @ R_1_to_2 @ p_2_to_3)
        + weight_of_middle * g @ (p_0_to_1_prime + R_0_to_1_prime @ p_1_to_2 + R_0_to_1_prime @ R_1_to_2 @ p_2_to_3 + R_0_to_1_prime @ R_1_to_2 @ R_2_to_3 @ p_3_to_4_middle)
        + weight_of_joint * g @ (p_0_to_1_prime + R_0_to_1_prime @ p_1_to_2 + R_0_to_1_prime @ R_1_to_2 @ p_2_to_3 + R_0_to_1_prime @ R_1_to_2 @ R_2_to_3 @ p_3_to_4)
    )
    du_dalpha_2 = (
        weight_of_middle * g @ R_0_to_1 @ p_1_to_2_middle_prime
        + weight_of_joint * g @ R_0_to_1 @ p_1_to_2_prime
        + weight_of_middle * g @ (R_0_to_1 @ p_1_to_2_middle_prime + R_0_to_1 @ R_1_to_2_prime @ p_2_to_3_middle)
        + weight_of_joint * g @ (R_0_to_1 @ p_1_to_2_prime + R_0_to_1 @ R_1_to_2_prime @ p_2_to_3)
        + weight_of_middle * g @ (R_0_to_1 @ p_1_to_2_middle_prime + R_0_to_1 @ R_1_to_2_prime @ p_2_to_3 + R_0_to_1 @ R_1_to_2_prime @ R_2_to_3 @ p_3_to_4_middle)
        + weight_of_joint * g @ (R_0_to_1 @ p_1_to_2_prime + R_0_to_1 @ R_1_to_2_prime @ p_2_to_3 + R_0_to_1 @ R_1_to_2_prime @ R_2_to_3 @ p_3_to_4)
    )
    du_dalpha_3 = (
        weight_of_middle * g @ R_0_to_1 @ R_1_to_2 @ p_2_to_3_middle_prime
        + weight_of_joint * g @ R_0_to_1 @ R_1_to_2 @ p_2_to_3_prime
        + weight_of_middle * g @ (R_0_to_1 @ R_1_to_2 @ p_2_to_3_middle_prime + R_0_to_1 @ R_1_to_2 @ R_2_to_3_prime @ p_3_to_4_middle)
        + weight_of_joint * g @ (R_0_to_1 @ R_1_to_2 @ p_2_to_3_prime + R_0_to_1 @ R_1_to_2 @ R_2_to_3_prime @ p_3_to_4)
    )
    du_dalpha_4 = (
        weight_of_middle * g @ R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4_middle_prime
        + weight_of_joint * g @ R_0_to_1 @ R_1_to_2 @ R_2_to_3 @ p_3_to_4_prime
    )

    du_dalpha = np.array([du_dalpha_1.item(), du_dalpha_2.item(), du_dalpha_3.item(), du_dalpha_4.item()])
    print("du_dalpha:", du_dalpha)
    return du_dalpha
