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
import pulp

from get_g import get_g

s = 110
d = 5
r_joint_1 = 85 / 2 / math.sqrt(2)
r_joint_2 = 85 / 2
r_wheel = 20

def get_tau_des(alpha):
    return 2.55 * alpha * (s/2) # N*mm

def get_wire_tension(tau_des):
    # tau_des = D * F
    # F: tention of wire
    D = np.array([[-r_joint_2, r_joint_2, 0, r_joint_2, -r_joint_2],
                  [-r_joint_2, -r_joint_2, r_joint_1, r_joint_2, r_joint_2],
                  [-r_joint_2, r_joint_2, 0, r_joint_2, -r_joint_2],
                  [-r_joint_2, -r_joint_2, 0, r_joint_2, r_joint_2]])
    
    m, n = D.shape  # m=4 equations, n=5 variables

    prob = pulp.LpProblem("Minimize_Total_Tension", pulp.LpMinimize)
    F = [pulp.LpVariable(f"F_{i}", lowBound=0) for i in range(n)]
    prob += pulp.lpSum(F)

    for row in range(m):
        prob += pulp.lpSum(D[row, col] * F[col] for col in range(n)) == tau_des[row], f"Eq_{row}"

    prob.solve()

    # print("Status:", pulp.LpStatus[prob.status])
    F_sol = np.array([pulp.value(var) for var in F])
    # print("Optimal F:", F_sol)
    # print("Sum of tensions (objective value):", np.sum(F_sol))
    # print("D @ F =", D @ F_sol)
    # print("tau_des =", tau_des)
    # print("誤差 =", np.linalg.norm(D @ F_sol - tau_des))
    return F_sol

def tension_to_current(tension):
    servo_torque = tension * r_wheel / 2
    # servo_torque = torque_constant * current
    torque_constant = 0.215 * 1000 / (1.47 * 1000) # N*mm/mA
    plus_and_minus = [1, -1, 1, -1, 1]
    return servo_torque / torque_constant * plus_and_minus


if __name__ == "__main__":
    rospy.init_node("tail_torque_based_control")
    tail_pub = rospy.Publisher("/servo/target_current", ServoControlCmd, queue_size=1)

    rate = rospy.Rate(10)

    try:
        while True:
            rospy.sleep(0.5)
            tail_msg = ServoControlCmd()
            tail_msg.index = [6, 7, 3, 4, 5]
    
            # alpha_1 = math.radians(20)
            # alpha_2 = math.radians(0)   
            # alpha_3 = alpha_1   
            # alpha_4 = math.radians(0)   

            alpha_1 = math.radians(float(input("alpha_1 (deg): ")))
            alpha_2 = math.radians(float(input("alpha_2 (deg): ")))
            alpha_3 = alpha_1
            alpha_4 = math.radians(float(input("alpha_4 (deg): ")))
            
            
            # tau_des = np.array([get_tau_des(alpha_1), get_tau_des(alpha_2), get_tau_des(alpha_4)])
            tau_des = np.array([get_tau_des(alpha_1), get_tau_des(alpha_2), get_tau_des(alpha_3), get_tau_des(alpha_4)])
            print("tau_des (N*mm): ", tau_des)
            tau_des -= get_g([alpha_1, alpha_2, alpha_3, alpha_4]) * 1000 # N*mm
            print("tau_des (N*mm): ", tau_des)

            wire_tension = get_wire_tension(tau_des)
            print("wire_tension (N): ", wire_tension)

            dest_servo_current = tension_to_current(wire_tension)
            print("dest_servo_current (mA): ", dest_servo_current)

            tail_msg.angles = dest_servo_current.astype(np.int32).tolist()
            tail_pub.publish(tail_msg)

            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))
        pass
