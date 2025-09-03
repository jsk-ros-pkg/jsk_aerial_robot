#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from scipy.interpolate import CubicSpline
from aerial_robot_msgs.msg import FlightNav
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_euler
from spinal.msg import DesireCoord
from sensor_msgs.msg import JointState
import math
import time

rospy.init_node("ninja_valve_opening")

assembly_nav_pub = rospy.Publisher("/assembly/uav/nav", FlightNav, queue_size=10)
att_control_pub = rospy.Publisher("/target_com_rot", DesireCoord, queue_size=1)
joint_control_pub = rospy.Publisher("/assembly/target_joint_pos", JointState, queue_size=10)

nav_msg_leader = FlightNav()
nav_msg_leader.target = 0
nav_msg_leader.control_frame = 0
nav_msg_leader.pos_xy_nav_mode=2
nav_msg_leader.target_pos_x = 0.3950676918029785
nav_msg_leader.target_pos_y = -0.003435059217736125


desire_att = DesireCoord()
desire_att.pitch = 0
desire_att.roll = 0
desire_att.yaw = 0

desire_joint = JointState()
desire_joint.name = ['mod1/pitch','mod1/yaw','mod2/pitch']
desire_joint.position = [-math.pi/2, 0, 0]

time.sleep(0.6)
assembly_nav_pub.publish(nav_msg_leader)
att_control_pub.publish(desire_att)
joint_control_pub.publish(desire_joint)
