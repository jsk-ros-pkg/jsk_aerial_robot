#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState
from spinal.msg import DesireCoord

rospy.init_node("ninja_transformation_demo")

joint_control_pub = rospy.Publisher("/assembly/target_joint_pos", JointState, queue_size=10)
att_control_pub = rospy.Publisher("/target_com_rot", DesireCoord, queue_size=1)

demo_mode = rospy.get_param("~mode", 0)
reset = rospy.get_param("~reset", False)
print(reset)

desire_joint = JointState()
desire_att = DesireCoord()

desire_joint.name = ['mod1/yaw','mod2/pitch','mod2/yaw','mod3/pitch','mod3/yaw','mod4/pitch']

if reset:
    rospy.loginfo('reset')
    desire_joint.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    desire_att.pitch = 0
    desire_att.roll = 0
elif demo_mode == 0: # V shape
    rospy.loginfo('mode 0')
    desire_joint.position = [-1.5,-0.7,0.0,1.4,-1.5,0.0]
    desire_att.pitch = 0.7
elif demo_mode == 1: # spiral
    rospy.loginfo('mode 1')
    desire_joint.position = [-1.0,-0.6,-1.5,0.8,-1.5,1.0]
    desire_att.pitch = -0.2
    desire_att.yaw = math.pi/2.0
elif demo_mode == 2: # circle
    rospy.loginfo('mode 1')
    desire_joint.position = [-1.57,-0.0,-1.57,0.0,-1.57,0.0]]
# elif demo_mode == 2: # mode model
#     rospy.loginfo('mode 2')
#     desire_joint.position = [0.0, half_pi, -1.5, 0.0, 0.0, half_pi]
#     desire_att.pitch = 0.75
else:
    sys.exit()

time.sleep(0.6)

joint_control_pub.publish(desire_joint)
att_control_pub.publish(desire_att)

