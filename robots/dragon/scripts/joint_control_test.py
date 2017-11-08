#!/usr/bin/env python

import sys
import time
import rospy
import math
from sensor_msgs.msg import JointState

def joint_state_cb(data):
    global start_flag
    global joint_state
    global ctrl_joint

    joint_state = []
    for i in range(0, len(data.position)):
        if "joint" in data.name[i] :
            joint_state.append(data.position[i])

    if start_flag == True:
        return

    for i in range(0, len(data.position)):
        if "joint" in data.name[i] :
            ctrl_joint.position.append(data.position[i])

    start_flag = True

if __name__=="__main__":
    rospy.init_node("joint_control_test")

    start_flag = False
    joint_state = JointState()
    joint_size = 0

    duration = rospy.get_param("~duration", 0.05) #20Hz

    joint_control_topic_name = rospy.get_param("~joint_control_topic_name", "/dragon/joints_ctrl")
    pub = rospy.Publisher(joint_control_topic_name, JointState, queue_size=10)
    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/dragon/joint_states")
    rospy.Subscriber(joint_state_topic_name, JointState, joint_state_cb)

    start_time = 0

    target_max_pitch = -0.34
    ctrl_joint = JointState()
    ctrl_joint.position = []

    init_step = True

    while not rospy.is_shutdown():

        if start_flag == False:
            continue

        if init_step == True:

            if len(joint_state) == 0:
                continue

            if abs(joint_state[1]) > 0.07 or abs(joint_state[0] - target_max_pitch) > 0.07:
                if abs(joint_state[1]) > 0.05:
                    rospy.logwarn("the joint 1 yaw is too big: %f", joint_state[1])

                if abs(joint_state[0] - target_max_pitch) > 0.05:
                    rospy.logwarn("the joint 1 pitch is too small %f", joint_state[0])

                ctrl_joint.position[0] = target_max_pitch
                ctrl_joint.position[1] = 0

                pub.publish(ctrl_joint)
                time.sleep(duration)
            else:
                init_step = False
                start_time = rospy.get_time()
            continue

        time_diff = start_time - rospy.get_time()
        ctrl_joint.position[0] = -0.34 * abs(math.cos(time_diff / math.pi ))
        ctrl_joint.position[1] = 0.78 * math.sin(2 * time_diff / math.pi / 2 )

        pub.publish(ctrl_joint)
        time.sleep(duration)
