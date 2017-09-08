#!/usr/bin/env python

import sys
import time
import rospy
import tf
import math
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf import transformations as t

def gimbal_state_cb(data):
    global gimbal_start_flag
    global gimbal_state

    if not gimbal_start_flag:
        for i in range(0, len(data.position)):
            if i % 2 == 0:
                gimbal_state.name.append("gimbal" + str(i / 2 + 1) + "_roll") # e.g. gimbal1_roll
            else:
                gimbal_state.name.append("gimbal" + str(i / 2 + 1) + "_pitch") # e.g. gimbal1_pitch

            gimbal_state.position.append(data.position[i])

        gimbal_start_flag = True
    else:
        for i in range(0, len(data.position)):
            gimbal_state.position[i] = data.position[i]

def cog2baselink_cb(data):

    t = tf.Transformer(True, rospy.Duration(10.0))
    t.setTransform(data)
    (inv_trans, inv_rot)  = t.lookupTransform('fc', 'cog', rospy.Time(0))

    br = tf.TransformBroadcaster()
    br.sendTransform(inv_trans,
                     inv_rot,
                     data.header.stamp,
                     data.header.frame_id,
                     data.child_frame_id)

def joint_state_cb(data):
    global joint_start_flag
    global joint_state

    if not joint_start_flag:
        for i in range(0, len(data.name)):
            joint_state.name.append(data.name[i])
            joint_state.position.append(data.position[i])

            #rospy.logwarn("joint name: %s", data.name[i])
        joint_start_flag = True
    else:
        for i in range(0, len(data.name)):
            joint_state.name[i] = data.name[i]
            joint_state.position[i] = data.position[i]

    joint_state.header = data.header

if __name__=="__main__":
    rospy.init_node("joint_control_test")

    gimbal_start_flag = False
    joint_start_flag = False
    gimbal_state = JointState()
    joint_state = JointState()

    duration = rospy.get_param("~duration", 0.05) #20Hz

    gimbal_state_topic_name = rospy.get_param("~gimbal_control_topic_name", "/dragon/gimbals_ctrl")
    rospy.Subscriber(gimbal_state_topic_name, JointState, gimbal_state_cb)
    joint_state_temp_topic_name = rospy.get_param("~joint_state_topic_name", "/dragon/joint_states_temp")
    rospy.Subscriber(joint_state_temp_topic_name, JointState, joint_state_cb)

    joint_state_topic_name = rospy.get_param("~joint_state_topic_name", "/dragon/joint_states")
    pub = rospy.Publisher(joint_state_topic_name, JointState, queue_size=10)

    cog2baselink_topic_name = rospy.get_param("~cog2baselink_topic_name", "/cog2baselink")
    rospy.Subscriber(cog2baselink_topic_name, TransformStamped, cog2baselink_cb)

    while not rospy.is_shutdown():

        if not gimbal_start_flag or not joint_start_flag:
            continue

        for i in range(0, len(gimbal_state.name)):
            joint_state.position[joint_state.name.index(gimbal_state.name[i])] = gimbal_state.position[i]

        pub.publish(joint_state)
        time.sleep(duration)
