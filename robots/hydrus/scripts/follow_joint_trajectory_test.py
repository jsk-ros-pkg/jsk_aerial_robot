#!/usr/bin/env python

import copy
import sys
import time
import rospy
import math
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectoryPoint

if __name__ == "__main__":

    rospy.init_node("follow_joint_trajectory_test")

    link_num = rospy.get_param("~link_num", 4)
    duration = rospy.get_param("~duration", 8)
    traj_points = rospy.get_param("~traj_points", 3)

    pub = rospy.Publisher("follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, queue_size=1)

    time.sleep(1)

    msg = FollowJointTrajectoryActionGoal()
    msg.header.stamp = rospy.Time.now()

    msg.goal.trajectory.header.stamp = rospy.Time(0.0)
    msg.goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
    init_point = JointTrajectoryPoint()
    init_point.positions = [2 * math.pi / link_num] * (link_num - 1)
    init_point.time_from_start = rospy.Duration(0.0)
    msg.goal.trajectory.points.append(init_point)

    for i in range(traj_points):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(msg.goal.trajectory.points[-1].positions)
        point.positions[i % (link_num-1)] =  -point.positions[i % (link_num-1)]
        point.time_from_start = msg.goal.trajectory.points[-1].time_from_start  + rospy.Duration(duration)
        msg.goal.trajectory.points.append(point)

    pub.publish(msg)
