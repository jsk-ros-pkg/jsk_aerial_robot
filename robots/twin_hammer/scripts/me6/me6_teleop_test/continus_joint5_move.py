#!/usr/bin/env python3
import rospy
import actionlib
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

if __name__ == "__main__":
    rospy.init_node("me6_incremental_joint5")

    client = actionlib.SimpleActionClient(
        "/me6_robot/joint_controller/follow_joint_trajectory",
        FollowJointTrajectoryAction
    )
    rospy.loginfo("Waiting for action server...")
    client.wait_for_server()
    rospy.loginfo("Connected to action server")

    joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]

    # 初期姿勢
    joint_positions = np.array([3.14, 0.0, 0.0, 0.0, -2.0, 0.0])

    rate = rospy.Rate(5)  # 1 Hz = 1秒ごとにゴール送信
    step = 0.5            # 1ステップの増分 [rad]

    init_flag = False

    while not rospy.is_shutdown():
        # joint5 を少しずつ増加
        joint_positions[4] += step

        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start = rospy.Duration(3.0)  # 1秒で到達

        traj = JointTrajectory()
        traj.joint_names = joint_names
        traj.points.append(point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj

        # ゴール送信
        client.send_goal(goal)
        rospy.loginfo(f"Sent new goal: joint5 = {joint_positions[4]:.2f} rad")
        if not init_flag:
            rospy.sleep(2.0)
            init_flag = True


        # 送信周期
        rate.sleep()
