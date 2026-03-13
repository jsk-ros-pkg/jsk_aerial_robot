#!/usr/bin/env python3
import rospy
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
import math
import threading

class Me6DirectJointSender:
    def __init__(self):
        rospy.init_node("me6_direct_send_joint_angles")

        self.client = actionlib.SimpleActionClient(
            "/me6_robot/joint_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to server")

        # ---- joint state ----
        self.joint_names = ["joint1","joint2","joint3","joint4","joint5","joint6"]
        self.current_joint_pos = None
        self.lock = threading.Lock()

        # ---- target joint angles ---- #
        # joint_positions = [3.14, 0.7, -0.7, -1.57, 2.0, 1.57] # for force sensor safety
        # joint_positions = [1.57, 0.5, -2.5, 2.0, 1.57, 0.0] # for teleop initial point
        joint_positions = [2.37, -1.5, 2.0, -0.5, 0.77, 0.0] # for initial point in green base
        # joint_positions = [2.57, -2.2, 2.2, -1.5, 1.57, 0.0] # for brush wetting
        joint_positions = [2.07, 0.4, -0.4, -1.5, 1.5, 0.0] # for safety in green base
        # joint_positions = [0.87, -1.5, 2.0, -0.5, 2.27, 0.0]

        self.target_joint_angles = joint_positions
        self.max_joint_velocity = 0.1

        rospy.Subscriber("/me6_robot/joint_states", JointState, self.joint_state_cb)

    def joint_state_cb(self, msg):
        with self.lock:
            pos_dict = dict(zip(msg.name, msg.position))
            try:
                self.current_joint_pos = [
                    pos_dict[j] for j in self.joint_names
                ]
            except KeyError:
                pass  # 起動直後など

    def wait_for_joint_state(self):
        rospy.loginfo("Waiting for joint_states...")
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                if self.current_joint_pos is not None:
                    return
            rate.sleep()

    def send_joint_goal_with_velocity(self, target_positions, max_vel):
        """
        target_positions: [rad]
        max_vel: [rad/s] (スカラー or 関節数分のリスト)
        """

        with self.lock:
            current = self.current_joint_pos[:]

        if isinstance(max_vel, (int, float)):
            max_vel = [max_vel] * len(target_positions)

        # 各関節の必要時間を計算
        times = []
        for q0, q1, v in zip(current, target_positions, max_vel):
            dq = abs(q1 - q0)
            times.append(dq / v if v > 0 else 0.0)

        move_time = max(times)
        move_time = max(move_time, 1.0)  # safety

        rospy.loginfo(f"Computed move time: {move_time:.2f} sec")

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = rospy.Duration(move_time)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points.append(point)

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj

        self.client.send_goal(goal)
        rospy.loginfo("Goal sent, waiting for result...")
        self.client.wait_for_result()
        rospy.loginfo("Done!")

    def main(self):
        self.send_joint_goal_with_velocity(
            self.target_joint_angles,
            self.max_joint_velocity
        )


if __name__ == "__main__":
    sender = Me6DirectJointSender()

    sender.wait_for_joint_state()

    sender.main()
