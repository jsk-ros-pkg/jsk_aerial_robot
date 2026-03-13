#!/usr/bin/env python3
import rospy
import actionlib
import threading
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState


class Me6DirectJointSender:
    def __init__(self):
        rospy.init_node("me6_direct_send_joint_angles")

        # --------------------------------------------------
        # Action client
        # --------------------------------------------------
        self.client = actionlib.SimpleActionClient(
            "/me6_robot/joint_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to action server")

        # --------------------------------------------------
        # Joint state
        # --------------------------------------------------
        self.joint_names = [
            "joint1", "joint2", "joint3",
            "joint4", "joint5", "joint6"
        ]

        self.current_joint_pos = None
        self.lock = threading.Lock()

        rospy.Subscriber("/me6_robot/joint_states", JointState, self.joint_state_cb)

        # --------------------------------------------------
        # User-defined joint sequence [rad]
        # --------------------------------------------------
        self.joint_sequence = [
          #  # 中間姿勢1
          #  [0.87, -1.7, 2.0, -0.3, 2.27, 0.0],

          #  # 中間姿勢2
          #  [1.57, -2.2, 2.4, -0.2, 1.57, 0.0],

          #  # 中間姿勢1
          #  [2.37, -1.4, 0.7, 0.9, 0.77, 0.0],

           # 中間姿勢2
           [2.37, -1.4, 1.7, 0.7, 0.77, 0.0],

           # 中間姿勢3
           [2.57, -2.0, 2.0, 0.9, 1.57, 0.0],

           # 撮影姿勢
           [2.57, -2.2, 2.2, -1.5, 1.57, 0.0]
        ]

        # --------------------------------------------------
        # Motion parameters
        # --------------------------------------------------
        self.max_joint_velocity = 0.2   # [rad/s]
        self.joint_tolerance = 0.05     # [rad] 到達判定誤差
        self.settle_time = 0.5          # [sec] 到達後の安定待ち

    # ==================================================
    # Callback
    # ==================================================
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

    # ==================================================
    # Utility
    # ==================================================
    def wait_until_reached(self, target, tol):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with self.lock:
                current = self.current_joint_pos[:]

            errors = [abs(c - t) for c, t in zip(current, target)]
            if max(errors) < tol:
                return

            rate.sleep()

    # ==================================================
    # Motion command
    # ==================================================
    def send_joint_goal_with_velocity(self, target_positions, max_vel):
        with self.lock:
            current = self.current_joint_pos[:]

        if isinstance(max_vel, (int, float)):
            max_vel = [max_vel] * len(target_positions)

        # 各関節の移動時間計算
        times = []
        for q0, q1, v in zip(current, target_positions, max_vel):
            dq = abs(q1 - q0)
            times.append(dq / v if v > 0 else 0.0)

        move_time = max(max(times), 1.0)  # 最低秒の安全時間

        rospy.loginfo(f"Move time: {move_time:.2f} sec")

        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.time_from_start = rospy.Duration(move_time)

        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        traj.points = [point]

        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj

        # ① ゴール送信
        self.client.send_goal(goal)

        # ② Action 完了待ち
        self.client.wait_for_result()

        # ③ 実関節角の到達待ち
        rospy.loginfo("Waiting until joints reach target...")
        self.wait_until_reached(target_positions, self.joint_tolerance)

        # ④ 安定待ち
        rospy.sleep(self.settle_time)

    # ==================================================
    # Sequence execution
    # ==================================================
    def execute_joint_sequence(self):
        rospy.loginfo("Executing joint sequence")

        for i, target in enumerate(self.joint_sequence):
            rospy.loginfo(f"Step {i+1}/{len(self.joint_sequence)}")
            self.send_joint_goal_with_velocity(
                target,
                self.max_joint_velocity
            )

        rospy.loginfo("Joint sequence completed")

    # ==================================================
    def main(self):
        self.execute_joint_sequence()


if __name__ == "__main__":
    sender = Me6DirectJointSender()
    sender.wait_for_joint_state()
    sender.main()
