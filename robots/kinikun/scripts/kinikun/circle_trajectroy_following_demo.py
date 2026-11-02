#!/usr/bin/env python

import sys
import time
import rospy
import math
import signal
from aerial_robot_msgs.msg import FlightNav, PoseControlPid
from std_msgs.msg import Bool

class CircTrajFollow():
  def __init__(self):
    # パラメータ
    self.period = rospy.get_param("~period", 40.0)
    self.radius = rospy.get_param("~radius", 0.5)
    self.init_theta = rospy.get_param("~init_theta", 0.0)
    self.yaw = rospy.get_param("~yaw", False)
    self.loop_count = rospy.get_param("~loop_count", 3)  # 0: 無限ループ, 1以上: 指定回数

    # パブリッシャー
    self.nav_pub = rospy.Publisher("/kinikun1/uav/nav", FlightNav, queue_size=1)
    self.tracking_flag_pub = rospy.Publisher("~is_tracking", Bool, queue_size=1, latch=True)

    # サブスクライバー
    self.control_sub = rospy.Subscriber("/kinikun1/debug/pose/pid", PoseControlPid, self.controlCb)

    # 状態変数
    self.center_pos_x = None
    self.center_pos_y = None
    self.initial_target_yaw = 0.0
    self.current_loop = 0

    # 軌道パラメータ
    self.omega = 2 * math.pi / self.period
    self.velocity = self.omega * self.radius

    # 制御レート
    self.nav_rate = rospy.get_param("~nav_rate", 20.0)  # Hz
    self.nav_rate = 1.0 / self.nav_rate

    # FlightNavメッセージの初期化
    self.flight_nav = FlightNav()
    self.flight_nav.target = FlightNav.COG
    self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
    if self.yaw:
      self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE

    # シグナルハンドラ
    signal.signal(signal.SIGINT, self.stopRequest)

    # 初期状態をパブリッシュ
    self.tracking_flag_pub.publish(Bool(data=False))

    time.sleep(0.5)

    rospy.loginfo("=== Circle Trajectory Follower ===")
    rospy.loginfo("Period: %.2f s, Radius: %.2f m", self.period, self.radius)
    rospy.loginfo("Loop count: %s", "infinite" if self.loop_count == 0 else str(self.loop_count))

  def controlCb(self, msg):
    self.initial_target_yaw = msg.yaw.target_p

    self.center_pos_x = msg.x.target_p - math.cos(self.init_theta) * self.radius
    self.center_pos_y = msg.y.target_p - math.sin(self.init_theta) * self.radius

    rospy.loginfo("Center position: [%.3f, %.3f]", self.center_pos_x, self.center_pos_y)

    self.control_sub.unregister()

  def stopRequest(self, signal, frame):
    rospy.loginfo("Stop requested")
    self.stopTracking()
    sys.exit(0)

  def stopTracking(self):
    """追従を停止し、速度をゼロにする"""
    self.flight_nav.target_vel_x = 0
    self.flight_nav.target_vel_y = 0
    self.flight_nav.target_omega_z = 0
    self.nav_pub.publish(self.flight_nav)
    self.tracking_flag_pub.publish(Bool(data=False))
    rospy.loginfo("Tracking stopped")

  def main(self):
    cnt = 0
    steps_per_loop = int(self.period / self.nav_rate)

    while not rospy.is_shutdown():

      # 初期位置を待つ
      if self.center_pos_x is None:
        rospy.loginfo_throttle(1.0, "Waiting for controller message...")
        time.sleep(self.nav_rate)
        continue

      # 追従開始時にフラグをTrue
      if cnt == 0 and self.current_loop == 0:
        self.tracking_flag_pub.publish(Bool(data=True))
        rospy.loginfo("Tracking started")

      # 軌道計算
      theta = self.init_theta + cnt * self.nav_rate * self.omega

      # 目標位置
      self.flight_nav.target_pos_x = self.center_pos_x + math.cos(theta) * self.radius
      self.flight_nav.target_pos_y = self.center_pos_y + math.sin(theta) * self.radius

      # 目標速度（接線方向）
      self.flight_nav.target_vel_x = -math.sin(theta) * self.velocity
      self.flight_nav.target_vel_y = math.cos(theta) * self.velocity

      # ヨー制御
      if self.yaw:
        self.flight_nav.target_yaw = self.initial_target_yaw + cnt * self.nav_rate * self.omega
        self.flight_nav.target_omega_z = self.omega

      self.nav_pub.publish(self.flight_nav)

      cnt += 1

      # 1周完了判定
      if cnt >= steps_per_loop:
        self.current_loop += 1
        rospy.loginfo("Loop %d/%s completed",
                      self.current_loop,
                      "inf" if self.loop_count == 0 else str(self.loop_count))

        # 指定回数に達したか確認
        if self.loop_count > 0 and self.current_loop >= self.loop_count:
          time.sleep(0.1)
          self.stopTracking()
          break

        # 次のループへ
        cnt = 0

      time.sleep(self.nav_rate)

    rospy.loginfo("Finished %d loop(s)", self.current_loop)


if __name__ == "__main__":
  rospy.init_node("circle_trajectory_follow")

  tracker = CircTrajFollow()
  tracker.main()