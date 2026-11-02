#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
theta_target_bspline_node.py

/target_deg_cmd, /target_deg_cmd2 に入ってきた目標角度[deg]をトリガに、
現在のターゲット角度から目標角度までを B-spline で補間して、
/theta_target_deg, /theta_target_deg_2 を一定周期で publish するノード。

前提:
  - メッセージ型はすべて std_msgs/Float32（deg単位）
  - SciPy がインストールされていること (scipy.interpolate.BSpline)
"""

from __future__ import print_function

import rospy
from std_msgs.msg import Float32

import numpy as np
try:
    from scipy.interpolate import BSpline
except ImportError:
    raise ImportError("scipy が必要です: pip install scipy などでインストールしてください")


class ThetaTargetBSplineNode(object):
    def __init__(self):
        rospy.init_node("theta_target_bspline_node")

        # パラメータ
        self.rate_hz = rospy.get_param("~rate2", 100.0)       # publishレート [Hz]
        self.duration = rospy.get_param("~duration", 3.0)    # 補間にかける時間 [s]
        self.n_ctrl_pts = rospy.get_param("~n_ctrl_pts", 4)  # B-splineの制御点数（4でOK）
        self.degree = 3                                      # cubic B-spline固定

        if self.n_ctrl_pts < self.degree + 1:
            rospy.logwarn("n_ctrl_pts < degree+1 なので n_ctrl_pts を修正します")
            self.n_ctrl_pts = self.degree + 1

        # 現在のターゲット角度（deg）: これを基準に次のスプラインをつなぐ
        self.current1_deg = 0.0
        self.current2_deg = 0.0

        # アクティブなスプラインと開始時刻
        self.spline1 = None
        self.spline2 = None
        self.start_time1 = None
        self.start_time2 = None

        # Publisher / Subscriber
        self.pub1 = rospy.Publisher("/mppi/theta_target_deg", Float32, queue_size=1)
        self.pub2 = rospy.Publisher("/mppi/theta_target_deg_2", Float32, queue_size=1)

        self.sub_cmd1 = rospy.Subscriber("/mppi/target_deg_cmd", Float32,
                                         self.cb_cmd1, queue_size=1)
        self.sub_cmd2 = rospy.Subscriber("/mppi/target_deg_cmd2", Float32,
                                         self.cb_cmd2, queue_size=1)

        rospy.loginfo("[theta_target_bspline_node] Started")
        rospy.loginfo("  duration = %.3f [s], rate = %.1f [Hz]" %
                      (self.duration, self.rate_hz))

    # ===== B-spline 生成 =====

    def make_bspline(self, theta_start_deg, theta_goal_deg):
        """
        start → goal を結ぶ clamped cubic B-spline を生成。
        - 時間パラメータ t ∈ [0, duration]
        - 制御点は start〜goal を線形に並べるだけの簡単版
        """
        n = self.n_ctrl_pts
        k = self.degree

        # 制御点: start〜goal を線形に並べる
        ctrl = np.linspace(theta_start_deg, theta_goal_deg, n)

        # clamped knot vector を作る
        # 内部ノットの個数: n - k + 1
        t_internal = np.linspace(0.0, self.duration, n - k + 1)
        # 端点を k 回ずつ複製
        t = np.concatenate(([t_internal[0]] * k,
                            t_internal,
                            [t_internal[-1]] * k))

        spline = BSpline(t, ctrl, k)
        return spline

    # ===== コマンドコールバック =====

    def cb_cmd1(self, msg):
        """System1 用: /target_deg_cmd の新しい目標角度[deg]を受け取る"""
        goal_deg = float(msg.data)
        start_deg = self.current1_deg  # 今のターゲット値からつなぐ

        self.spline1 = self.make_bspline(start_deg, goal_deg)
        self.start_time1 = rospy.Time.now().to_sec()

        rospy.loginfo("[theta_target_bspline_node] New command1: %.2f deg (from %.2f)"
                      % (goal_deg, start_deg))

    def cb_cmd2(self, msg):
        """System2 用: /target_deg_cmd2 の新しい目標角度[deg]を受け取る"""
        goal_deg = float(msg.data)
        start_deg = self.current2_deg

        self.spline2 = self.make_bspline(start_deg, goal_deg)
        self.start_time2 = rospy.Time.now().to_sec()

        rospy.loginfo("[theta_target_bspline_node] New command2: %.2f deg (from %.2f)"
                      % (goal_deg, start_deg))

    # ===== メインループ =====

    def spin(self):
        rate = rospy.Rate(self.rate_hz)

        # 起動直後に 0deg を一度送っておく
        self.pub1.publish(Float32(self.current1_deg))
        self.pub2.publish(Float32(self.current2_deg))

        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()

            # --- System1 ---
            if self.spline1 is not None and self.start_time1 is not None:
                tau = now - self.start_time1
                if tau < 0.0:
                    tau = 0.0
                if tau >= self.duration:
                    tau = self.duration

                self.current1_deg = float(self.spline1(tau))
                # duration 終了後はスプラインを破棄
                if tau >= self.duration:
                    self.spline1 = None
                    self.start_time1 = None
            # スプラインが無くても current1_deg をそのまま出し続ける
            self.pub1.publish(Float32(self.current1_deg))

            # --- System2 ---
            if self.spline2 is not None and self.start_time2 is not None:
                tau2 = now - self.start_time2
                if tau2 < 0.0:
                    tau2 = 0.0
                if tau2 >= self.duration:
                    tau2 = self.duration

                self.current2_deg = float(self.spline2(tau2))
                if tau2 >= self.duration:
                    self.spline2 = None
                    self.start_time2 = None
            self.pub2.publish(Float32(self.current2_deg))

            rate.sleep()


def main():
    node = ThetaTargetBSplineNode()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
