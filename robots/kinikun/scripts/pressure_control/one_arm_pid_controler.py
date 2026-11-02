#!/usr/bin/env python3
"""
one_arm_pid_controller.py  ― 完成版
==================================
■ 機能
  - /kinikun1/joint_states から arm1_joint の現在角（ラジアン）を取得
  - /target_angle または rqt_reconfigure で与えた目標角（ラジアン）を追従
  - PID 計算 → p1, p2 圧力 (MPa) → 電圧値 v1, v2 (0-4096) に変換して mpa_cmd を publish
  - rqt_reconfigure で P, I, D, target_angle をリアルタイム調整

■ 依存
  - rospy, sensor_msgs/JointState, geometry_msgs/Vector3, std_msgs/Float64
  - dynamic_reconfigure (kinikun/cfg/PID.cfg)

■ 使い方
  $ rosrun kinikun one_arm_pid_controller.py
  別ターミナルで:
  $ rostopic pub /target_angle std_msgs/Float64 "data: 0.3"   # 目標 0.3 rad
  または rqt_reconfigure でスライダ調整
"""
import math
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3,Quaternion
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from kinikun.cfg import PIDConfig


class OneArmPID:
    def __init__(self):
        # ---------------- パラメータ ----------------
        self.rate_hz = rospy.get_param("~rate", 50)
        self.kp = rospy.get_param("~kp", 1.0)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.1)
        self.integral_limit = rospy.get_param("~integral_limit", 1.0)  # [rad·s]

        # 圧力マッピング
        self.pressure_base = rospy.get_param("~pressure_base", 0.35)   # MPa
        self.pressure_span = rospy.get_param("~pressure_span", 0.25)   # ±span
        self.tanh_scale = rospy.get_param("~tanh_scale", 5.0)

        # ---------------- 内部状態 ----------------
        self.target_angle = 0.0          # [rad]
        self.current_angle = 0.0         # [rad]
        self.prev_error = 0.0
        self.integral = 0.0

        # ---------------- Dynamic Reconfigure ----------------
        self.server = Server(PIDConfig, self.reconfigure_cb)

        # ---------------- Subscriber / Publisher ----------------
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.joint_cb, queue_size=10)
        rospy.Subscriber("/target_angle", Float64, self.target_cb, queue_size=10)
        self.pub_mpa = rospy.Publisher("mpa_cmd", Quaternion, queue_size=10)

        # ---------------- 制御ループ ----------------
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self.control_loop)

    # ===== コールバック =====
    def reconfigure_cb(self, cfg, level):
        self.kp = cfg.p_gain
        self.ki = cfg.i_gain
        self.kd = cfg.d_gain
        self.target_angle = cfg.target_angle
        rospy.loginfo(f"[Reconf] P={self.kp:.3f}, I={self.ki:.3f}, D={self.kd:.3f}, Target={self.target_angle:.3f} rad")
        return cfg

    def joint_cb(self, msg):
        try:
            idx = msg.name.index("arm1_joint")
            self.current_angle = msg.position[idx]   # ラジアン
        except ValueError:
            pass  # 該当なしなら無視

    def target_cb(self, msg):
        self.target_angle = msg.data                # ラジアン

    # ===== メイン制御 =====
    def control_loop(self, _event):
        error = self.target_angle - self.current_angle

        # 積分項（アンチワインドアップ）
        self.integral += error / self.rate_hz
        self.integral = max(-self.integral_limit, min(self.integral, self.integral_limit))

        # 微分項
        derivative = (error - self.prev_error) * self.rate_hz
        self.prev_error = error

        # PID 出力
        u = self.kp * error + self.ki * self.integral + self.kd * derivative

        # --- 圧力へマッピング（tanh で滑らかに飽和）---
        p1 = self.pressure_base - self.pressure_span * math.tanh(u / self.tanh_scale)
        p2 = self.pressure_base + self.pressure_span * math.tanh(u / self.tanh_scale)

        # 範囲 [0.1, 0.6] MPa にクリップ
        p1 = max(0.1, min(0.6, p1))
        p2 = max(0.1, min(0.6, p2))

        # --- MPa → DAC 値 (0-4096) へ変換 ---
        v1 = p1 * 4096 / 0.9
        v2 = p2 * 4096 / 0.9

        # Publish
        self.pub_mpa.publish(Quaternion(v1, v2, 0.0, 0.0))

        rospy.loginfo_throttle(
            0.5,
            f"Tgt={self.target_angle:+.3f} rad  Cur={self.current_angle:+.3f} rad  "
            f"u={u:+.2f}  p1={p1:.3f} p2={p2:.3f}"
        )


if __name__ == "__main__":
    rospy.init_node("one_arm_pid_controller")
    try:
        OneArmPID()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
