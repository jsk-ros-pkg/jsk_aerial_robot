#!/usr/bin/env python3
import rospy
import numpy as np
from math import pi
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# dynamic_reconfigure
from dynamic_reconfigure.server import Server
from kinikun.cfg import AnglePIDConfig  # ← 下の cfg と対応

def bar_to_mpa(x):  return x / 10.0
def mpa_to_bar(x):  return x * 10.0

class AngleModelPidMPA:
    def __init__(self):
        # ==== モデル係数（あなたの最終fit; 圧力はbarスケールで有効） ====
        self.a = rospy.get_param("~a", 18.926031)
        self.b = rospy.get_param("~b", 3.653447)
        self.c = rospy.get_param("~c", 1.044299)
        self.d = rospy.get_param("~d", 6.341918)
        self.e = rospy.get_param("~e", 0.100000)

        # 幾何・オフセット
        self.L0_cm   = rospy.get_param("~L0_cm", 24.5)       # cm
        self.R_m     = rospy.get_param("~R_m", 0.1655)       # m
        self.r_m     = rospy.get_param("~r_m", 0.05094)      # m
        self.L_off_m = rospy.get_param("~L_off_m", 0.051)    # m
        self.use_old_sqrt = rospy.get_param("~use_old_sqrt", True)  # 旧 sqrt(p)補正を使う

        # 制御レンジ（すべて MPa）
        self.p_min_MPa = rospy.get_param("~p_min_MPa", 0.0)
        self.p_max_MPa = rospy.get_param("~p_max_MPa", 0.7)  # = 7 bar
        self.slew_MPa_per_s = rospy.get_param("~slew_MPa_per_s", 0.2)  # 出力の最大変化 [MPa/s]

        # 反対側圧力は常に 0 MPa（ご指定）
        self.p2_fixed_MPa = 0.0

        # ターゲット角度レンジ（0〜45°）
        self.theta_min = 0.0
        self.theta_max = pi/4.0  # 45 deg

        # 初期PID（単位は [MPa] を出すゲイン）
        self.kp = rospy.get_param("~kp", 0.30)   # MPa / rad
        self.ki = rospy.get_param("~ki", 0.00)   # MPa / (rad*s)
        self.kd = rospy.get_param("~kd", 0.02)   # MPa / (rad/s)
        self.i_clamp_MPa = rospy.get_param("~i_clamp_MPa", 0.05)
        self.d_filter_tau = rospy.get_param("~d_filter_tau", 0.05)  # [s]
        self.enable_ff = rospy.get_param("~enable_feedforward", True)

        # 状態
        self.target_angle = 0.0
        self.current_angle = 0.0
        self.have_angle = False
        self.int_err = 0.0
        self.prev_err = 0.0
        self.d_state = 0.0
        self.prev_p1 = 0.0
        self.prev_p2 = 0.0
        self.prev_t = None

        # ROS I/O
        self.sub_target = rospy.Subscriber("/target_angle", Float32, self.cb_target, queue_size=10)
        self.sub_js     = rospy.Subscriber("/kinikun1/joint_states", JointState, self.cb_joint, queue_size=10)
        self.pub_mpa    = rospy.Publisher("/mpa_cmd", Vector3, queue_size=10)
        self.pub_ff     = rospy.Publisher("~dbg_p_ff_MPa", Float32, queue_size=10)
        self.pub_pid    = rospy.Publisher("~dbg_p_pid_MPa", Float32, queue_size=10)

        # dynamic_reconfigurepub_p12
        self.srv = Server(AnglePIDConfig, self.reconf_cb)

        # 逆写像 LUT（θ→p）を構築（内部はbarで計算、入出力はMPa）
        self._build_inverse_table()

        rospy.loginfo("[angle_model_pid_mpa] ready. theta in [%.3f, %.3f] deg",
                      np.degrees(self.th_min), np.degrees(self.th_max))

    # ====== モデル（内部は bar） ======
    def L_core_cm(self, p_bar):
        term = (np.maximum(p_bar, 0.0) / self.c) ** self.d
        return self.a + self.b / ((1.0 + term) ** self.e)

    def L_model_cm(self, p_bar):
        L_cm = self.L_core_cm(p_bar)
        if self.use_old_sqrt:
            L_cm = L_cm - 0.009 * self.L0_cm * np.sqrt(np.maximum(p_bar, 0.0))
        return L_cm

    def theta_from_pbar(self, p_bar):
        L_m = self.L_model_cm(p_bar) / 100.0 - self.L_off_m
        x = (self.R_m**2 + self.r_m**2 - L_m**2) / (2.0 * self.R_m * self.r_m)
        x = np.clip(x, -1.0, 1.0)
        return np.pi/2.0 - np.arccos(x)

    def _build_inverse_table(self, n=2000):
        # p(bar)→theta(rad) の単調LUTを作る
        p_bar = np.linspace(mpa_to_bar(self.p_min_MPa), mpa_to_bar(self.p_max_MPa), n)
        th = self.theta_from_pbar(p_bar)
        th_mon = np.maximum.accumulate(th)  # 数値誤差での単調破りを修正
        self.p_grid_bar = p_bar
        self.th_grid = th_mon
        self.th_min = float(th_mon[0])
        self.th_max = float(th_mon[-1])

    def p_MPa_from_theta(self, theta_des):
        # 角度を0〜45degにクランプし、その範囲で逆補間
        theta_c = float(np.clip(theta_des, max(self.theta_min, self.th_min), 
                                           min(self.theta_max, self.th_max)))
        p_bar = float(np.interp(theta_c, self.th_grid, self.p_grid_bar))
        return bar_to_mpa(p_bar)

    # ====== dynamic_reconfigure ======
    def reconf_cb(self, cfg, level):
        self.kp = cfg.kp
        self.ki = cfg.ki
        self.kd = cfg.kd
        self.i_clamp_MPa = cfg.i_clamp_MPa
        self.d_filter_tau = max(1e-3, cfg.d_filter_tau)
        self.enable_ff = cfg.enable_feedforward
        self.slew_MPa_per_s = cfg.slew_MPa_per_s

        # 上限・下限やモデル係数を reconfigure したい場合はここで拾って LUT 再構築
        if (abs(cfg.p_max_MPa - self.p_max_MPa) > 1e-6) or (abs(cfg.p_min_MPa - self.p_min_MPa) > 1e-6):
            self.p_min_MPa = cfg.p_min_MPa
            self.p_max_MPa = cfg.p_max_MPa
            self._build_inverse_table()

        return cfg

    # ====== コールバック ======
    def cb_target(self, msg):
        # 0〜45degにクランプ
        th = float(msg.data)
        th = float(np.clip(th, self.theta_min, self.theta_max))
        self.target_angle = th

    def cb_joint(self, msg):
        if 'arm1_joint' in msg.name:
            idx = msg.name.index('arm1_joint')
            self.current_angle = float(msg.position[idx])
            self.have_angle = True

    # ====== ユーティリティ ======
    def _slew(self, prev, cmd, dt):
        if dt <= 0: return cmd
        dv = self.slew_MPa_per_s * dt
        return float(np.clip(cmd, prev - dv, prev + dv))

    def _sat(self, val, lo, hi):
        return float(np.clip(val, lo, hi))

    # ====== 制御ループ ======
    def step(self):
        t = rospy.Time.now().to_sec()
        if self.prev_t is None:
            self.prev_t = t
            return
        dt = max(1e-3, t - self.prev_t)
        self.prev_t = t

        if not self.have_angle:
            return

        # フィードフォワード: θ→p(MPa)
        p_ff = self.p_MPa_from_theta(self.target_angle) if self.enable_ff else 0.0

        # PID（角度[rad] → 圧力[MPa]）
        err = self.target_angle - self.current_angle

        # 積分・アンチワインドアップ（MPa基準）
        if self.ki > 0.0:
            self.int_err += err * dt
            lim = self.i_clamp_MPa / self.ki
            self.int_err = float(np.clip(self.int_err, -lim, +lim))
        else:
            self.int_err = 0.0

        # ローパス付きD
        raw_d = (err - self.prev_err) / dt
        alpha = np.exp(-dt / self.d_filter_tau)
        self.d_state = alpha * self.d_state + (1.0 - alpha) * raw_d
        self.prev_err = err

        p_pid = self.kp * err + self.ki * self.int_err + self.kd * self.d_state  # [MPa]

        # 片側加圧：p1 に (FF+PID) を、p2 は常に0 MPa
        p1_cmd = p_ff + p_pid
        p2_cmd = self.p2_fixed_MPa

        # 制限（スルー＋飽和）
        p1_cmd = self._sat(self._slew(self.prev_p1, p1_cmd, dt), self.p_min_MPa, self.p_max_MPa)
        p2_cmd = self._sat(self._slew(self.prev_p2, p2_cmd, dt), self.p_min_MPa, self.p_max_MPa)
        self.prev_p1, self.prev_p2 = p1_cmd, p2_cmd

        # 出力（MPa）
        v = Vector3()
        v.x = p1_cmd * 4096 / 0.9  # MPA→4096/0.9変換
        v.y = p2_cmd * 4096 / 0.9
        v.z = 0.0
        self.pub_mpa.publish(v)
        self.pub_ff.publish(Float32(p_ff))
        self.pub_pid.publish(Float32(p_pid))

def main():
    rospy.init_node("angle_model_pid_mpa")
    node = AngleModelPidMPA()
    rate = rospy.Rate(rospy.get_param("~rate_hz", 40.0))
    while not rospy.is_shutdown():
        node.step()
        rate.sleep()

if __name__ == "__main__":
    main()
