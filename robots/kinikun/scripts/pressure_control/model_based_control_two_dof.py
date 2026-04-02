#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, importlib.util, math
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

def load_predictor(py_path):
    spec = importlib.util.spec_from_file_location("pred", py_path)
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    # 期待: f_static(ps, pd) と tau0,tau1（なければ param で指定）
    if not hasattr(mod, "f_static"):
        raise RuntimeError("predictorに f_static(ps,pd) がありません")
    return mod

def clip(x, lo, hi): return max(lo, min(hi, x))

def brent_or_bisect(f, a, b, tol=1e-4, itmax=60):
    fa, fb = f(a), f(b)
    # 符号が同じなら最小誤差点を端で返す
    if fa*fb > 0:
        fae, fbe = abs(fa), abs(fb)
        return a if fae <= fbe else b, (fa if fae <= fbe else fb)
    # 符号反転あり→二分法（堅牢）
    lo, hi, flo, fhi = a, b, fa, fb
    for _ in range(itmax):
        mid = 0.5*(lo+hi); fmid = f(mid)
        if abs(hi-lo) < tol or abs(fmid) < 1e-5:
            return mid, fmid
        if flo*fmid <= 0:
            hi, fhi = mid, fmid
        else:
            lo, flo = mid, fmid
    mid = 0.5*(lo+hi); return mid, f(mid)

class FFController:
    def __init__(self):
        # === パラメータ ===
        self.predictor_path = rospy.get_param("~predictor_path", "theta_dyn_predictor.py")
        self.joint_topic = rospy.get_param("~joint_topic", "/kinikun1/joint_states")
        self.joint_name  = rospy.get_param("~joint_name",  "arm1_joint")
        self.target_topic= rospy.get_param("~target_topic","/target_angle")
        self.target_in_deg = rospy.get_param("~target_in_deg", False)

        self.pub_topic_p12 = rospy.get_param("~pub_topic_p12", "/p1p2_cmd")  # MPa
        self.also_pub_mpa_cmd = rospy.get_param("~also_pub_mpa_cmd", False)
        self.pub_topic_mpa_cmd = rospy.get_param("~pub_topic_mpa_cmd", "/mpa_cmd")
        self.raw_counts_per_MPa = rospy.get_param("~raw_counts_per_MPa", 4096.0/0.9)  # 逆スケール（必要時）

        self.pmax = rospy.get_param("~p_max_MPa", 0.8)
        self.ps_min = rospy.get_param("~ps_min_MPa", 0.02)
        self.ps_max = rospy.get_param("~ps_max_MPa", 2.0*self.pmax)

        # τスケジューラ
        self.tau0 = rospy.get_param("~tau0", None)
        self.tau1 = rospy.get_param("~tau1", None)
        self.tau_min = rospy.get_param("~tau_min", 0.05)
        self.tau_max = rospy.get_param("~tau_max", 1.5)
        self.tau_fast = rospy.get_param("~tau_fast", 0.12)
        self.k_tau = rospy.get_param("~k_tau", 0.6)
        self.alpha_tau = rospy.get_param("~alpha_tau", 8.0)

        # レート制限
        self.dps_max = rospy.get_param("~dps_max_MPa_s", 1.0)   # 和圧
        self.dpd_max = rospy.get_param("~dpd_max_MPa_s", 1.0)   # 差圧

        # 小さなPI（任意）
        self.use_pi = rospy.get_param("~use_pi", False)
        self.kp_pd  = rospy.get_param("~kp_pd", 0.0)
        self.ki_pd  = rospy.get_param("~ki_pd", 0.0)
        self.int_e  = 0.0

        # 予測器ロード
        self.pred = load_predictor(self.predictor_path)
        if self.tau0 is None: self.tau0 = float(getattr(self.pred, "tau0", 0.2))
        if self.tau1 is None: self.tau1 = float(getattr(self.pred, "tau1", -0.1))

        # 状態
        self.theta = None
        self.theta_ref = 0.0
        self.ps_cmd = 0.05
        self.pd_cmd = 0.0
        self.last_t = rospy.Time.now().to_sec()

        # ROS I/O
        rospy.Subscriber(self.joint_topic, JointState, self.cb_joint)
        rospy.Subscriber(self.target_topic, Float32, self.cb_target)
        self.pub_p12 = rospy.Publisher(self.pub_topic_p12, Vector3, queue_size=10)
        self.pub_mpa = rospy.Publisher(self.pub_topic_mpa_cmd, Vector3, queue_size=10) if self.also_pub_mpa_cmd else None

    def cb_target(self, msg):
        val = float(msg.data)
        self.theta_ref = math.radians(val) if self.target_in_deg else val

    def cb_joint(self, msg):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            self.theta = float(msg.position[i])

    def f_static(self, ps, pd):
        # predictorのf_staticはベクトルも受ける実装の想定
        import numpy as np
        return float(self.pred.f_static(np.array([ps]), np.array([pd]))[0]) if callable(self.pred.f_static) else 0.0

    def rate_limit(self, target, current, rate_max, dt):
        step = clip(target - current, -rate_max*dt, rate_max*dt)
        return current + step

    def compute(self, dt):
        if self.theta is None:
            return None, None

        e = self.theta_ref - self.theta

        # 1) τターゲット（誤差が大きいほど速く → 小さいτ）
        tau_des = self.tau_fast + self.k_tau / (1.0 + self.alpha_tau*abs(e))
        tau_des = clip(tau_des, self.tau_min, self.tau_max)

        # 2) 和圧を一意化（tau(ps)=tau0+tau1*ps）
        # tau1 が負の場合が多い想定
        if abs(self.tau1) < 1e-6:
            ps_des = self.ps_min
        else:
            ps_des = (tau_des - self.tau0)/self.tau1
        ps_des = clip(ps_des, self.ps_min, self.ps_max)

        # 3) 差圧の1D根探し（f(ps_des,pd)=theta_ref）
        pd_lim = min(ps_des, 2.0*self.pmax - ps_des)
        a, b = -pd_lim, +pd_lim

        def g(pd): return self.f_static(ps_des, pd) - self.theta_ref

        pd_des, gval = brent_or_bisect(g, a, b, tol=1e-4, itmax=60)
        # ルートがなければ最小誤差端点が返る（実装内）

        # 4) 任意の小PIで微整形（差圧にだけ）
        if self.use_pi:
            self.int_e += e*dt
            pd_des += self.kp_pd*e + self.ki_pd*self.int_e

        # 5) レート制限
        self.ps_cmd = self.rate_limit(ps_des, self.ps_cmd, self.dps_max, dt)
        self.pd_cmd = self.rate_limit(pd_des, self.pd_cmd, self.dpd_max, dt)

        # 6) 変換＆飽和
        p1 = 0.5*(self.ps_cmd + self.pd_cmd)
        p2 = 0.5*(self.ps_cmd - self.pd_cmd)
        p1 = clip(p1, 0.0, self.pmax)
        p2 = clip(p2, 0.0, self.pmax)
        return p1, p2

    def spin(self):
        rate_hz = rospy.get_param("~rate_hz", 100.0)
        r = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt = max(1e-3, now - self.last_t); self.last_t = now
            p = self.compute(dt)
            if p is not None:
                p1, p2 = p
                msg = Vector3(x=p1, y=p2, z=0.0)
                self.pub_p12.publish(msg)
                if self.pub_mpa is not None:
                    # raw counts（必要時のみ）
                    raw1 = int(round(p1 * self.raw_counts_per_MPa))
                    raw2 = int(round(p2 * self.raw_counts_per_MPa))
                    self.pub_mpa.publish(Vector3(x=raw1, y=raw2, z=0))
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("model_ff_controller")
    FFController().spin()
