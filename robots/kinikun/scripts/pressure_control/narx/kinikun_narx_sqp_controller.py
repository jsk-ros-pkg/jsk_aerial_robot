#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NARX+z 共通メタ (narx_common_meta.npz) を用いた 1-step SQP コントローラ（ROS）
出力はスケーリング済み /mpa_cmd (Vector3: x=p1_scaled, y=p2_scaled) のみ。

- 入力:
    目標角度: /target_angle [Float32, deg 既定] or ~topic_target_angle
    現在角度: /joint_states の ~theta_joint_name（rad想定）または ~topic_theta [Float32]
- 出力:
    /mpa_cmd (Vector3): x = p1[MPa]*scale, y = p2[MPa]*scale, z=0
      ※ scale = ~reg_scale (既定 0.9/4096)

最適化: SLSQP（SciPy）
  目的関数: J = wθ*(θ_pred - θ_ref)^2 + wz*z(ps,pd)^2 + wu*||Δu||^2
  変数    : u = [ps, pd]
  制約    : 0 <= p1 = (ps+pd)/2 <= p_ind_max
            0 <= p2 = (ps-pd)/2 <= p_ind_max
            （⇔ 0<=ps<=2*p_ind_max, |pd|<=ps）

npz 必須フィールド:
  degree, lag_y, lag_u, delay, coef[], z_coef[], z_feat_names[], dt
"""

import os, math
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# deps
try:
    from sklearn.preprocessing import PolynomialFeatures
    SKLEARN_OK = True
except Exception:
    SKLEARN_OK = False

try:
    from scipy.optimize import minimize
    SCIPY_OK = True
except Exception:
    SCIPY_OK = False


def clamp(x, lo, hi): return max(lo, min(hi, x))

def ps_pd_to_p12(ps, pd):
    return 0.5*(ps+pd), 0.5*(ps-pd)

def z_features(ps, pd, names):
    vals = {
        '1': 1.0,
        'ps': float(ps),
        'pd': float(pd),
        'ps*pd': float(ps)*float(pd),
        'ps^2': float(ps)*float(ps),
        'pd^2': float(pd)*float(pd),
    }
    return np.array([vals[nm] for nm in names], float)


class NarxSqpController(object):
    def __init__(self):
        if not SKLEARN_OK:
            raise RuntimeError("scikit-learn が必要です（PolynomialFeatures）。")
        if not SCIPY_OK:
            raise RuntimeError("SciPy が必要です（SLSQP 最適化）。")

        # ---- params ----
        self.meta_npz_path   = rospy.get_param("~meta_npz_path", "/home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/narx/narx_common_meta.npz")
        self.ctrl_rate_hz    = float(rospy.get_param("~ctrl_rate_hz", 100.0))
        self.theta_in_deg    = bool(rospy.get_param("~theta_in_deg", True))
        self.theta_joint_name= rospy.get_param("~theta_joint_name", "arm1_joint")  # 空なら topic_theta を使う
        self.topic_theta     = rospy.get_param("~topic_theta", "/kinikun1/joint_states")
        self.topic_target    = rospy.get_param("~topic_target_angle", "/target_angle")
        self.p_ind_max       = float(rospy.get_param("~p_ind_max_MPa", 0.70))
        self.w_theta         = float(rospy.get_param("~w_theta", 10.0))
        self.w_z             = float(rospy.get_param("~w_z", 1.0))
        self.w_u             = float(rospy.get_param("~w_u", 0.1))
        self.u_delta_clip    = float(rospy.get_param("~u_delta_clip_MPa", 0.10))
        self.reg_scale       = float(rospy.get_param("~reg_scale", 4096/0.9))  # 出力圧スケール（必要に応じて）
        self.verbose_every   = int(rospy.get_param("~verbose_every", 50))
        self.pub_topic       = rospy.get_param("~pub_cmd_p12", "/mpa_cmd")  # ← ここだけ publish

        # ---- load meta ----
        if not os.path.exists(self.meta_npz_path):
            raise RuntimeError("meta_npz_path が見つかりません: %s" % self.meta_npz_path)
        meta = np.load(self.meta_npz_path, allow_pickle=True)
        self.degree  = int(meta["degree"])
        self.lag_y   = int(meta["lag_y"])
        self.lag_u   = int(meta["lag_u"])
        self.delay   = int(meta["delay"])
        self.coef    = np.array(meta["coef"], dtype=float).reshape(-1)
        self.dt_meta = float(meta["dt"])
        self.z_coef  = np.array(meta["z_coef"], dtype=float).reshape(-1)
        self.z_names = [str(s) for s in meta["z_feat_names"]]
        # 省略可
        self.poly_feature_names = None
        if "poly_feature_names" in meta.files:
            try:
                self.poly_feature_names = [str(s) for s in meta["poly_feature_names"]]
            except Exception:
                pass

        # PolynomialFeatures の形を確定
        self.n_in = self.lag_y + 2*self.lag_u
        self.pf = PolynomialFeatures(degree=self.degree, include_bias=True)
        self.pf.fit(np.zeros((1, self.n_in), float))

        # 状態
        self.theta_now_deg = 0.0
        self.theta_ref_deg = 0.0
        self.have_theta = False
        self.have_ref   = False
        self.y_hist = [0.0]*max(1, self.lag_y)
        self.u_hist = [[0.0, 0.0]]*max(1, self.lag_u+self.delay)
        self.ps_prev = 0.0
        self.pd_prev = 0.0

        # pub/sub
        self.pub_p   = rospy.Publisher(self.pub_topic, Vector3, queue_size=1)
        if self.theta_joint_name:
            rospy.Subscriber("/kinikun1/joint_states", JointState, self._cb_jointstate, queue_size=10)
        else:
            rospy.Subscriber(self.topic_theta, Float32, self._cb_theta_float, queue_size=10)
        rospy.Subscriber(self.topic_target, Float32, self._cb_target, queue_size=10)

        self.rate = rospy.Rate(self.ctrl_rate_hz)
        self.step = 0

        rospy.loginfo("[NARX-SQP] loaded %s | degree=%d, lag_y=%d, lag_u=%d, delay=%d | publish -> %s (scaled %.6f)",
                      self.meta_npz_path, self.degree, self.lag_y, self.lag_u, self.delay,
                      self.pub_topic, self.reg_scale)

    # ---- callbacks ----
    def _cb_theta_float(self, msg):
        v = float(msg.data)
        self.theta_now_deg = v if self.theta_in_deg else (v*180.0/math.pi)
        self.have_theta = True

    def _cb_jointstate(self, msg):
        try:
            name = self.theta_joint_name
            if name in msg.name:
                idx = msg.name.index(name)
                v = float(msg.position[idx])  # rad
                self.theta_now_deg = (v*180.0/math.pi) if not self.theta_in_deg else v
                self.have_theta = True
        except Exception:
            pass

    def _cb_target(self, msg):
        v = float(msg.data)
        self.theta_ref_deg = v if self.theta_in_deg else (v*180.0/math.pi)
        self.have_ref = True

    # ---- narx predict ----
    def _narx_predict_deg(self, ps, pd):
        feats = []
        feats.extend(self.y_hist[:self.lag_y])
        start = self.delay
        for k in range(start, start+self.lag_u):
            ps_k, pd_k = self.u_hist[k]
            feats.extend([ps_k, pd_k])
        x  = np.array(feats, float).reshape(1, -1)
        xp = self.pf.transform(x)
        return float(np.dot(xp.reshape(-1), self.coef.reshape(-1)))

    # ---- z ----
    def _z_value(self, ps, pd):
        return float(np.dot(z_features(ps, pd, self.z_names), self.z_coef))

    # ---- objective ----
    def _objective(self, u):
        ps, pd = float(u[0]), float(u[1])
        e_th = self._narx_predict_deg(ps, pd) - self.theta_ref_deg
        zval = self._z_value(ps, pd)
        du2  = (ps - self.ps_prev)**2 + (pd - self.pd_prev)**2
        return self.w_theta*(e_th*e_th) + self.w_z*(zval*zval) + self.w_u*du2

    # ---- constraints ----
    def _bounds(self):
        ps_hi = 2.0*self.p_ind_max
        return [(0.0, ps_hi), (-ps_hi, ps_hi)]

    def _ineq_constraints(self):
        # |pd| <= ps
        return [{"type":"ineq", "fun": lambda u: float(u[0] - abs(u[1]))}]

    # ---- solve one step ----
    def _solve_once(self):
        u0 = np.array([self.ps_prev, self.pd_prev], float)
        res = minimize(self._objective, u0, method="SLSQP",
                       bounds=self._bounds(), constraints=self._ineq_constraints(),
                       options=dict(maxiter=50, ftol=1e-9, disp=False))
        ps, pd = (u0 if not res.success else res.x)
        # 1周期の変化クリップ
        ps = self.ps_prev + clamp(ps - self.ps_prev, -self.u_delta_clip, self.u_delta_clip)
        pd = self.pd_prev + clamp(pd - self.pd_prev, -self.u_delta_clip, self.u_delta_clip)
        # 個別圧へ＆クリップ
        p1, p2 = ps_pd_to_p12(ps, pd)
        p1 = clamp(p1, 0.0, self.p_ind_max)
        p2 = clamp(p2, 0.0, self.p_ind_max)
        # 再合成（安全側）
        ps = p1 + p2
        pd = p1 - p2
        return ps, pd, p1, p2

    def _update_hist(self, ps, pd, theta_deg):
        self.y_hist = [theta_deg] + self.y_hist[:-1]
        self.u_hist = [[ps, pd]] + self.u_hist[:-1]

    # ---- loop ----
    def spin(self):
        while not rospy.is_shutdown():
            if not self.have_theta:
                self.rate.sleep(); continue
            if not self.have_ref:
                self.theta_ref_deg = self.theta_now_deg  # 未受信なら追従

            ps, pd, p1, p2 = self._solve_once()

            # publish ONLY scaled /mpa_cmd
            self.pub_p.publish(Vector3(p1*self.reg_scale, p2*self.reg_scale, 0.0))

            # update internals
            self._update_hist(ps, pd, self.theta_now_deg)
            self.ps_prev, self.pd_prev = ps, pd

            if (self.step % self.verbose_every) == 0:
                th_pred = self._narx_predict_deg(ps, pd)
                zval = self._z_value(ps, pd)
                rospy.loginfo("[NARX-SQP] θ_now=%.3f θ_ref=%.3f θ_pred=%.3f | p1=%.3f p2=%.3f (scaled x%.6f) | z=%.4f",
                              self.theta_now_deg, self.theta_ref_deg, th_pred,
                              p1, p2, self.reg_scale, zval)
            self.step += 1
            self.rate.sleep()


def main():
    rospy.init_node("kinikun_narx_sqp_controller")
    try:
        NarxSqpController().spin()
    except Exception as e:
        rospy.logerr("Fatal: %s", str(e))

if __name__ == "__main__":
    main()
