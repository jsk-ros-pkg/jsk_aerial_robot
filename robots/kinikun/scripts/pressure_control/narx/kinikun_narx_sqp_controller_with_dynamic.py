#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
NARX+z + 逐次SLSQP 制御（/mpa_cmdのみPublish）
動的リコンフィグで：wの調整、p上限、Δuクリップ、reg_scale、
さらに 予測用 入力経路に 追加遅延・飽和・ヒステリシス を ON/OFF。

- 予測系（モデルに与える u_eff）:
    u_hist -> [Delay extra] -> [Saturation] -> [Hysteresis] -> NARX予測, z
- コマンド系（実出力）:
    通常はそのまま。apply_effects_to_cmd=True で同じ効果を p1,p2 側にも適用可能。

依存: rospy, numpy, dynamic_reconfigure, sklearn(PolynomialFeatures), scipy(optimize)
"""

import os, math
import numpy as np
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# dyn reconf
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from kinikun.cfg import KinikunControllerConfig

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
def ps_pd_to_p12(ps, pd): return 0.5*(ps+pd), 0.5*(ps-pd)
def p12_to_pspd(p1,p2):   return (p1+p2, p1-p2)

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

# --- 入力効果モデル（予測経路用） --------------------------------
class DelayLine:
    """追加遅延: 最新が先頭の u_hist(list of [ps,pd]) から extra遅延分を読み出す"""
    def __init__(self, extra_steps=0):
        self.set_steps(extra_steps)
    def set_steps(self, extra_steps):
        self.steps = max(0, int(extra_steps))
    def apply(self, u_hist_slice):
        # u_hist_slice: list (最新が先頭) 長さ >= steps+1 を想定
        k = self.steps
        k = min(k, len(u_hist_slice)-1)
        return list(u_hist_slice[k])

class SaturationStatic:
    """静的飽和: tanh型（スケールは p_ind_max 基準）"""
    def __init__(self, p_ind_max, g_ps=3.0, g_pd=3.0):
        self.p_ind_max = float(p_ind_max)
        self.g_ps = float(g_ps); self.g_pd = float(g_pd)
    def update(self, p_ind_max=None, g_ps=None, g_pd=None):
        if p_ind_max is not None: self.p_ind_max = float(p_ind_max)
        if g_ps is not None: self.g_ps = float(g_ps)
        if g_pd is not None: self.g_pd = float(g_pd)
    def apply(self, ps, pd):
        # ps∈[0,2pmax] 程度、pd∈[-ps,ps] 程度を想定
        # 正規化してtanh、戻す
        s = max(1e-9, self.p_ind_max)
        ps_sat = 2*s * math.tanh(self.g_ps * (ps/(2*s)))
        # pdの有効範囲は ±ps だが、あくまで非線形の型だけ当てる
        pd_sat = (2*s) * math.tanh(self.g_pd * (pd/(2*s)))
        return ps_sat, pd_sat

class PlayHysteresis:
    """
    単純な play 演算子: 閾値 h でデッドゾーンを持つヒステリシス
      if u > y + h: y = u - h
      elif u < y - h: y = u + h
      else: y = y
    さらに leak(0..1) で y <- (1-leak)*y + leak*u を混ぜ、gain を掛けて出力
    """
    def __init__(self, h_ps=0.02, h_pd=0.02, leak=0.02, gain=1.0):
        self.h_ps = float(h_ps); self.h_pd = float(h_pd)
        self.leak = float(leak); self.gain = float(gain)
        self.y_ps = 0.0
        self.y_pd = 0.0
    def update(self, h_ps=None, h_pd=None, leak=None, gain=None):
        if h_ps is not None: self.h_ps = float(h_ps)
        if h_pd is not None: self.h_pd = float(h_pd)
        if leak is not None: self.leak = float(leak)
        if gain is not None: self.gain = float(gain)
    def reset(self, y_ps=0.0, y_pd=0.0):
        self.y_ps = float(y_ps); self.y_pd = float(y_pd)
    def _play(self, u, y, h):
        if u > y + h: y = u - h
        elif u < y - h: y = u + h
        # leak toward input
        y = (1.0 - self.leak)*y + self.leak*u
        return y
    def apply(self, ps, pd):
        self.y_ps = self._play(ps, self.y_ps, self.h_ps)
        self.y_pd = self._play(pd, self.y_pd, self.h_pd)
        return self.gain*self.y_ps, self.gain*self.y_pd

# -------------------------------------------------------------------

class NarxSqpController(object):
    def __init__(self):
        if not SKLEARN_OK:  raise RuntimeError("scikit-learn が必要です（PolynomialFeatures）。")
        if not SCIPY_OK:    raise RuntimeError("SciPy が必要です（SLSQP 最適化）。")

        # ---- params (ROS param server) ----
        self.meta_npz_path   = rospy.get_param("~meta_npz_path", "/home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/narx/narx_common_meta.npz")
        self.ctrl_rate_hz    = float(rospy.get_param("~ctrl_rate_hz", 100.0))
        self.theta_in_deg    = bool(rospy.get_param("~theta_in_deg", True))
        self.theta_joint_name= rospy.get_param("~theta_joint_name", "")
        self.topic_theta     = rospy.get_param("~topic_theta", "/theta")
        self.topic_target    = rospy.get_param("~topic_target_angle", "/target_angle")
        self.pub_topic       = rospy.get_param("~pub_cmd_p12", "/mpa_cmd")

        # ---- load NARX meta ----
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

        # PF shape
        self.n_in = self.lag_y + 2*self.lag_u
        self.pf = PolynomialFeatures(degree=self.degree, include_bias=True)
        self.pf.fit(np.zeros((1, self.n_in), float))

        # 状態
        self.theta_now_deg = 0.0
        self.theta_ref_deg = 0.0
        self.have_theta = False
        self.have_ref   = False
        self.y_hist = [0.0]*max(1, self.lag_y)
        self.u_hist = [[0.0, 0.0]]*max(1, self.lag_u+self.delay+20)  # 余裕分
        self.ps_prev = 0.0
        self.pd_prev = 0.0

        # 予測用効果モデル（デフォはOFF想定、DRCで切替）
        self.delay_extra = DelayLine(extra_steps=0)
        self.sat_model   = SaturationStatic(p_ind_max=0.70, g_ps=3.0, g_pd=3.0)
        self.hys_model   = PlayHysteresis(h_ps=0.02, h_pd=0.02, leak=0.02, gain=1.0)

        # 動的リコンフィグ初期値（cfgに合わせる）
        self.cfg = None
        self.server = DynamicReconfigureServer(KinikunControllerConfig, self._on_reconf)

        # pub/sub
        self.pub_p = rospy.Publisher(self.pub_topic, Vector3, queue_size=1)
        if self.theta_joint_name:
            rospy.Subscriber("/joint_states", JointState, self._cb_jointstate, queue_size=10)
        else:
            rospy.Subscriber(self.topic_theta, Float32, self._cb_theta_float, queue_size=10)
        rospy.Subscriber(self.topic_target, Float32, self._cb_target, queue_size=10)

        self.rate = rospy.Rate(self.ctrl_rate_hz)
        self.step = 0

        rospy.loginfo("[NARX-SQP] loaded %s | degree=%d, lag_y=%d, lag_u=%d, delay=%d | publish -> %s",
                      self.meta_npz_path, self.degree, self.lag_y, self.lag_u, self.delay, self.pub_topic)

    # ---------- dynamic reconfigure ----------
    def _on_reconf(self, cfg, level):
        self.cfg = cfg
        # 依存更新
        self.sat_model.update(p_ind_max=cfg.p_ind_max_MPa, g_ps=cfg.sat_ps_gain, g_pd=cfg.sat_pd_gain)
        self.delay_extra.set_steps(cfg.delay_extra_steps)
        self.hys_model.update(h_ps=cfg.hyst_width_ps, h_pd=cfg.hyst_width_pd,
                              leak=cfg.hyst_leak, gain=cfg.hyst_gain)
        return cfg

    # ---------- callbacks ----------
    def _cb_theta_float(self, msg):
        v = float(msg.data)
        self.theta_now_deg = v if self.cfg and self.cfg.w_theta is not None and self.theta_in_deg \
            else (v*180.0/math.pi)
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

    # ---------- 入力効果（予測or指令に適用） ----------
    def _apply_effects_chain(self, ps, pd, for_prediction=True):
        """
        予測入力（ps,pd）に対して、DRCの設定に応じて
        [extra delay] -> [saturation] -> [hysteresis] を適用。
        for_prediction=False のときは、指令系に適用（通常False推奨）。
        """
        cfg = self.cfg
        if cfg is None:
            return ps, pd

        # 追加遅延（予測時は u_hist から取り出す。指令側に遅延は通常適用しない）
        if for_prediction and cfg.enable_delay_extra:
            # u_hist: 先頭が最新
            ps_d, pd_d = self.delay_extra.apply(self.u_hist)
        else:
            ps_d, pd_d = ps, pd

        # 飽和
        if cfg.enable_saturation:
            ps_s, pd_s = self.sat_model.apply(ps_d, pd_d)
        else:
            ps_s, pd_s = ps_d, pd_d

        # ヒステリシス（play）
        if cfg.enable_hysteresis:
            ps_h, pd_h = self.hys_model.apply(ps_s, pd_s)
        else:
            ps_h, pd_h = ps_s, pd_s

        return ps_h, pd_h

    # ---------- narx predict ----------
    def _narx_predict_deg(self, ps, pd):
        """
        現在までの履歴＆（必要なら効果を適用した）入力で1ステップ予測
        """
        cfg = self.cfg
        # 特徴ベクトル作成
        feats = []
        feats.extend(self.y_hist[:self.lag_y])

        # 入力ラグは meta.delay + (optionally extra delay は別途適用済みu_hist) とする。
        # ここでは meta.delay に基づき u_hist から抜き出す
        start = self.delay
        for k in range(start, start+self.lag_u):
            ps_k, pd_k = self.u_hist[k]
            # 予測入力に効果を適用したい場合、ここに chain を掛けても良いが、
            # 「履歴の u には適用しない・現在候補 u だけ適用」の方が直感的。
            feats.extend([ps_k, pd_k])

        x  = np.array(feats, float).reshape(1, -1)
        xp = self.pf.transform(x)
        return float(np.dot(xp.reshape(-1), self.coef.reshape(-1)))

    def _z_value(self, ps, pd):
        return float(np.dot(z_features(ps, pd, self.z_names), self.z_coef))

    # ---------- objective ----------
    def _objective(self, u):
        ps_raw, pd_raw = float(u[0]), float(u[1])

        # 予測用に効果を適用（候補uに対してのみ）
        ps_eff, pd_eff = self._apply_effects_chain(ps_raw, pd_raw, for_prediction=True)

        th_pred = self._narx_predict_deg(ps_eff, pd_eff)
        e_th = th_pred - self.theta_ref_deg
        zval = self._z_value(ps_eff, pd_eff)
        du2  = (ps_raw - self.ps_prev)**2 + (pd_raw - self.pd_prev)**2

        return self.cfg.w_theta*(e_th*e_th) + self.cfg.w_z*(zval*zval) + self.cfg.w_u*du2

    # ---------- constraints ----------
    def _bounds(self):
        ps_hi = 2.0*self.cfg.p_ind_max_MPa
        return [(0.0, ps_hi), (-ps_hi, ps_hi)]

    def _ineq_constraints(self):
        # |pd| <= ps
        return [{"type":"ineq", "fun": lambda u: float(u[0] - abs(u[1]))}]

    # ---------- solve one step ----------
    def _solve_once(self):
        u0 = np.array([self.ps_prev, self.pd_prev], float)
        res = minimize(self._objective, u0, method="SLSQP",
                       bounds=self._bounds(), constraints=self._ineq_constraints(),
                       options=dict(maxiter=50, ftol=1e-9, disp=False))
        ps, pd = (u0 if not res.success else res.x)

        # 1周期の変化クリップ（安全側）
        ps = self.ps_prev + clamp(ps - self.ps_prev, -self.cfg.u_delta_clip_MPa, self.cfg.u_delta_clip_MPa)
        pd = self.pd_prev + clamp(pd - self.pd_prev, -self.cfg.u_delta_clip_MPa, self.cfg.u_delta_clip_MPa)

        # 個別圧へ＆クリップ
        p1, p2 = ps_pd_to_p12(ps, pd)
        p1 = clamp(p1, 0.0, self.cfg.p_ind_max_MPa)
        p2 = clamp(p2, 0.0, self.cfg.p_ind_max_MPa)
        # 再合成
        ps, pd = p12_to_pspd(p1, p2)

        return ps, pd, p1, p2

    def _update_hist(self, ps, pd, theta_deg):
        # 最新先頭
        self.y_hist = [theta_deg] + self.y_hist[:-1]
        self.u_hist = [[ps, pd]]   + self.u_hist[:-1]

    # ---------- loop ----------
    def spin(self):
        while not rospy.is_shutdown():
            if self.cfg is None or not self.have_theta:
                self.rate.sleep(); continue
            if not self.have_ref:
                self.theta_ref_deg = self.theta_now_deg

            ps, pd, p1, p2 = self._solve_once()

            # 指令側にも効果を適用したい場合（通常はFalseでOK）
            if self.cfg.apply_effects_to_cmd:
                ps_cmd, pd_cmd = self._apply_effects_chain(ps, pd, for_prediction=False)
                p1, p2 = ps_pd_to_p12(ps_cmd, pd_cmd)
                p1 = clamp(p1, 0.0, self.cfg.p_ind_max_MPa)
                p2 = clamp(p2, 0.0, self.cfg.p_ind_max_MPa)

            # publish ONLY scaled /mpa_cmd
            self.pub_p.publish(Vector3(p1*self.cfg.reg_scale, p2*self.cfg.reg_scale, 0.0))

            # 内部更新（予測は実測θを履歴へ）
            self._update_hist(ps, pd, self.theta_now_deg)
            self.ps_prev, self.pd_prev = ps, pd

            if (self.step % 50) == 0:
                th_pred = self._narx_predict_deg(ps, pd)
                zval = self._z_value(ps, pd)
                rospy.loginfo("[NARX-SQP] θ_now=%.3f θ_ref=%.3f θ_pred=%.3f | p1=%.3f p2=%.3f | z=%.4f | eff_on: delay=%s sat=%s hyst=%s cmd_eff=%s",
                              self.theta_now_deg, self.theta_ref_deg, th_pred, p1, p2, zval,
                              str(self.cfg.enable_delay_extra), str(self.cfg.enable_saturation),
                              str(self.cfg.enable_hysteresis), str(self.cfg.apply_effects_to_cmd))
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
