#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Neural-Hammerstein predictive inverse (MPC-like) ROS node for kinikun.
- Internals use degrees [deg] for all angles (matches trained model).
- JointState input (rad) is converted to deg.
- Target angle: if ~target_in_deg==false, rad->deg; if true, already deg.
- No pressure sensors: uses commanded pressure history for plant delay.
- Keeps pressure when target is reached ("hold") instead of venting to zero.

Publish:
  ~pub_topic_p12 (geometry_msgs/Vector3)  x=p1[MPa], y=p2[MPa]
Optionally:
  ~pub_topic_mpa (geometry_msgs/Vector3)  raw DAC counts if needed

Dependencies: rospy, numpy, torch, (scipy optional for LS solver)
"""

import os, math, importlib
import numpy as np

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# ---- try torch (required) ----
try:
    import torch
    import torch.nn as nn
except Exception as e:
    raise RuntimeError("PyTorch is required: pip install torch") from e

# ---- optional scipy ----
try:
    from scipy.optimize import least_squares
    _HAS_SCIPY = True
except Exception:
    _HAS_SCIPY = False


# =============== Neural static model (must match training) ===============
class MonoDeltaNN(nn.Module):
    """
    θ_stat(Σ̂,Δ̂) = A(Σ̂) + Σ_j softplus(w_j(Σ̂))*tanh( softplus(s_j(Σ̂))*(Δ̂ - c_j) )
    Inputs are normalized: Σ̂=(Σ-muS)/sdS, Δ̂=(Δ-muD)/sdD
    Output θ_stat in [deg].
    """
    def __init__(self, M, hidden, c_grid):
        super().__init__()
        self.M = int(M)
        self.c = nn.Parameter(torch.tensor(c_grid, dtype=torch.float32), requires_grad=False)
        self.enc = nn.Sequential(
            nn.Linear(1, hidden), nn.ELU(),
            nn.Linear(hidden, hidden), nn.ELU()
        )
        self.head_A = nn.Linear(hidden, 1)
        self.head_w = nn.Linear(hidden, self.M)
        self.head_s = nn.Linear(hidden, self.M)
        # match training init
        nn.init.zeros_(self.head_w.weight); nn.init.zeros_(self.head_w.bias)
        nn.init.zeros_(self.head_s.weight); nn.init.constant_(self.head_s.bias, 1.0)

    def forward(self, Sigma_hat, Delta_hat):
        h = self.enc(Sigma_hat)
        A = self.head_A(h)
        w = torch.nn.functional.softplus(self.head_w(h)) + 1e-6
        s = torch.nn.functional.softplus(self.head_s(h)) + 1e-6
        d = Delta_hat - self.c.view(1, -1)
        bank = torch.tanh(s * d)
        return A + (w * bank).sum(dim=1, keepdim=True)  # [N,1] in deg


def make_theta_stat_from_files(meta_path, pt_path):
    meta = np.load(meta_path, allow_pickle=True)
    state = torch.load(pt_path, map_location="cpu")

    M = int(meta["M"])
    hidden = int(meta["hidden"])
    c_grid = meta["c_grid"]
    muS, sdS = float(meta["muS"]), float(meta["sdS"])
    muD, sdD = float(meta["muD"]), float(meta["sdD"])

    model = MonoDeltaNN(M, hidden, c_grid)
    model.load_state_dict(state)
    model.eval()

    @torch.no_grad()
    def theta_stat(ps, pd):
        """
        ps,pd: scalar MPa
        returns θ_stat in deg
        """
        S = torch.tensor([(ps - muS) / sdS], dtype=torch.float32).view(1, 1)
        D = torch.tensor([(pd - muD) / sdD], dtype=torch.float32).view(1, 1)
        y = model(S, D)
        return float(y.item())

    # dynamics & constraints from meta
    dyn = dict(alpha=float(meta["alpha"]),
               kS=float(meta["kSigma"]),
               kD=float(meta["kDelta"]),
               delay=int(meta["delay"]),
               dt=float(meta["dt"]))
    limits = dict(pmax=float(meta["pmax"]),
                  sigma_ref=float(meta["sigma_ref"]))
    return theta_stat, dyn, limits


# =============== Utilities ===============
def clamp_box(p1, p2, pmax):
    return (max(0.0, min(p1, pmax)), max(0.0, min(p2, pmax)))

def pd_limits(ps, pmax):
    # pd is limited by p1>=0 and p2>=0:
    # p1=(ps+pd)/2 >=0  => pd>=-ps
    # p2=(ps-pd)/2 >=0  => pd<= ps
    # and by p1<=pmax, p2<=pmax:
    # (ps+pd)/2 <= pmax => pd <= 2*pmax - ps
    # (ps-pd)/2 <= pmax => -pd <= 2*pmax - ps => pd >= ps - 2*pmax
    lo = max(-ps, ps - 2.0*pmax)
    hi = min(ps, 2.0*pmax - ps)
    return lo, hi

def invert_delta_bisect(ps, theta_target_deg, theta_stat_fn, pmax, n_it=50):
    """Monotone in Δ assumption. Find pd s.t. theta_stat(ps,pd)=theta_target."""
    lo, hi = pd_limits(ps, pmax)
    # widen a bit if degenerate
    if hi - lo < 1e-6:
        lo, hi = lo - 0.5, hi + 0.5
    # if sign not bracketing, return the closer endpoint
    f_lo = theta_stat_fn(ps, lo) - theta_target_deg
    f_hi = theta_stat_fn(ps, hi) - theta_target_deg
    if f_lo * f_hi > 0:
        return lo if abs(f_lo) <= abs(f_hi) else hi
    for _ in range(n_it):
        mid = 0.5*(lo + hi)
        f_m = theta_stat_fn(ps, mid) - theta_target_deg
        if abs(f_m) < 1e-4:
            return mid
        if f_lo * f_m <= 0:
            hi, f_hi = mid, f_m
        else:
            lo, f_lo = mid, f_m
    return 0.5*(lo + hi)

def rate_limit(target, current, rate_max_per_s, dt):
    step = max(-rate_max_per_s*dt, min(rate_max_per_s*dt, target - current))
    return current + step


# =============== Main Node ===============
class KinikunNeuralMPCNode(object):
    def __init__(self):
        # --- model files (default to your workspace paths) ---
        self.model_pt   = rospy.get_param("~model_pt",
            os.path.expanduser("~") + "/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh.pt")
        self.model_meta = rospy.get_param("~model_meta",
            os.path.expanduser("~") + "/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh_meta.npz")

        # topics & inputs
        self.joint_topic   = rospy.get_param("~joint_topic", "/kinikun1/joint_states")
        self.joint_name    = rospy.get_param("~joint_name",  "arm1_joint")
        self.target_topic  = rospy.get_param("~target_topic", "/target_angle")
        self.target_in_deg = bool(rospy.get_param("~target_in_deg", False))  # false => rad input

        # outputs
        self.pub_topic_p12 = rospy.get_param("~pub_topic_p12", "/p1p2_cmd")  # MPa
        self.also_pub_mpa_cmd = bool(rospy.get_param("~also_pub_mpa_cmd", True))
        self.pub_topic_mpa_cmd = rospy.get_param("~pub_topic_mpa_cmd", "/mpa_cmd")
        self.raw_counts_per_MPa = float(rospy.get_param("~raw_counts_per_MPa", 4096.0/0.9))

        # MPC hyperparams
        self.H = int(rospy.get_param("~H", 10))
        self.w_stage = float(rospy.get_param("~w_stage", 1.0))
        self.w_term  = float(rospy.get_param("~w_term", 5.0))
        self.w_sigma = float(rospy.get_param("~w_sigma", 0.02))
        self.w_rate  = float(rospy.get_param("~w_rate", 0.3))
        self.w_block = float(rospy.get_param("~w_block", 10.0))
        self.block   = int(rospy.get_param("~block", 2))
        self.rate_sigma = float(rospy.get_param("~rate_sigma", 0.12))  # MPa/step (cmd)
        self.rate_delta = float(rospy.get_param("~rate_delta", 0.12))
        self.max_nfev   = int(rospy.get_param("~max_nfev", 600))

        # HOLD logic
        self.hold_deadband_deg = float(rospy.get_param("~hold_deadband_deg", 1.0))
        self.hold_rate_deg_s   = float(rospy.get_param("~hold_rate_deg_s", 2.0))
        self.hold_dwell_s      = float(rospy.get_param("~hold_dwell_s", 0.25))

        # --- load model ---
        self.theta_stat, self.dyn, self.lim = make_theta_stat_from_files(self.model_meta, self.model_pt)
        self.alpha = self.dyn["alpha"]
        self.kS    = self.dyn["kS"]
        self.kD    = self.dyn["kD"]
        self.delay = self.dyn["delay"]
        self.dt    = self.dyn["dt"]
        self.pmax  = self.lim["pmax"]
        self.sigma_ref = self.lim["sigma_ref"]

        # --- internal state (angles in DEG) ---
        self.theta = None        # current θ [deg]
        self.theta_ref = 0.0     # target θ* [deg]
        self._last_theta = None  # for rate [deg/s]
        self._last_t_s   = None

        # command memory (used as "applied pressure", no sensors)
        self.ps_cmd = 0.05
        self.pd_cmd = 0.0
        self.hist_sigma = [self.ps_cmd for _ in range(self.delay)]
        self.hist_delta = [self.pd_cmd for _ in range(self.delay)]

        # HOLD flags
        self._hold_on = False
        self._hold_since = None

        # ROS I/O
        rospy.Subscriber(self.joint_topic, JointState, self._cb_joint)
        rospy.Subscriber(self.target_topic, Float32, self._cb_target)
        self.pub_p12 = rospy.Publisher(self.pub_topic_p12, Vector3, queue_size=10)
        self.pub_mpa = rospy.Publisher(self.pub_topic_mpa_cmd, Vector3, queue_size=10) if self.also_pub_mpa_cmd else None

        rospy.loginfo("NeuralMPC: loaded model delay=%d, alpha=%.3f, kS=%.3f, kD=%.3f, dt=%.4f, pmax=%.2f",
                      self.delay, self.alpha, self.kS, self.kD, self.dt, self.pmax)

    # ---------- ROS callbacks ----------
    def _cb_target(self, msg: Float32):
        val = float(msg.data)
        self.theta_ref = val if self.target_in_deg else math.degrees(val)
        # new target -> release hold
        self._hold_on = False
        self._hold_since = None

    def _cb_joint(self, msg: JointState):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            theta_rad = float(msg.position[i])
            self.theta = math.degrees(theta_rad)  # unify to deg

    # ---------- helpers ----------
    def _theta_rate(self, theta_deg, now_s):
        if self._last_theta is None or self._last_t_s is None:
            self._last_theta, self._last_t_s = theta_deg, now_s
            return 0.0
        dt = max(1e-3, now_s - self._last_t_s)
        rate = (theta_deg - self._last_theta) / dt  # deg/s
        self._last_theta, self._last_t_s = theta_deg, now_s
        return rate

    # ---------- MPC core ----------
    def _simulate_cost(self, u_vec, theta0_deg):
        """
        u_vec: [Σ0,Δ0, Σ1,Δ1, ...] command sequence (not yet delayed)
        Plant sees delayed/previous commands via hist_*.
        Returns per-step cost + terminal (for least_squares).
        """
        H = self.H
        dt = self.dt
        alpha, kS, kD = self.alpha, self.kS, self.kD
        pmax = self.pmax
        sigma_ref = self.sigma_ref

        # plant input includes delay history
        sig_appl = np.r_[self.hist_sigma, u_vec[0::2]]
        del_appl = np.r_[self.hist_delta, u_vec[1::2]]

        theta = theta0_deg
        costs = []
        for k in range(H):
            # effective inputs at plant port
            ps_eff = float(sig_appl[k])
            pd_eff = float(del_appl[k])
            # previous for rates (effective)
            if k == 0:
                ps_prev = self.hist_sigma[-1] if self.delay > 0 else self.ps_cmd
                pd_prev = self.hist_delta[-1] if self.delay > 0 else self.pd_cmd
            else:
                ps_prev = float(sig_appl[k-1])
                pd_prev = float(del_appl[k-1])
            dS_eff = (ps_eff - ps_prev) / dt
            dD_eff = (pd_eff - pd_prev) / dt

            # discrete Euler
            ths = self.theta_stat(ps_eff, pd_eff)  # deg
            theta = theta + dt*(alpha*(ths - theta) + kS*dS_eff + kD*dD_eff)

            # command penalties
            ps_cmd = float(u_vec[2*k+0])
            pd_cmd = float(u_vec[2*k+1])

            # box (soft)
            p1 = 0.5*(ps_cmd + pd_cmd); p2 = 0.5*(ps_cmd - pd_cmd)
            box_violation = 0.0
            if p1 < 0: box_violation += -p1
            if p2 < 0: box_violation += -p2
            if p1 > pmax: box_violation += (p1 - pmax)
            if p2 > pmax: box_violation += (p2 - pmax)

            # rate (cmd)
            if k == 0:
                dS_cmd = ps_cmd - self.ps_cmd
                dD_cmd = pd_cmd - self.pd_cmd
            else:
                dS_cmd = ps_cmd - float(u_vec[2*(k-1)+0])
                dD_cmd = pd_cmd - float(u_vec[2*(k-1)+1])

            # block-hold (discourage change within a block)
            block_pen = 0.0
            if self.block > 1 and (k % self.block != 0):
                ps_prev_cmd = float(u_vec[2*(k-1)+0])
                pd_prev_cmd = float(u_vec[2*(k-1)+1])
                block_pen = (ps_cmd-ps_prev_cmd)**2 + (pd_cmd-pd_prev_cmd)**2

            c = self.w_stage*(theta - self.theta_ref)**2 \
              + self.w_sigma*((ps_cmd - sigma_ref)**2) \
              + self.w_rate*(dS_cmd**2 + dD_cmd**2) \
              + self.w_block*block_pen \
              + 1e6*box_violation \
              + 1e3*max(0.0, abs(dS_cmd) - self.rate_sigma) \
              + 1e3*max(0.0, abs(dD_cmd) - self.rate_delta)
            costs.append(c)

        # terminal cost
        c_term = self.w_term*(theta - self.theta_ref)**2
        return np.array(costs + [c_term], dtype=float)

    def _solve_mpc(self, theta0_deg):
        H = self.H
        # warm start: static inverse at sigma_ref
        u0 = np.zeros(2*H, float)
        for k in range(H):
            ps = self.sigma_ref
            pd = invert_delta_bisect(ps, self.theta_ref, self.theta_stat, self.pmax)
            # project into box
            p1 = 0.5*(ps+pd); p2 = 0.5*(ps-pd)
            p1, p2 = clamp_box(p1, p2, self.pmax)
            u0[2*k+0] = p1 + p2   # Σ
            u0[2*k+1] = p1 - p2   # Δ

        if _HAS_SCIPY:
            res = least_squares(lambda x: self._simulate_cost(x, theta0_deg),
                                u0, method="trf", ftol=1e-8, xtol=1e-8, gtol=1e-8,
                                max_nfev=self.max_nfev)
            u_opt = res.x
        else:
            # simple gradient fallback
            u_opt = u0.copy()
            lr = 0.1
            for _ in range(80):
                base = self._simulate_cost(u_opt, theta0_deg).sum()
                g = np.zeros_like(u_opt)
                h = 1e-3
                for i in range(len(u_opt)):
                    u_opt[i] += h
                    c_plus = self._simulate_cost(u_opt, theta0_deg).sum()
                    u_opt[i] -= 2*h
                    c_minus = self._simulate_cost(u_opt, theta0_deg).sum()
                    u_opt[i] += h
                    g[i] = (c_plus - c_minus)/(2*h)
                u_opt -= lr * g

        return u_opt

    # ---------- main loop ----------
    def spin(self):
        rate_hz = float(rospy.get_param("~rate_hz", 100.0))
        r = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if self.theta is None:
                r.sleep()
                continue

            now_s = rospy.Time.now().to_sec()
            theta_deg = self.theta            # already deg
            theta_star = self.theta_ref       # deg
            rate_deg_s = self._theta_rate(theta_deg, now_s)
            e_deg = theta_star - theta_deg

            # HOLD判定
            if self._hold_on:
                # 目標が変わったら解除される（_cb_targetで）
                pass
            else:
                within = (abs(e_deg) <= self.hold_deadband_deg) and (abs(rate_deg_s) <= self.hold_rate_deg_s)
                if within:
                    if self._hold_since is None:
                        self._hold_since = now_s
                    elif (now_s - self._hold_since) >= self.hold_dwell_s:
                        self._hold_on = True
                        rospy.loginfo_throttle(2.0, "NeuralMPC: HOLD engaged (theta≈target).")

            # MPC 実行（HOLD時もそのままの圧力を維持するイメージなので、MPCは走らせず保持）
            if self._hold_on:
                ps_cmd_next, pd_cmd_next = self.ps_cmd, self.pd_cmd
            else:
                u_opt = self._solve_mpc(theta_deg)
                ps_cmd_next = float(u_opt[0])
                pd_cmd_next = float(u_opt[1])

            # レート制限（コマンド）
            ps_cmd_next = rate_limit(ps_cmd_next, self.ps_cmd, self.rate_sigma, self.dt)
            pd_cmd_next = rate_limit(pd_cmd_next, self.pd_cmd, self.rate_delta, self.dt)

            # ボックス
            p1 = 0.5*(ps_cmd_next + pd_cmd_next)
            p2 = 0.5*(ps_cmd_next - pd_cmd_next)
            p1, p2 = clamp_box(p1, p2, self.pmax)

            # 更新＆遅れライン
            self.ps_cmd = p1 + p2
            self.pd_cmd = p1 - p2
            if self.delay > 0:
                self.hist_sigma.pop(0); self.hist_sigma.append(self.ps_cmd)
                self.hist_delta.pop(0); self.hist_delta.append(self.pd_cmd)

            # Publish
            self.pub_p12.publish(Vector3(x=p1, y=p2, z=0.0))
            if self.pub_mpa is not None:
                raw1 = int(round(p1 * self.raw_counts_per_MPa))
                raw2 = int(round(p2 * self.raw_counts_per_MPa))
                self.pub_mpa.publish(Vector3(x=raw1, y=raw2, z=0))

            r.sleep()


if __name__ == "__main__":
    rospy.init_node("kinikun_neural_mpc")
    try:
        KinikunNeuralMPCNode().spin()
    except rospy.ROSInterruptException:
        pass
