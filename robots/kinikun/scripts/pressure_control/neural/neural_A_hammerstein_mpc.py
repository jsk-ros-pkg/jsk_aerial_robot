#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS node: Receding-Horizon MPC for pressure reference (p1, p2) using Neural Hammerstein (+delay).

Subscribes:
  - ~joint_topic (sensor_msgs/JointState): current angle θ (rad or deg selectable)
  - ~target_topic (std_msgs/Float32): desired angle θ* (rad or deg selectable)
  - (optional) ~p12_meas_topic (geometry_msgs/Vector3): measured p1,p2 [MPa]  (use_measured_pressures=True)

Publishes:
  - ~pub_topic_p12 (geometry_msgs/Vector3): commanded pressures p1_ref, p2_ref [MPa]
  - (optional) ~pub_topic_mpa_cmd (geometry_msgs/Vector3): raw counts for your IO (if also_pub_mpa_cmd=True)

Params (main):
  - ~model_pt (str): path to *_nh.pt
  - ~model_meta (str): path to *_nh_meta.npz
  - ~joint_topic (str), ~joint_name (str), ~target_topic (str)
  - ~theta_in_deg (bool): target and joint are degrees (true) or radians (false)
  - ~rate_hz (float): control loop rate (default 100 Hz)

MPC params:
  - ~H (int): horizon steps (default 10)
  - ~w_stage, ~w_term, ~wz, ~w_sigma, ~w_rate, ~w_block
  - ~block (int): move-blocking length (1=none)
  - ~rate_sigma, ~rate_delta (MPa/step): soft rate limits (on command)
  - ~max_nfev (int): optimizer budget

Other:
  - ~use_measured_pressures (bool): if true, use p1/p2 measurement for delay-line plant input. Else use previous commands.
  - ~p12_meas_topic (str): measurement topic when enabled.
  - ~also_pub_mpa_cmd (bool), ~pub_topic_mpa_cmd (str), ~raw_counts_per_MPa (float)

Usage:
rosrun kinikun neural_hammerstein_mpc.py   _model_pt:="$HOME/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh.pt"   _model_meta:="$HOME/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/model_k4b_nh_meta.npz"   _joint_topic:=/kinikun1/joint_states   _joint_name:=arm1_joint   _target_topic:=/target_angle   _theta_in_deg:=true   _pub_topic_p12:=/p1p2_cmd   _rate_hz:=100   _H:=10 _w_stage:=1.0 _w_term:=6.0   _wz:=0.2 _w_sigma:=0.02 _w_rate:=0.4   _w_block:=8.0 _block:=2   _rate_sigma:=0.12 _rate_delta:=0.12   _max_nfev:=600

"""

import rospy, math, importlib
from collections import deque
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import torch, torch.nn as nn

# --------- NN (must match training) ----------
class MonoDeltaNN(nn.Module):
    def __init__(self, M, hidden, c_grid):
        super().__init__()
        self.M = M
        self.c = nn.Parameter(torch.tensor(c_grid, dtype=torch.float32), requires_grad=False)
        self.enc = nn.Sequential(
            nn.Linear(1, hidden), nn.ELU(),
            nn.Linear(hidden, hidden), nn.ELU()
        )
        self.head_A = nn.Linear(hidden, 1)
        self.head_w = nn.Linear(hidden, M)
        self.head_s = nn.Linear(hidden, M)
        nn.init.zeros_(self.head_w.weight); nn.init.zeros_(self.head_w.bias)
        nn.init.zeros_(self.head_s.weight); nn.init.constant_(self.head_s.bias, 1.0)

    def forward(self, Sigma_hat, Delta_hat):
        h = self.enc(Sigma_hat)
        A = self.head_A(h)
        w = torch.nn.functional.softplus(self.head_w(h)) + 1e-6
        s = torch.nn.functional.softplus(self.head_s(h)) + 1e-6
        d = Delta_hat - self.c.view(1,-1)
        bank = torch.tanh(s * d)
        return A + (w*bank).sum(dim=1, keepdim=True)

# ---------- utilities ----------
def clamp_box(p1, p2, pmax):
    return np.clip(p1,0,pmax), np.clip(p2,0,pmax)

def build_theta_stat(meta, state_dict):
    M = int(meta["M"]); hidden = int(meta["hidden"])
    c_grid = meta["c_grid"].tolist() if hasattr(meta["c_grid"], "tolist") else list(meta["c_grid"])
    muS, sdS = float(meta["muS"]), float(meta["sdS"])
    muD, sdD = float(meta["muD"]), float(meta["sdD"])

    model = MonoDeltaNN(M, hidden, c_grid)
    model.load_state_dict(state_dict)
    model.eval()

    @torch.no_grad()
    def f(ps, pd):
        S = torch.tensor([(ps-muS)/sdS], dtype=torch.float32).view(1,1)
        D = torch.tensor([(pd-muD)/sdD], dtype=torch.float32).view(1,1)
        return float(model(S,D).item())
    return f

# ---------- MPC core (single-shot; returns first step) ----------
def mpc_first_step(theta_stat, meta, theta_star, theta0,
                   hist_sigma, hist_delta,
                   H=10,
                   w_stage=1.0, w_term=5.0, w_z=0.2, w_sigma=0.02, w_rate=0.3, w_block=0.0,
                   block=1,
                   rate_sigma=0.12, rate_delta=0.12,
                   max_nfev=600):
    alpha = float(meta["alpha"]); kS = float(meta["kSigma"]); kD = float(meta["kDelta"])
    dt = float(meta["dt"]); delay = int(meta["delay"])
    pmax = float(meta["pmax"]); sigma_ref = float(meta["sigma_ref"])
    zc = meta["z_coef"] if "z_coef" in meta.files else np.array([])

    def z_pred(ps, pd, zc):
        if zc is None or zc.size < 4: return 0.0
        return float(zc[0] + zc[1]*ps + zc[2]*pd + zc[3]*ps*pd)

    # warm start: static inverse at theta*
    def invert_delta(ps, theta_tgt):
        # robust bisection in Δ (MPa)
        lo, hi = -pmax, pmax
        f_lo = theta_stat(ps, lo) - theta_tgt
        f_hi = theta_stat(ps, hi) - theta_tgt
        if f_lo*f_hi > 0:  # no sign change → return closer end
            return lo if abs(f_lo) < abs(f_hi) else hi
        for _ in range(50):
            mid = 0.5*(lo+hi)
            f_mid = theta_stat(ps, mid) - theta_tgt
            if abs(hi-lo) < 1e-4 or abs(f_mid) < 1e-4:
                return mid
            if f_lo*f_mid <= 0:
                hi, f_hi = mid, f_mid
            else:
                lo, f_lo = mid, f_mid
        return 0.5*(lo+hi)

    u0 = np.zeros(2*H, float)
    for k in range(H):
        ps = sigma_ref
        pd = invert_delta(ps, theta_star)
        p1 = 0.5*(ps+pd); p2=0.5*(ps-pd); p1,p2 = clamp_box(p1,p2,pmax)
        u0[2*k+0] = p1+p2
        u0[2*k+1] = p1-p2

    rateS = float(rate_sigma); rateD = float(rate_delta)
    block = max(1, int(block))

    # pre-append histories as numpy
    hist_sigma = np.asarray(hist_sigma, float)
    hist_delta = np.asarray(hist_delta, float)

    def simulate(u_vec):
        # plant-applied (with delay handled by prepending history)
        sig_appl = np.r_[hist_sigma, u_vec[0::2]]
        del_appl = np.r_[hist_delta, u_vec[1::2]]

        theta = theta0
        cost_terms = []
        for k in range(H):
            ps_eff = sig_appl[k]
            pd_eff = del_appl[k]
            if k == 0:
                ps_prev = hist_sigma[-1] if len(hist_sigma)>0 else sig_appl[0]
                pd_prev = hist_delta[-1] if len(hist_delta)>0 else del_appl[0]
            else:
                ps_prev = sig_appl[k-1]
                pd_prev = del_appl[k-1]
            dS_eff = (ps_eff-ps_prev)/dt
            dD_eff = (pd_eff-pd_prev)/dt

            ths = theta_stat(ps_eff, pd_eff)
            theta = theta + dt*( alpha*(ths - theta) + kS*dS_eff + kD*dD_eff )

            # command penalties
            ps_cmd = u_vec[2*k+0]; pd_cmd = u_vec[2*k+1]
            p1 = 0.5*(ps_cmd+pd_cmd); p2 = 0.5*(ps_cmd-pd_cmd)
            box_violation = 0.0
            if p1<0: box_violation += -p1
            if p2<0: box_violation += -p2
            if p1>pmax: box_violation += (p1-pmax)
            if p2>pmax: box_violation += (p2-pmax)

            if k==0:
                dS_cmd = ps_cmd - (hist_sigma[-1] if len(hist_sigma)>0 else ps_cmd)
                dD_cmd = pd_cmd - (hist_delta[-1] if len(hist_delta)>0 else pd_cmd)
            else:
                dS_cmd = ps_cmd - u_vec[2*(k-1)+0]
                dD_cmd = pd_cmd - u_vec[2*(k-1)+1]

            rate_pen = max(0.0, abs(dS_cmd)-rateS) + max(0.0, abs(dD_cmd)-rateD)

            block_pen = 0.0
            if block>1 and (k%block!=0):
                ps_prev_cmd = u_vec[2*(k-1)+0]; pd_prev_cmd = u_vec[2*(k-1)+1]
                block_pen = (ps_cmd-ps_prev_cmd)**2 + (pd_cmd-pd_prev_cmd)**2

            c = (w_stage*(theta-theta_star)**2
                 + w_z*(z_pred(ps_cmd,pd_cmd,zc)**2)
                 + w_sigma*((ps_cmd - sigma_ref)**2)
                 + w_rate*(dS_cmd**2 + dD_cmd**2)
                 + w_block*block_pen
                 + 1e6*box_violation + 1e3*rate_pen)
            cost_terms.append(c)

        c_term = w_term*(theta-theta_star)**2
        return np.array(cost_terms + [c_term], float)

    # solve
    try:
        from scipy.optimize import least_squares
        res = least_squares(lambda x: simulate(x), u0, method="trf",
                            ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=int(max_nfev))
        u_opt = res.x
    except Exception:
        # coordinate-descent fallback
        u_opt = u0.copy()
        for _ in range(80):
            base = simulate(u_opt).sum()
            for i in range(len(u_opt)):
                h = 1e-3
                u_opt[i]+=h; c_plus = simulate(u_opt).sum()
                u_opt[i]-=2*h; c_minus= simulate(u_opt).sum()
                u_opt[i]+=h
                g=(c_plus-c_minus)/(2*h)
                u_opt[i]-=0.1*g

    ps0 = float(u_opt[0]); pd0 = float(u_opt[1])
    p1 = 0.5*(ps0+pd0); p2 = 0.5*(ps0-pd0)
    return clamp_box(p1,p2,pmax)

# ---------- ROS Node ----------
class MPCNode:
    def __init__(self):
        # I/O topics & params
        self.joint_topic  = rospy.get_param("~joint_topic", "/kinikun1/joint_states")
        self.joint_name   = rospy.get_param("~joint_name",  "arm1_joint")
        self.target_topic = rospy.get_param("~target_topic","/target_angle")
        self.theta_in_deg = rospy.get_param("~theta_in_deg", False)

        self.pub_topic_p12 = rospy.get_param("~pub_topic_p12", "/p1p2_cmd")
        self.also_pub_mpa_cmd = rospy.get_param("~also_pub_mpa_cmd", False)
        self.pub_topic_mpa_cmd = rospy.get_param("~pub_topic_mpa_cmd", "/mpa_cmd")
        self.raw_counts_per_MPa = rospy.get_param("~raw_counts_per_MPa", 4096.0/0.9)

        # model files
        self.model_pt   = rospy.get_param("~model_pt",   "out/model_k4b/model_k4b_nh.pt")
        self.model_meta = rospy.get_param("~model_meta", "out/model_k4b/model_k4b_nh_meta.npz")

        # (optional) measured pressures for delay-line
        self.use_meas_press = rospy.get_param("~use_measured_pressures", False)
        self.p12_meas_topic = rospy.get_param("~p12_meas_topic", "/p1p2_meas")

        # MPC settings
        self.H          = int(rospy.get_param("~H", 10))
        self.w_stage    = float(rospy.get_param("~w_stage", 1.0))
        self.w_term     = float(rospy.get_param("~w_term", 5.0))
        self.wz         = float(rospy.get_param("~wz", 0.2))
        self.w_sigma    = float(rospy.get_param("~w_sigma", 0.02))
        self.w_rate     = float(rospy.get_param("~w_rate", 0.3))
        self.w_block    = float(rospy.get_param("~w_block", 10.0))
        self.block      = int(rospy.get_param("~block", 2))
        self.rate_sigma = float(rospy.get_param("~rate_sigma", 0.12))
        self.rate_delta = float(rospy.get_param("~rate_delta", 0.12))
        self.max_nfev   = int(rospy.get_param("~max_nfev", 600))

        # load model
        meta = np.load(self.model_meta, allow_pickle=True)
        state = torch.load(self.model_pt, map_location="cpu")
        self.theta_stat = build_theta_stat(meta, state)

        # meta essentials
        self.delay = int(meta["delay"])
        self.dt    = float(meta["dt"])
        self.pmax  = float(meta["pmax"])
        self.sigma_ref = float(meta["sigma_ref"])

        # states
        self.theta = None
        self.theta_ref = 0.0
        self.p1_meas = None; self.p2_meas = None
        self.p1_cmd  = 0.5*self.sigma_ref
        self.p2_cmd  = 0.5*self.sigma_ref

        # history for delay
        self.hist_sigma = deque([self.sigma_ref]*self.delay, maxlen=self.delay)
        self.hist_delta = deque([0.0]*self.delay,          maxlen=self.delay)

        # ROS IO
        rospy.Subscriber(self.joint_topic, JointState, self.cb_joint)
        rospy.Subscriber(self.target_topic, Float32, self.cb_target)
        if self.use_meas_press:
            rospy.Subscriber(self.p12_meas_topic, Vector3, self.cb_p12_meas)
        self.pub_p12 = rospy.Publisher(self.pub_topic_p12, Vector3, queue_size=10)
        self.pub_mpa = rospy.Publisher(self.pub_topic_mpa_cmd, Vector3, queue_size=10) if self.also_pub_mpa_cmd else None

        self.rate_hz = float(rospy.get_param("~rate_hz", 100.0))
        self.last_t  = rospy.Time.now().to_sec()

        rospy.loginfo("neural_hammerstein_mpc_node started. delay=%d, dt=%.4f s, pmax=%.3f MPa", self.delay, self.dt, self.pmax)

    # --- Callbacks ---
    def cb_target(self, msg):
        val = float(msg.data)
        self.theta_ref = math.radians(val) if self.theta_in_deg else val

    def cb_joint(self, msg):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            self.theta = float(msg.position[i])
            if self.theta_in_deg:
                self.theta = math.degrees(self.theta)  # unify to 'deg' if your targets are deg
                # NOTE: If your training used 'deg', keep this; else remove this line and pass radians consistently.

    def cb_p12_meas(self, msg):
        self.p1_meas = float(msg.x); self.p2_meas = float(msg.y)

    # --- main loop ---
    def spin(self):
        r = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            dt_loop = max(1e-3, now - self.last_t); self.last_t = now

            if self.theta is None:
                r.sleep(); continue

            theta0 = self.theta
            theta_star = self.theta_ref

            # choose history source (measured or last commanded)
            if self.use_meas_press and (self.p1_meas is not None) and (self.p2_meas is not None):
                sig_hist_val = self.p1_meas + self.p2_meas
                del_hist_val = self.p1_meas - self.p2_meas
            else:
                sig_hist_val = self.p1_cmd + self.p2_cmd
                del_hist_val = self.p1_cmd - self.p2_cmd

            # keep delay-line updated even when MPC fails
            self.hist_sigma.append(sig_hist_val)
            self.hist_delta.append(del_hist_val)

            try:
                p1_ref, p2_ref = mpc_first_step(
                    self.theta_stat, np.load(self.model_meta, allow_pickle=True),
                    theta_star=theta_star, theta0=theta0,
                    hist_sigma=list(self.hist_sigma), hist_delta=list(self.hist_delta),
                    H=self.H, w_stage=self.w_stage, w_term=self.w_term,
                    w_z=self.wz, w_sigma=self.w_sigma, w_rate=self.w_rate, w_block=self.w_block,
                    block=self.block, rate_sigma=self.rate_sigma, rate_delta=self.rate_delta,
                    max_nfev=self.max_nfev
                )
            except Exception as e:
                rospy.logwarn_throttle(1.0, "MPC solve failed: %s (using last command)", str(e))
                p1_ref, p2_ref = self.p1_cmd, self.p2_cmd

            # update command & publish
            self.p1_cmd, self.p2_cmd = p1_ref, p2_ref
            self.pub_p12.publish(Vector3(x=self.p1_cmd, y=self.p2_cmd, z=0.0))

            if self.pub_mpa is not None:
                raw1 = int(round(self.p1_cmd * self.raw_counts_per_MPa))
                raw2 = int(round(self.p2_cmd * self.raw_counts_per_MPa))
                self.pub_mpa.publish(Vector3(x=raw1, y=raw2, z=0))

            r.sleep()

# ---------- main ----------
if __name__ == "__main__":
    rospy.init_node("neural_hammerstein_mpc_node")
    MPCNode().spin()
