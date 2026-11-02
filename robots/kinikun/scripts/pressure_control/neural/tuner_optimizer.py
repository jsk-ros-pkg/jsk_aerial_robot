#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Online Evaluator & Safe Black-box Optimizer for NN→MPC→PID stack (stabilized).
- Observes topics, scores each "episode", and proposes next params (YAML).
- Does NOT change live params; only writes next-run suggestions.
- Stable tuning: batch update (N=4), step clamp (15%), momentum blend (β=0.35),
  trust-region with bounded sigma, and parameter guardrails.

Deps: numpy, rospy, yaml
"""

import os, sys, time, math, random, copy
import numpy as np
import rospy, yaml
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState

# ---- (optional) deterministic sampling ----
# np.random.seed(42); random.seed(42)

def clip(x, lo, hi): return lo if x < lo else (hi if x > hi else x)

# -------- Config space (bounds & scale) with guardrails --------
SPACE = {
    # MPC weights
    "w_stage":  dict(lo=0.5,  hi=2.5,  scale="log"),
    "w_term":   dict(lo=3.0,  hi=15.0, scale="log"),
    "w_rate":   dict(lo=0.2,  hi=1.2,  scale="log"),
    "w_sigma":  dict(lo=0.02, hi=0.15, scale="log"),
    "wz":       dict(lo=0.05, hi=1.0,  scale="lin"),  # keep > 0

    # planner discretization
    "plan_dt":       dict(lo=0.015, hi=0.035, scale="lin"),
    "plan_horizon":  dict(lo=12,    hi=30,    scale="lin_int"),
    "block":         dict(lo=1,     hi=4,     scale="lin_int"),

    # rate penalties (planning)
    "rate_sigma_plan": dict(lo=0.04, hi=0.15, scale="lin"),
    "rate_delta_plan": dict(lo=0.04, hi=0.15, scale="lin"),

    # command smoothing & runtime caps
    "tau_cmd":          dict(lo=0.06, hi=0.18, scale="lin"),
    "rate_sigma_ctrl":  dict(lo=0.3,  hi=0.8,  scale="lin"),
    "rate_delta_ctrl":  dict(lo=0.3,  hi=0.8,  scale="lin"),  # upper capped

    # PID (Δ)
    "kp_delta":         dict(lo=0.010, hi=0.030, scale="log"),
    "ki_delta":         dict(lo=0.0,   hi=0.010, scale="lin"),
    "kd_delta":         dict(lo=0.002, hi=0.012, scale="lin"),  # keep D>0

    "pid_deadband_deg": dict(lo=0.4,  hi=1.0,  scale="lin"),
    "pid_max_abs":      dict(lo=0.03, hi=0.07, scale="lin"),

    # success judge
    "success_tol_deg":  dict(lo=0.6, hi=1.0, scale="lin"),
    "success_hold_s":   dict(lo=0.5, hi=0.9, scale="lin"),
}

# Fallback defaults (used if ~param absent)
DEFAULT_PARAMS = {
    "w_stage":1.5, "w_term":8.0, "w_rate":0.6, "w_sigma":0.05, "wz":0.2,
    "plan_dt":0.02, "plan_horizon":30, "block":3,
    "rate_sigma_plan":0.08, "rate_delta_plan":0.08,
    "tau_cmd":0.10, "rate_sigma_ctrl":0.5, "rate_delta_ctrl":0.5,
    "kp_delta":0.015, "ki_delta":0.010, "kd_delta":0.004,
    "pid_deadband_deg":0.5, "pid_max_abs":0.04,
    "success_tol_deg":0.7, "success_hold_s":0.6,
}

def sample_around(center, sigma_frac=0.12):
    """Gaussian sampling in trust region around 'center' with bounds + scale."""
    cand = {}
    for k, meta in SPACE.items():
        lo, hi = meta["lo"], meta["hi"]
        base = center.get(k, DEFAULT_PARAMS.get(k, (lo+hi)*0.5))
        if meta["scale"]=="log":
            loL, hiL = math.log(lo+1e-12), math.log(hi)
            baseL = math.log(clip(base, lo, hi))
            sd = (hiL-loL)*sigma_frac
            v = math.exp(clip(np.random.normal(baseL, sd), loL, hiL))
        elif meta["scale"]=="lin_int":
            span = int(round((hi-lo)*sigma_frac))
            span = max(span, 1)
            v = int(round(clip(int(round(base)) + np.random.randint(-span, span+1), lo, hi)))
        else:  # linear float
            span = (hi-lo)*sigma_frac
            v = clip(np.random.normal(base, span), lo, hi)
        cand[k]=v
    return cand

def within_bounds(p):
    for k, meta in SPACE.items():
        if k not in p: return False
        v = p[k]
        if v < meta["lo"] or v > meta["hi"]:
            return False
    return True

def clamp_stepwise(old, new, rel=0.15, abs_int=2):
    """Limit one-shot change: floats ≤±15% (of |old|), ints ≤±2."""
    out = {}
    for k, v_new in new.items():
        v_old = old.get(k, v_new)
        if isinstance(v_old, int):
            dv = int(np.clip(int(round(v_new)) - int(round(v_old)), -abs_int, abs_int))
            out[k] = int(round(v_old)) + dv
        else:
            if abs(v_old) < 1e-9:
                step = rel * max(1e-3, abs(SPACE[k]["hi"]-SPACE[k]["lo"])*0.05)
            else:
                step = rel * abs(v_old)
            v_lim = np.clip(v_new, v_old - step, v_old + step)
            out[k] = float(v_lim)
    return out

def momentum_blend(old, new, beta=0.35):
    """Apply momentum smoothing: new_state = (1-β)*old + β*new."""
    out = {}
    for k in old.keys():
        a, b = old[k], new.get(k, old[k])
        if isinstance(a, int):
            out[k] = int(round((1-beta)*a + beta*b))
        else:
            out[k] = float((1-beta)*a + beta*b)
        # enforce bounds strictly after blending
        lo, hi = SPACE[k]["lo"], SPACE[k]["hi"]
        out[k] = int(round(clip(out[k], lo, hi))) if isinstance(a, int) else float(clip(out[k], lo, hi))
    return out

# -------- Episode scorer --------
class EpisodeScorer:
    def __init__(self, tol_deg, hold_s, sigma_ref=0.55, pmax=1.05):
        self.tol_deg = float(tol_deg)
        self.hold_s  = float(hold_s)
        self.sigma_ref = float(sigma_ref)
        self.pmax = float(pmax)
        self.reset()

    def reset(self):
        self.ts=[]; self.theta=[]; self.theta_ref=[]
        self.ps=[]; self.pd=[]; self.p1=[]; self.p2=[]
        self.start_t = None

    def push(self, t, theta, theta_ref, p1=None, p2=None):
        if self.start_t is None: self.start_t = t
        self.ts.append(t); self.theta.append(theta); self.theta_ref.append(theta_ref)
        if p1 is not None and p2 is not None:
            self.p1.append(p1); self.p2.append(p2)
            self.ps.append(p1+p2); self.pd.append(p1-p2)

    def is_success_now(self):
        if len(self.ts) < 5: return False
        t = np.array(self.ts); th = np.array(self.theta); thr = np.array(self.theta_ref)
        e_deg = np.degrees(thr-th)
        ok = np.abs(e_deg) <= self.tol_deg
        if not ok.any(): return False
        last_t = t[-1]
        win_ok = (t >= last_t - self.hold_s) & ok
        return win_ok.sum() >= 5

    def finalize_and_score(self):
        t = np.array(self.ts); th = np.array(self.theta); thr = np.array(self.theta_ref)
        if len(t)<10:
            return dict(valid=False, reason="too_short")

        e_deg = np.degrees(thr - th)
        rmse = float(np.sqrt(np.mean(e_deg**2)))
        mae  = float(np.mean(np.abs(e_deg)))

        target = thr[-1]
        err_to_target = np.degrees(target - th)
        sign = np.sign(np.degrees(thr[0]-target) + 1e-6)  # initial direction
        overshoot = float(np.max(sign*err_to_target))

        # settle time: first time after which |theta-target|<=tol persistently
        tol = self.tol_deg
        ok = (np.abs(np.degrees(th - target)) <= tol)
        settle_s = float(t[-1]-t[0])
        for i in range(len(t)):
            if np.all(ok[i:]):
                settle_s = float(t[i]-t[0]); break

        # oscillation amplitude in last 1s
        win = t >= (t[-1]-1.0)
        amp = float((np.max(np.degrees(th[win])) - np.min(np.degrees(th[win]))) if win.any() else 0.0)

        effort = sigma_bias = sat_ratio = 0.0
        if len(self.ps)>2:
            ps = np.array(self.ps); pd = np.array(self.pd)
            effort = float(np.sum(np.abs(np.diff(pd))))  # Δ total variation
            sigma_bias = float(np.mean(np.abs(ps - self.sigma_ref)))
            p1 = np.array(self.p1); p2 = np.array(self.p2)
            sat = ((p1<=1e-6)|(p2<=1e-6)|(p1>=self.pmax-1e-6)|(p2>=self.pmax-1e-6))
            sat_ratio = float(np.mean(sat))

        # scalar loss (pre-normalization)
        loss = (1.0*rmse
                + 0.4*max(0.0, overshoot)
                + 0.5*settle_s
                + 0.3*amp
                + 0.2*effort
                + 0.2*sigma_bias
                + 1.0*sat_ratio)

        # step normalization by commanded angle step (deg)
        deg_step = abs(np.degrees(thr[-1] - thr[0]))
        loss /= max(5.0, deg_step)

        return dict(valid=True, rmse=rmse, mae=mae, overshoot=overshoot,
                    settle_s=settle_s, amp=amp, effort=effort, sigma_bias=sigma_bias,
                    sat_ratio=sat_ratio, loss=loss, step_deg=deg_step)

# -------- Optimizer (trust-region random search) --------
class SafeTRS:
    def __init__(self, init_params, tr_sigma=0.12):
        self.best = copy.deepcopy(init_params)
        self.best_loss = None
        self.sigma = tr_sigma
        self.sigma_min = 0.08
        self.sigma_max = 0.25

    def ask(self, n=1):
        return [ sample_around(self.best, sigma_frac=self.sigma) for _ in range(n) ]

    def tell(self, cand_list, loss_list):
        # keep best feasible
        for c, L in zip(cand_list, loss_list):
            if not within_bounds(c): continue
            if (self.best_loss is None) or (L < self.best_loss):
                self.best = copy.deepcopy(c); self.best_loss = float(L)
        # success rate to adapt sigma
        improves = [ (self.best_loss is not None and L <= self.best_loss) for L in loss_list ]
        rate = sum(improves)/float(len(loss_list)) if loss_list else 0.0
        if rate < 0.2:
            self.sigma = max(self.sigma_min, self.sigma*0.7)
        elif rate > 0.6:
            self.sigma = min(self.sigma_max, self.sigma*1.15)

# -------- ROS Tuner Node --------
class TunerNode:
    def __init__(self):
        # I/O paths & constants
        self.out_yaml = rospy.get_param("~out_param_yaml", "/tmp/next_params.yaml")
        self.print_diff = bool(rospy.get_param("~print_diff", True))
        self.sigma_ref = float(rospy.get_param("~sigma_ref", 0.55))
        self.pmax = float(rospy.get_param("~pmax", 1.05))

        # Baseline current params (read from ~ namespace, typically loaded via rosparam)
        self.current = {}
        for k, v in DEFAULT_PARAMS.items():
            self.current[k] = type(v)(rospy.get_param("~"+k, v))

        # ensure current within bounds (clip)
        for k, meta in SPACE.items():
            if k in self.current:
                v = self.current[k]
                lo, hi = meta["lo"], meta["hi"]
                self.current[k] = int(round(clip(v, lo, hi))) if isinstance(v, int) else float(clip(v, lo, hi))

        # optimizer
        self.opt = SafeTRS(self.current, tr_sigma=0.12)

        # episode state
        self.active = False
        self.scorer = EpisodeScorer(tol_deg=self.current["success_tol_deg"],
                                    hold_s=self.current["success_hold_s"],
                                    sigma_ref=self.sigma_ref, pmax=self.pmax)
        self.theta = None
        self.target_angle_deg = 0.0
        self.p1p2 = (None, None)

        # batch buffers
        self.UPDATE_BATCH = 4
        self.pending_cands = []
        self.cand_losses = []
        self.current_cand = None

        # ROS I/O
        # (Remap these topics at launch if needed)
        rospy.Subscriber("/target_angle", Float32, self.cb_target)
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.cb_joint)
        rospy.Subscriber("/mpa_cmd", Vector3, self.cb_p12)

        rospy.loginfo("[tuner] ready. Logging episodes (batch=%d) and proposing next params to %s",
                      self.UPDATE_BATCH, self.out_yaml)

    def cb_target(self, msg):
        self.target_angle_deg = float(msg.data)
        self.active = True
        # refresh scorer with latest success criteria (may be tuned)
        self.scorer = EpisodeScorer(tol_deg=self.current["success_tol_deg"],
                                    hold_s=self.current["success_hold_s"],
                                    sigma_ref=self.sigma_ref, pmax=self.pmax)
        rospy.loginfo("[tuner] episode start: target=%.2f deg", self.target_angle_deg)

    def cb_joint(self, msg):
        if "arm1_joint" in msg.name:
            i = msg.name.index("arm1_joint")
            self.theta = float(msg.position[i])

    def cb_p12(self, msg):
        self.p1p2 = (float(msg.x)*0.9/4096, float(msg.y)*0.9/4096)

    def spin(self):
        r = rospy.Rate(100)
        last_success_flag = False

        while not rospy.is_shutdown():
            if self.active and (self.theta is not None):
                p1, p2 = self.p1p2
                self.scorer.push(time.time(), self.theta, math.radians(self.target_angle_deg), p1=p1, p2=p2)

                succ = self.scorer.is_success_now()
                if succ and not last_success_flag:
                    res = self.scorer.finalize_and_score()
                    if res["valid"]:
                        rospy.loginfo("[tuner] result: loss=%.3f RMSE=%.3fdeg overshoot=%.2fdeg settle=%.2fs amp=%.2fdeg effort=%.3f sigma_bias=%.3f sat=%.2f%% (step=%.1fdeg)",
                                      res["loss"], res["rmse"], res["overshoot"], res["settle_s"],
                                      res["amp"], res["effort"], res["sigma_bias"], 100*res["sat_ratio"], res["step_deg"])

                        # add to batch
                        if self.current_cand is None:
                            self.current_cand = copy.deepcopy(self.current)  # baseline on first episode
                        self.pending_cands.append(self.current_cand)
                        self.cand_losses.append(res["loss"])

                        if len(self.pending_cands) >= self.UPDATE_BATCH:
                            # 1) optimizer update with the batch
                            self.opt.tell(self.pending_cands, self.cand_losses)

                            # 2) guard step & momentum blend from current → best
                            best_raw = self.opt.best
                            guarded = clamp_stepwise(self.current, best_raw, rel=0.15, abs_int=2)
                            blended = momentum_blend(self.current, guarded, beta=0.35)

                            # 3) write YAML with blended (smoothed) params
                            self._emit_yaml(blended)
                            if self.print_diff: self._print_diff(self.current, blended)

                            # 4) update baseline and prepare next candidate
                            self.current = copy.deepcopy(blended)
                            self.pending_cands.clear()
                            self.cand_losses.clear()
                            self.current_cand = self.opt.ask(n=1)[0]
                            rospy.loginfo("[tuner] next candidate sampled (TR σ=%.2f)", self.opt.sigma)
                        else:
                            # ask another candidate around current best (for the next episode)
                            self.current_cand = self.opt.ask(n=1)[0]
                            rospy.loginfo("[tuner] batch %d/%d collected, sampling next candidate.", len(self.pending_cands), self.UPDATE_BATCH)
                    else:
                        rospy.logwarn("[tuner] episode invalid (%s)", res.get("reason","?"))
                    self.active = False
                last_success_flag = succ
            r.sleep()

    def _emit_yaml(self, params):
        os.makedirs(os.path.dirname(self.out_yaml), exist_ok=True)
        with open(self.out_yaml, "w") as f:
            yaml.safe_dump(params, f, sort_keys=True)
        rospy.loginfo("[tuner] wrote next params → %s", self.out_yaml)

    def _print_diff(self, base, new):
        rospy.loginfo("---- suggested param changes ----")
        for k in sorted(new.keys()):
            a, b = base[k], new[k]
            if isinstance(a, int):
                if a != b:
                    rospy.loginfo("%-18s: %s -> %s", k, str(a), str(b))
            else:
                if abs(b-a) >= 1e-6:
                    rospy.loginfo("%-18s: %8.4f  -> %8.4f  (Δ=%+.4f)", k, a, b, b-a)
        rospy.loginfo("--------------------------------")

if __name__ == "__main__":
    rospy.init_node("kinikun_param_tuner")
    try:
        TunerNode().spin()
    except rospy.ROSInterruptException:
        pass
