#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#NN→MPC→PID 成功判定付き
import os, math, time, collections
import numpy as np
import rospy
import torch, torch.nn as nn
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

def clip(x, lo, hi): return max(lo, min(hi, x))
def rate_limit(target, current, rate_max, dt):
    return current + clip(target - current, -rate_max*dt, rate_max*dt)

# ---------- Neural static kernel ----------
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

def make_theta_stat(meta, state_dict):
    M = int(meta["M"]); hidden = int(meta["hidden"])
    c_grid = meta["c_grid"].tolist() if hasattr(meta["c_grid"], "tolist") else list(meta["c_grid"])
    muS, sdS = float(meta["muS"]), float(meta["sdS"])
    muD, sdD = float(meta["muD"]), float(meta["sdD"])
    model = MonoDeltaNN(M, hidden, c_grid); model.load_state_dict(state_dict); model.eval()
    @torch.no_grad()
    def f(ps, pd):
        S = torch.tensor([(ps-muS)/sdS], dtype=torch.float32).view(1,1)
        D = torch.tensor([(pd-muD)/sdD], dtype=torch.float32).view(1,1)
        return float(model(S,D).item())
    return f

# ---------- simple predictive planner ----------
class Planner(object):
    def __init__(self, theta_stat, alpha, kS, kD, dt_model, delay, pmax, sigma_ref):
        self.f = theta_stat
        self.alpha, self.kS, self.kD = float(alpha), float(kS), float(kD)
        self.dt_model = float(dt_model); self.delay = int(delay)
        self.pmax = float(pmax); self.sigma_ref = float(sigma_ref)

    def clamp_box(self, ps, pd):
        p1 = 0.5*(ps+pd); p2 = 0.5*(ps-pd)
        p1 = clip(p1, 0.0, self.pmax); p2 = clip(p2, 0.0, self.pmax)
        return p1+p2, p1-p2

    def simulate_cost_terms(self, u_vec, H, dt, theta0, ps0, pd0,
                            theta_star, w_stage, w_term, wz, w_sigma, w_rate, w_block,
                            rate_sigma, rate_delta, block, zc=None):
        def z_pred(ps, pd):
            if zc is None or zc.size < 4: return 0.0
            return float(zc[0] + zc[1]*ps + zc[2]*pd + zc[3]*ps*pd)

        theta = theta0; cost = []
        for k in range(H):
            ps_cmd = u_vec[2*k+0]; pd_cmd = u_vec[2*k+1]
            if k==0: dS = ps_cmd-ps0; dD = pd_cmd-pd0
            else:    dS = ps_cmd-u_vec[2*(k-1)+0]; dD = pd_cmd-u_vec[2*(k-1)+1]
            rate_pen = max(0.0, abs(dS)-rate_sigma) + max(0.0, abs(dD)-rate_delta)

            ps_eff, pd_eff = self.clamp_box(ps_cmd, pd_cmd)
            p1 = 0.5*(ps_cmd+pd_cmd); p2 = 0.5*(ps_cmd-pd_cmd)
            box_viol = 0.0
            if p1<0: box_viol += -p1
            if p2<0: box_viol += -p2
            if p1>self.pmax: box_viol += (p1-self.pmax)
            if p2>self.pmax: box_viol += (p2-self.pmax)

            block_pen = 0.0
            if block>1 and (k%block!=0):
                ps_prev = u_vec[2*(k-1)+0]; pd_prev = u_vec[2*(k-1)+1]
                block_pen = (ps_cmd-ps_prev)**2 + (pd_cmd-pd_prev)**2

            ths = self.f(ps_eff, pd_eff)
            if k==0:
                dS_eff = (ps_eff-ps0)/dt; dD_eff = (pd_eff-pd0)/dt
            else:
                ps_prev_eff, pd_prev_eff = self.clamp_box(u_vec[2*(k-1)+0], u_vec[2*(k-1)+1])
                dS_eff = (ps_eff-ps_prev_eff)/dt; dD_eff = (pd_eff-pd_prev_eff)/dt
            theta = theta + dt*( self.alpha*(ths - theta) + self.kS*dS_eff + self.kD*dD_eff )

            c = (w_stage*(theta-theta_star)**2 + wz*(z_pred(ps_cmd, pd_cmd)**2)
                 + w_sigma*((ps_cmd-self.sigma_ref)**2) + w_rate*(dS**2 + dD**2)
                 + w_block*block_pen + 1e6*box_viol + 1e3*rate_pen)
            cost.append(c)
        cost.append(w_term*(theta-theta_star)**2)
        return np.array(cost, float)

    def plan(self, theta0, ps0, pd0, theta_star, H, dt,
             w_stage, w_term, wz, w_sigma, w_rate, w_block,
             rate_sigma, rate_delta, block, zc=None, max_nfev=500):
        u = np.zeros(2*H, float)
        for k in range(H):
            ps = self.sigma_ref; lo, hi = -self.pmax, self.pmax
            for _ in range(40):
                mid = 0.5*(lo+hi)
                val = self.f(ps, mid) - theta_star
                vlo = self.f(ps, lo) - theta_star
                if np.sign(vlo)*np.sign(val) <= 0: hi = mid
                else: lo = mid
            pd = 0.5*(lo+hi)
            ps_cmd, pd_cmd = self.clamp_box(ps, pd)
            u[2*k+0], u[2*k+1] = ps_cmd, pd_cmd

        def obj(x):
            return self.simulate_cost_terms(x, H, dt, theta0, ps0, pd0, theta_star,
                                            w_stage, w_term, wz, w_sigma, w_rate, w_block,
                                            rate_sigma, rate_delta, block, zc)

        try:
            from scipy.optimize import least_squares
            res = least_squares(lambda x: obj(x), u, method="trf",
                                ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=int(max_nfev))
            u_opt = res.x
        except Exception:
            u_opt = u.copy()
            for _ in range(50):
                for i in range(len(u_opt)):
                    base = obj(u_opt).sum(); h=1e-3
                    u_opt[i]+=h; c_plus=obj(u_opt).sum()
                    u_opt[i]-=2*h; c_minus=obj(u_opt).sum()
                    u_opt[i]+=h; g=(c_plus-c_minus)/(2*h); u_opt[i]-=0.05*g

        return u_opt[0::2].copy(), u_opt[1::2].copy()

# ---------- ROS Node with watchdog ----------
class MPCOnceThenPIDNode(object):
    def __init__(self):
        # model paths
        self.model_pt   = rospy.get_param("~model_pt")
        self.model_meta = rospy.get_param("~model_meta")
        if not (self.model_pt and self.model_meta):
            raise RuntimeError("~model_pt と ~model_meta を指定してください。")

        # rates & topics
        self.ctrl_rate_hz  = float(rospy.get_param("~ctrl_rate_hz", 100.0))
        self.target_in_deg = bool(rospy.get_param("~target_in_deg", True))
        self.joint_topic = rospy.get_param("~joint_topic", "/kinikun1/joint_states")
        self.joint_name  = rospy.get_param("~joint_name",  "arm1_joint")
        self.target_topic= rospy.get_param("~target_topic","/target_angle")
        self.pub_topic_p12 = rospy.get_param("~pub_topic_p12", "/p1p2_cmd")
        self.also_pub_counts = bool(rospy.get_param("~also_pub_counts", True))
        self.raw_counts_per_MPa = float(rospy.get_param("~raw_counts_per_MPa", 4096.0/0.9))
        self.pub_topic_counts = rospy.get_param("~pub_topic_counts", "/mpa_cmd")

        # planning params
        self.plan_dt      = float(rospy.get_param("~plan_dt", 0.02))
        self.plan_horizon = int(rospy.get_param("~plan_horizon", 30))
        self.block        = int(rospy.get_param("~block", 3))
        self.w_stage      = float(rospy.get_param("~w_stage", 1.5))
        self.w_term       = float(rospy.get_param("~w_term", 8.0))
        self.w_rate       = float(rospy.get_param("~w_rate", 0.6))
        self.w_sigma      = float(rospy.get_param("~w_sigma", 0.05))
        self.wz           = float(rospy.get_param("~wz", 0.2))
        self.rate_sigma_plan = float(rospy.get_param("~rate_sigma_plan", 0.08))
        self.rate_delta_plan = float(rospy.get_param("~rate_delta_plan", 0.08))

        # command smoothing
        self.tau_cmd         = float(rospy.get_param("~tau_cmd", 0.10))
        self.rate_sigma_ctrl = float(rospy.get_param("~rate_sigma_ctrl", 0.5))
        self.rate_delta_ctrl = float(rospy.get_param("~rate_delta_ctrl", 0.5))

        # PID (Δのみ)
        self.use_pid      = bool(rospy.get_param("~use_pid", True))
        self.kp_delta     = float(rospy.get_param("~kp_delta", 0.015))
        self.ki_delta     = float(rospy.get_param("~ki_delta", 0.010))
        self.kd_delta     = float(rospy.get_param("~kd_delta", 0.004))
        self.pid_max_abs  = float(rospy.get_param("~pid_max_abs", 0.04))
        self.pid_deadband_deg = float(rospy.get_param("~pid_deadband_deg", 0.5))
        self.pid_freeze_on_sat = bool(rospy.get_param("~pid_freeze_on_sat", True))
        self.tau_theta    = float(rospy.get_param("~tau_theta", 0.08))  # 角度LPF

        # watchdog / convergence
        self.success_tol_deg       = float(rospy.get_param("~success_tol_deg", 0.7))
        self.success_hold_s        = float(rospy.get_param("~success_hold_s", 0.6))
        self.stagnation_window_s   = float(rospy.get_param("~stagnation_window_s", 1.2))
        self.stagnation_slope_thresh_deg_per_s = float(rospy.get_param("~stagnation_slope_thresh_deg_per_s", -0.02))
        self.stagnation_min_err_deg= float(rospy.get_param("~stagnation_min_err_deg", 1.0))
        self.oscill_amp_window_s   = float(rospy.get_param("~oscill_amp_window_s", 1.0))
        self.oscill_amp_thresh_deg = float(rospy.get_param("~oscill_amp_thresh_deg", 1.2))
        self.replan_cooldown_s     = float(rospy.get_param("~replan_cooldown_s", 0.8))

        # load model
        meta = np.load(self.model_meta, allow_pickle=True)
        state = torch.load(self.model_pt, map_location="cpu")
        self.theta_stat = make_theta_stat(meta, state)
        self.alpha = float(meta["alpha"]); self.kS = float(meta["kSigma"]); self.kD = float(meta["kDelta"])
        self.dt_model = float(meta["dt"]); self.delay = int(meta["delay"])
        self.pmax = float(meta["pmax"]) if "pmax" in meta.files else 1.05
        self.sigma_ref = float(meta["sigma_ref"]) if "sigma_ref" in meta.files else 0.55
        self.z_coef = meta["z_coef"] if "z_coef" in meta.files else np.array([])
        self.planner = Planner(self.theta_stat, self.alpha, self.kS, self.kD,
                               self.dt_model, self.delay, self.pmax, self.sigma_ref)

        # state
        self.theta = None; self.theta_f = None; self.theta_prev = None
        self.theta_ref = 0.0  # [rad]
        # 初期圧 p1=p2=0.2 → Σ=0.4, Δ=0
        self.ps_cmd = 0.40; self.pd_cmd = 0.00
        self.pid_out = 0.0; self.int_e = 0.0

        self.plan_ps=[]; self.plan_pd=[]; self.plan_idx=0; self.plan_time_accum=0.0
        self.phase = "HOLD"  # HOLD / PLAN_EXEC / PID_HOLD / SUCCESS
        self.success_timer = 0.0
        self.last_replan_time = -1e9

        # error buffers for watchdog
        self.err_abs_buf = collections.deque()  # (t, |e_deg|)
        self.err_abs_buf_max = 10000

        # ROS I/O
        rospy.Subscriber(self.joint_topic, JointState, self.cb_joint)
        rospy.Subscriber(self.target_topic, Float32, self.cb_target)
        self.pub_p12 = rospy.Publisher(self.pub_topic_p12, Vector3, queue_size=10)
        self.pub_counts = rospy.Publisher(self.pub_topic_counts, Vector3, queue_size=10) if self.also_pub_counts else None
        rospy.loginfo("[node] model loaded. alpha=%.3f kS=%.3f kD=%.3f delay=%d pmax=%.2f",
                      self.alpha, self.kS, self.kD, self.delay, self.pmax)

    # callbacks
    def cb_target(self, msg):
        val = float(msg.data)
        self.theta_ref = math.radians(val) if self.target_in_deg else val
        self.trigger_plan()
        rospy.loginfo("[target] new target=%.3f deg", (val if self.target_in_deg else math.degrees(val)))

    def cb_joint(self, msg):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            self.theta = float(msg.position[i])

    # planning
    def trigger_plan(self):
        if self.theta is None: return
        theta0_deg = math.degrees(self.theta)
        theta_star_deg = math.degrees(self.theta_ref)
        H = int(self.plan_horizon); dt = float(self.plan_dt)
        ps0 = float(self.ps_cmd); pd0 = float(self.pd_cmd)

        ps_seq, pd_seq = self.planner.plan(theta0=theta0_deg, ps0=ps0, pd0=pd0, theta_star=theta_star_deg,
                                           H=H, dt=dt, w_stage=self.w_stage, w_term=self.w_term, wz=self.wz,
                                           w_sigma=self.w_sigma, w_rate=self.w_rate, w_block=0.0,
                                           rate_sigma=self.rate_sigma_plan, rate_delta=self.rate_delta_plan,
                                           block=max(1,self.block), zc=self.z_coef, max_nfev=600)
        self.plan_ps = ps_seq.tolist(); self.plan_pd = pd_seq.tolist()
        self.plan_idx = 0; self.plan_time_accum = 0.0
        self.phase = "PLAN_EXEC"
        self.pid_out = 0.0; self.int_e = 0.0
        self.success_timer = 0.0
        self.last_replan_time = time.time()
        if len(self.plan_ps)>0:
            rospy.loginfo("[plan] ok: H=%d, dt=%.3f, first Σ=%.3f,Δ=%.3f",
                          len(self.plan_ps), dt, self.plan_ps[0], self.plan_pd[0])
        else:
            rospy.logwarn("[plan] failed")

    # watchdog: success/stagnation/oscillation
    def watchdog(self, dt):
        if self.theta_f is None: return
        e_deg = abs(math.degrees(self.theta_ref - self.theta_f))
        now = time.time()
        # buffer update
        self.err_abs_buf.append((now, e_deg))
        while len(self.err_abs_buf)>self.err_abs_buf_max:
            self.err_abs_buf.popleft()
        # success window
        if e_deg <= self.success_tol_deg:
            self.success_timer += dt
            if self.success_timer >= self.success_hold_s and self.phase != "SUCCESS":
                self.phase = "SUCCESS"
                self.pid_out = 0.0; self.int_e = 0.0
                rospy.loginfo("[success] within %.2f deg for %.2fs → hold pressures",
                              self.success_tol_deg, self.success_hold_s)
        else:
            self.success_timer = 0.0
            # stagnation / oscillation check only if cooldown passed and not planning
            if (now - self.last_replan_time) > self.replan_cooldown_s and self.phase in ("PID_HOLD","PLAN_EXEC"):
                # slope of |e| over stagnation_window
                t0 = now - self.stagnation_window_s
                buf = [ (t,e) for (t,e) in self.err_abs_buf if t>=t0 ]
                if len(buf) >= 5:
                    ts = np.array([b[0]-buf[0][0] for b in buf]); es = np.array([b[1] for b in buf])
                    A = np.vstack([ts, np.ones_like(ts)]).T
                    m, _ = np.linalg.lstsq(A, es, rcond=None)[0]  # deg/s
                    # oscillation amplitude in another window
                    t1 = now - self.oscill_amp_window_s
                    buf2 = [e for (t,e) in self.err_abs_buf if t>=t1]
                    amp = (max(buf2)-min(buf2)) if buf2 else 0.0
                    if ( (m > self.stagnation_slope_thresh_deg_per_s and e_deg > self.stagnation_min_err_deg) or
                         (amp > self.oscill_amp_thresh_deg) ):
                        rospy.logwarn("[watchdog] replan: slope=%.3f deg/s, amp=%.2f deg, err=%.2f deg",
                                      m, amp, e_deg)
                        self.trigger_plan()

    # control loop
    def step(self, dt):
        # theta LPF
        if self.theta is not None:
            if self.theta_f is None: self.theta_f = self.theta
            a = clip(dt/max(1e-3, self.tau_theta), 0.0, 1.0)
            self.theta_f += a*(self.theta - self.theta_f)

        # plan exec
        if self.phase == "PLAN_EXEC" and len(self.plan_ps)>0:
            k = self.plan_idx
            if k >= len(self.plan_ps)-1:
                self.phase = "PID_HOLD"; self.pid_out=0.0; self.int_e=0.0
            else:
                k1 = k+1
                f = clip(self.plan_time_accum/max(1e-9,self.plan_dt), 0.0, 1.0)
                tgt_ps = (1-f)*self.plan_ps[k] + f*self.plan_ps[k1]
                tgt_pd = (1-f)*self.plan_pd[k] + f*self.plan_pd[k1]
                a_cmd = clip(dt/max(1e-3,self.tau_cmd), 0.0, 1.0)
                ps_ref = self.ps_cmd + a_cmd*(tgt_ps - self.ps_cmd)
                pd_ref = self.pd_cmd + a_cmd*(tgt_pd - self.pd_cmd)
                self.ps_cmd = rate_limit(ps_ref, self.ps_cmd, self.rate_sigma_ctrl, dt)
                self.pd_cmd = rate_limit(pd_ref, self.pd_cmd, self.rate_delta_ctrl, dt)
                self.plan_time_accum += dt
                if self.plan_time_accum >= self.plan_dt:
                    self.plan_time_accum -= self.plan_dt; self.plan_idx += 1

        # PID hold
        if self.phase == "PID_HOLD" and self.use_pid and (self.theta_f is not None):
            e_deg_signed = math.degrees(self.theta_ref - self.theta_f)
            e_deg = 0.0 if abs(e_deg_signed) < self.pid_deadband_deg else e_deg_signed
            dtheta = 0.0
            if self.theta_prev is not None: dtheta = (self.theta - self.theta_prev)/max(dt,1e-3)
            self.theta_prev = self.theta
            de_deg = math.degrees(dtheta)

            pid_raw = self.kp_delta*e_deg + self.ki_delta*self.int_e + self.kd_delta*de_deg
            pid_raw = float(np.clip(pid_raw, -self.pid_max_abs, self.pid_max_abs))
            a_pid = clip(dt/max(1e-3,self.tau_cmd), 0.0, 1.0)
            self.pid_out += a_pid * (pid_raw - self.pid_out)

            if not (self.pid_freeze_on_sat and (abs(pid_raw) >= self.pid_max_abs-1e-9)):
                self.int_e += e_deg * dt

            pid_step = rate_limit(self.pid_out, 0.0, 0.2, dt)  # Δの最大変更速度
            self.pd_cmd = clip(self.pd_cmd + pid_step, -self.pmax, self.pmax)

        # SUCCESS：そのまま保持（偶発的な抑制のため緩い一次遅れは残す）
        if self.phase == "SUCCESS":
            a_cmd = clip(dt/max(1e-3,self.tau_cmd), 0.0, 1.0)
            self.ps_cmd = self.ps_cmd + a_cmd*(self.ps_cmd - self.ps_cmd)  # no-op, placeholder
            self.pd_cmd = self.pd_cmd + a_cmd*(self.pd_cmd - self.pd_cmd)

        # clamp and publish
        self.ps_cmd, self.pd_cmd = self.planner.clamp_box(self.ps_cmd, self.pd_cmd)
        p1 = 0.5*(self.ps_cmd + self.pd_cmd); p2 = 0.5*(self.ps_cmd - self.pd_cmd)
        self.pub_p12.publish(Vector3(x=p1, y=p2, z=0.0))
        if self.also_pub_counts:
            self.pub_counts.publish(Vector3(
                x=int(round(p1*self.raw_counts_per_MPa)),
                y=int(round(p2*self.raw_counts_per_MPa)), z=0.0))

        # watchdog
        self.watchdog(dt)

    def spin(self):
        r = rospy.Rate(self.ctrl_rate_hz); last_t = time.time()
        while not rospy.is_shutdown():
            now = time.time(); dt = max(1e-3, now - last_t); last_t = now
            self.step(dt); r.sleep()

if __name__ == "__main__":
    rospy.init_node("neural_hammerstein_mpc_once_then_pid_watchdog_v3")
    node = MPCOnceThenPIDNode(); node.spin()
