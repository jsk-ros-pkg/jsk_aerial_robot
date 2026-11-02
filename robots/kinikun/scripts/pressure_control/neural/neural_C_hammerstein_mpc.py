#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import os, math, time
import numpy as np
import rospy
import torch, torch.nn as nn
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3

# ---------- helpers ----------
def clip(x, lo, hi): return max(lo, min(hi, x))
def rate_limit(target, current, rate_max, dt):
    return current + clip(target - current, -rate_max*dt, rate_max*dt)

# ---------- Neural static kernel (must match training) ----------
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

    model = MonoDeltaNN(M, hidden, c_grid)
    model.load_state_dict(state_dict)
    model.eval()

    @torch.no_grad()
    def f(ps, pd):
        S = torch.tensor([(ps-muS)/sdS], dtype=torch.float32).view(1,1)
        D = torch.tensor([(pd-muD)/sdD], dtype=torch.float32).view(1,1)
        return float(model(S,D).item())
    return f

# ---------- simple predictive planner (least-squares; with fallback) ----------
class Planner(object):
    def __init__(self, theta_stat, alpha, kS, kD, dt_model, delay, pmax, sigma_ref):
        self.f = theta_stat
        self.alpha, self.kS, self.kD = float(alpha), float(kS), float(kD)
        self.dt_model = float(dt_model)
        self.delay = int(delay)
        self.pmax = float(pmax)
        self.sigma_ref = float(sigma_ref)

    def clamp_box(self, ps, pd):
        p1 = 0.5*(ps+pd); p2=0.5*(ps-pd)
        p1 = clip(p1, 0.0, self.pmax)
        p2 = clip(p2, 0.0, self.pmax)
        return p1+p2, p1-p2

    def simulate_cost_terms(self, u_vec, H, dt, theta0, ps0, pd0,
                            theta_star, w_stage, w_term, wz, w_sigma, w_rate, w_block,
                            rate_sigma, rate_delta, block, zc=None):
        def z_pred(ps, pd):
            if zc is None or zc.size < 4: return 0.0
            return float(zc[0] + zc[1]*ps + zc[2]*pd + zc[3]*ps*pd)

        theta = theta0
        cost = []
        for k in range(H):
            ps_cmd = u_vec[2*k+0]; pd_cmd = u_vec[2*k+1]
            # 速度ペナルティ
            if k==0:
                dS = ps_cmd - ps0; dD = pd_cmd - pd0
            else:
                dS = ps_cmd - u_vec[2*(k-1)+0]
                dD = pd_cmd - u_vec[2*(k-1)+1]
            rate_pen = max(0.0, abs(dS)-rate_sigma) + max(0.0, abs(dD)-rate_delta)

            # ボックス制約は大罰則
            ps_eff, pd_eff = self.clamp_box(ps_cmd, pd_cmd)
            p1 = 0.5*(ps_cmd+pd_cmd); p2 = 0.5*(ps_cmd-pd_cmd)
            box_viol = 0.0
            if p1<0: box_viol += -p1
            if p2<0: box_viol += -p2
            if p1>self.pmax: box_viol += (p1-self.pmax)
            if p2>self.pmax: box_viol += (p2-self.pmax)

            # ブロック固定のずれ
            block_pen = 0.0
            if block>1 and (k%block!=0):
                ps_prev = u_vec[2*(k-1)+0]; pd_prev = u_vec[2*(k-1)+1]
                block_pen = (ps_cmd-ps_prev)**2 + (pd_cmd-pd_prev)**2

            # モデル1ステップ
            ths = self.f(ps_eff, pd_eff)
            # 入力の時間微分はコマンド差分で近似
            if k==0:
                dS_eff = (ps_eff - ps0)/dt; dD_eff = (pd_eff - pd0)/dt
            else:
                ps_prev_eff, pd_prev_eff = self.clamp_box(u_vec[2*(k-1)+0], u_vec[2*(k-1)+1])
                dS_eff = (ps_eff - ps_prev_eff)/dt; dD_eff = (pd_eff - pd_prev_eff)/dt

            theta = theta + dt*( self.alpha*(ths - theta) + self.kS*dS_eff + self.kD*dD_eff )

            c = (w_stage*(theta-theta_star)**2
                 + wz*(z_pred(ps_cmd, pd_cmd)**2)
                 + w_sigma*((ps_cmd - self.sigma_ref)**2)
                 + w_rate*(dS**2 + dD**2)
                 + w_block*block_pen
                 + 1e6*box_viol + 1e3*rate_pen)
            cost.append(c)

        cost.append(w_term*(theta-theta_star)**2)
        return np.array(cost, float)

    def plan(self, theta0, ps0, pd0, theta_star, H, dt,
             w_stage, w_term, wz, w_sigma, w_rate, w_block,
             rate_sigma, rate_delta, block, zc=None, max_nfev=500):
        # 初期解：静的逆でΔだけ埋め、Σはsigma_refへ
        u = np.zeros(2*H, float)
        for k in range(H):
            ps = self.sigma_ref
            # 単純な二分でΔ近似
            lo, hi = -self.pmax, self.pmax
            for _ in range(40):
                mid = 0.5*(lo+hi)
                val = self.f(ps, mid) - theta_star
                vlo = self.f(ps, lo) - theta_star
                if np.sign(vlo)*np.sign(val) <= 0:
                    hi = mid
                else:
                    lo = mid
            pd = 0.5*(lo+hi)
            ps_cmd, pd_cmd = self.clamp_box(ps, pd)
            u[2*k+0] = ps_cmd
            u[2*k+1] = pd_cmd

        def obj(x):
            return self.simulate_cost_terms(x, H, dt, theta0, ps0, pd0,
                                            theta_star, w_stage, w_term, wz, w_sigma,
                                            w_rate, w_block, rate_sigma, rate_delta, block, zc)

        # SciPy 最小二乗（TRF）→ 失敗なら粗い数値勾配GD
        try:
            from scipy.optimize import least_squares
            res = least_squares(lambda x: obj(x), u, method="trf",
                                ftol=1e-9, xtol=1e-9, gtol=1e-9, max_nfev=int(max_nfev))
            u_opt = res.x
        except Exception:
            u_opt = u.copy()
            for _ in range(50):
                for i in range(len(u_opt)):
                    base = obj(u_opt).sum()
                    h = 1e-3
                    u_opt[i] += h; c_plus = obj(u_opt).sum()
                    u_opt[i] -= 2*h; c_minus = obj(u_opt).sum()
                    u_opt[i] += h
                    g = (c_plus - c_minus)/(2*h)
                    u_opt[i] -= 0.05*g

        ps_seq = u_opt[0::2].copy()
        pd_seq = u_opt[1::2].copy()
        return ps_seq, pd_seq

# ---------- ROS Node ----------
class MPCOnceThenPIDNode(object):
    def __init__(self):
        # --- params ---
        self.model_pt   = rospy.get_param("~model_pt")
        self.model_meta = rospy.get_param("~model_meta")
        if not (self.model_pt and self.model_meta):
            raise RuntimeError("~model_pt と ~model_meta を指定してください。")

        self.ctrl_rate_hz   = float(rospy.get_param("~ctrl_rate_hz", 100.0))
        self.target_in_deg  = bool(rospy.get_param("~target_in_deg", True))

        # planning
        self.plan_dt        = float(rospy.get_param("~plan_dt", 0.02))
        self.plan_horizon   = int(rospy.get_param("~plan_horizon", 30))
        self.block          = int(rospy.get_param("~block", 3))
        self.w_stage        = float(rospy.get_param("~w_stage", 1.5))
        self.w_term         = float(rospy.get_param("~w_term", 8.0))
        self.w_rate         = float(rospy.get_param("~w_rate", 0.6))
        self.w_sigma        = float(rospy.get_param("~w_sigma", 0.05))
        self.wz             = float(rospy.get_param("~wz", 0.2))
        self.rate_sigma_plan= float(rospy.get_param("~rate_sigma_plan", 0.08))
        self.rate_delta_plan= float(rospy.get_param("~rate_delta_plan", 0.08))

        # command smoothing & rate limits (control stage)
        self.tau_cmd        = float(rospy.get_param("~tau_cmd", 0.10))  # [s] 1st-order smoothing
        self.rate_sigma_ctrl= float(rospy.get_param("~rate_sigma_ctrl", 0.5))  # MPa/s
        self.rate_delta_ctrl= float(rospy.get_param("~rate_delta_ctrl", 0.5))  # MPa/s

        # PID (low-band, Δ only)
        self.use_pid        = bool(rospy.get_param("~use_pid", True))
        self.kp_delta       = float(rospy.get_param("~kp_delta", 0.015))
        self.ki_delta       = float(rospy.get_param("~ki_delta", 0.010))
        self.kd_delta       = float(rospy.get_param("~kd_delta", 0.004))
        self.pid_max_abs    = float(rospy.get_param("~pid_max_abs", 0.04))
        self.pid_deadband_deg = float(rospy.get_param("~pid_deadband_deg", 0.5))
        self.pid_freeze_on_sat = bool(rospy.get_param("~pid_freeze_on_sat", True))
        self.tau_theta      = float(rospy.get_param("~tau_theta", 0.08))  # [s] for theta LPF

        # I/O topics
        self.joint_topic = rospy.get_param("~joint_topic", "/kinikun1/joint_states")
        self.joint_name  = rospy.get_param("~joint_name",  "arm1_joint")
        self.target_topic= rospy.get_param("~target_topic","/target_angle")
        self.pub_topic_p12 = rospy.get_param("~pub_topic_p12", "/p1p2_cmd")
        self.also_pub_counts = bool(rospy.get_param("~also_pub_counts", True))
        self.raw_counts_per_MPa = float(rospy.get_param("~raw_counts_per_MPa", 4096.0/0.9))
        self.pub_topic_counts = rospy.get_param("~pub_topic_counts", "/mpa_cmd")

        # --- load model ---
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

        # --- state ---
        self.theta = None
        self.theta_ref = 0.0  # [rad] 内部表現
        # 初期圧: p1=p2=0.2 -> Σ=0.4, Δ=0
        self.ps_cmd = 0.40
        self.pd_cmd = 0.00
        self.pid_out = 0.0
        self.int_e = 0.0
        self.theta_f = None
        self.theta_prev = None

        # plan buffer
        self.plan_ps = []; self.plan_pd = []
        self.plan_idx = 0
        self.plan_time_accum = 0.0
        self.phase = "HOLD"  # HOLD / PLAN_EXEC / PID_HOLD

        # ROS I/O
        rospy.Subscriber(self.joint_topic, JointState, self.cb_joint)
        rospy.Subscriber(self.target_topic, Float32, self.cb_target)
        self.pub_p12 = rospy.Publisher(self.pub_topic_p12, Vector3, queue_size=10)
        self.pub_counts = rospy.Publisher(self.pub_topic_counts, Vector3, queue_size=10) if self.also_pub_counts else None

        rospy.loginfo("[node] loaded model. alpha=%.3f kS=%.3f kD=%.3f delay=%d pmax=%.2f",
                      self.alpha, self.kS, self.kD, self.delay, self.pmax)

    # --- callbacks ---
    def cb_target(self, msg):
        val = float(msg.data)
        self.theta_ref = math.radians(val) if self.target_in_deg else val
        # 新目標 → 計画フェーズへ
        self.make_plan()
        self.phase = "PLAN_EXEC"
        self.plan_idx = 0
        self.plan_time_accum = 0.0
        rospy.loginfo("[target] new target=%.3f deg", (val if self.target_in_deg else math.degrees(val)))

    def cb_joint(self, msg):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            self.theta = float(msg.position[i])  # [rad]

    # --- planning ---
    def make_plan(self):
        if self.theta is None:
            return
        theta0_deg = math.degrees(self.theta)
        theta_star_deg = math.degrees(self.theta_ref)
        H = int(self.plan_horizon)
        dt = float(self.plan_dt)

        # 現在コマンドの Σ,Δ を初期値とする
        ps0 = float(self.ps_cmd); pd0 = float(self.pd_cmd)
        ps_seq, pd_seq = self.planner.plan(
            theta0=theta0_deg, ps0=ps0, pd0=pd0, theta_star=theta_star_deg,
            H=H, dt=dt, w_stage=self.w_stage, w_term=self.w_term, wz=self.wz,
            w_sigma=self.w_sigma, w_rate=self.w_rate, w_block=self.w_block if hasattr(self, "w_block") else 0.0,
            rate_sigma=self.rate_sigma_plan, rate_delta=self.rate_delta_plan,
            block=max(1,self.block), zc=self.z_coef, max_nfev=600
        )

        self.plan_ps = ps_seq.tolist()
        self.plan_pd = pd_seq.tolist()
        if len(self.plan_ps)>0:
            rospy.loginfo("[plan] ok: H=%d, dt=%.3f, first Σ=%.3f,Δ=%.3f",
                          len(self.plan_ps), dt, self.plan_ps[0], self.plan_pd[0])
        else:
            rospy.logwarn("[plan] failed: empty plan")

    # --- control loop ---
    def step(self, dt):
        # 角度フィルタ（低域通過）
        if self.theta is not None:
            if self.theta_f is None: self.theta_f = self.theta
            a = clip(dt/max(1e-3, self.tau_theta), 0.0, 1.0)
            self.theta_f = self.theta_f + a*(self.theta - self.theta_f)
        # プラン実行 or PID保持
        if self.phase == "PLAN_EXEC" and len(self.plan_ps)>0:
            k = self.plan_idx
            if k >= len(self.plan_ps)-1:
                # 最終点へ到達したら PID_HOLD に移行
                self.phase = "PID_HOLD"
                self.pid_out = 0.0  # リセット
                self.int_e = 0.0
            else:
                # 線形補間（連続参照）
                k_next = k + 1
                f = clip(self.plan_time_accum / max(1e-9, self.plan_dt), 0.0, 1.0)
                tgt_ps = (1.0-f)*self.plan_ps[k] + f*self.plan_ps[k_next]
                tgt_pd = (1.0-f)*self.plan_pd[k] + f*self.plan_pd[k_next]

                # 一次遅れで滑らかに追従
                a_cmd = clip(dt / max(1e-3, self.tau_cmd), 0.0, 1.0)
                ps_ref = self.ps_cmd + a_cmd*(tgt_ps - self.ps_cmd)
                pd_ref = self.pd_cmd + a_cmd*(tgt_pd - self.pd_cmd)

                # レート制限（制御段）
                self.ps_cmd = rate_limit(ps_ref, self.ps_cmd, self.rate_sigma_ctrl, dt)
                self.pd_cmd = rate_limit(pd_ref, self.pd_cmd, self.rate_delta_ctrl, dt)

                # 次サンプルへ
                self.plan_time_accum += dt
                if self.plan_time_accum >= self.plan_dt:
                    self.plan_time_accum -= self.plan_dt
                    self.plan_idx += 1

        # PID保持（Δのみ微修正）
        if self.phase == "PID_HOLD" and self.use_pid and (self.theta_f is not None):
            e_deg = math.degrees(self.theta_ref - self.theta_f)
            e_eff = 0.0 if abs(e_deg) < self.pid_deadband_deg else e_deg
            dtheta = 0.0
            if self.theta_prev is not None:
                dtheta = (self.theta - self.theta_prev) / max(dt,1e-3)
            self.theta_prev = self.theta

            de_deg = math.degrees(dtheta)
            # 積分は飽和時に凍結
            pid_raw = self.kp_delta*e_eff + self.ki_delta*self.int_e + self.kd_delta*de_deg
            pid_raw = float(np.clip(pid_raw, -self.pid_max_abs, self.pid_max_abs))

            # PID出力も一次遅れで滑らかに
            a_pid = clip(dt / max(1e-3, self.tau_cmd), 0.0, 1.0)
            self.pid_out = self.pid_out + a_pid * (pid_raw - self.pid_out)

            if not (self.pid_freeze_on_sat and (abs(pid_raw) >= self.pid_max_abs-1e-9)):
                self.int_e += e_eff * dt

            # PID出力にレート制限を掛けてからΔへ加算
            pid_step = rate_limit(self.pid_out, 0.0, 0.2, dt)  # 0.2 MPa/s くらい
            self.pd_cmd = clip(self.pd_cmd + pid_step, -self.pmax, self.pmax)

        # ボックス制約
        self.ps_cmd, self.pd_cmd = self.planner.clamp_box(self.ps_cmd, self.pd_cmd)

        # 出力
        p1 = 0.5*(self.ps_cmd + self.pd_cmd)
        p2 = 0.5*(self.ps_cmd - self.pd_cmd)
        self.publish_p12(p1, p2)

    def publish_p12(self, p1, p2):
        self.pub_p12.publish(Vector3(x=p1, y=p2, z=0.0))
        if self.pub_counts is not None:
            raw1 = int(round(p1 * self.raw_counts_per_MPa))
            raw2 = int(round(p2 * self.raw_counts_per_MPa))
            self.pub_counts.publish(Vector3(x=raw1, y=raw2, z=0.0))

    def spin(self):
        r = rospy.Rate(self.ctrl_rate_hz)
        last_t = time.time()
        while not rospy.is_shutdown():
            now = time.time()
            dt = max(1e-3, now - last_t); last_t = now
            self.step(dt)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("neural_hammerstein_mpc_once_then_pid_v2")
    node = MPCOnceThenPIDNode()
    node.spin()
