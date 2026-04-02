#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal MPA experiment protocol (bag-first; no CSV)

Publishes:
  /mpa_cmd     : geometry_msgs/Vector3  (x=p1_scaled, y=p2_scaled, z=0), where scaled = MPa * scale_gain
  exp/ps_MPa   : std_msgs/Float32  (sum pressure ps in MPa)
  exp/pd_MPa   : std_msgs/Float32  (diff pressure pd in MPa)
  exp/mode     : std_msgs/String   (state indicator; latched)
  exp/is_busy  : std_msgs/Bool     (latched)
  exp/tag      : std_msgs/String   (lightweight tag for labeling)

Inputs (sensors, optional but useful for settle check in static mode):
  ~theta_topic : std_msgs/Float32 [deg]
  ~z_topic     : std_msgs/Float32 [m]

Services:
  /exp/pause, /exp/resume, /exp/skip_point, /exp/estop  (std_srvs/Trigger)

Modes:
  _mode:=static_grid / dyn_prbs / dyn_multisine / dyn_chirp / dyn_random

static_grid1 (low pressure, fewer points):
rosrun kinikun data_train_pspd.py \
  _mode:=static_grid _rate_hz:=200 \
  _p_max_MPa:=0.70 _p_sum_min_MPa:=0.10 _p_sum_max_MPa:=0.80 \
  _grid_sum_pts:=8 _grid_diff_pts:=13 \
  _dwell_sec:=1.2 _settle_min_sec:=0.5 _settle_max_sec:=3.0 \
  _slope_win_sec:=0.25 _settle_eps_deg:=0.18 _settle_eps_z:=0.002 \
  _theta_topic:=/theta_deg _z_topic:=/z_m \
  _publish_scaled_cmd:=true _warmup_sec:=3.0

static_grid2 (more points)
rosrun kinikun data_train_pspd.py \
  _mode:=static_grid _rate_hz:=200 \
  _p_max_MPa:=0.70 _p_sum_min_MPa:=0.10 _p_sum_max_MPa:=1.30 \
  _grid_sum_pts:=13 _grid_diff_pts:=13 \
  _dwell_sec:=1.0 _settle_min_sec:=0.5 _settle_max_sec:=3.0 \
  _slope_win_sec:=0.25 _settle_eps_deg:=0.20 _settle_eps_z:=0.002 \
  _theta_topic:=/theta_deg _z_topic:=/z_m \
  _publish_scaled_cmd:=true _warmup_sec:=2.0

dyn_prbs:
rosrun kinikun data_train_pspd.py \
  _mode:=dyn_prbs _session_sec:=900 _rate_hz:=200 \
  _dyn_ps_bias:=0.55 _dyn_ps_span:=0.25 _dyn_pd_span:=0.60 \
  _prbs_pd_interval_min:=0.15 _prbs_pd_interval_max:=0.35 \
  _prbs_ps_interval_min:=0.8  _prbs_ps_interval_max:=1.5 \
  _rate_ps_max:=2.0 _rate_pd_max:=4.0 \
  _theta_topic:=/theta_deg _z_topic:=/z_m \
  _publish_scaled_cmd:=true _warmup_sec:=2.0

dyn_multisine:
rosrun kinikun data_train_pspd.py \
  _mode:=dyn_multisine _session_sec:=420 _rate_hz:=200 \
  _dyn_ps_bias:=0.50 _dyn_ps_span:=0.30 _dyn_pd_span:=0.55 \
  _multisine_pd_freqs:="0.3,0.6,1.2,2.4" \
  _multisine_ps_freqs:="0.2,0.4" \
  _rate_ps_max:=2.0 _rate_pd_max:=4.0 \
  _publish_scaled_cmd:=true

dyn_chirp:
rosrun kinikun data_train_pspd.py \
  _mode:=dyn_chirp _session_sec:=360 _rate_hz:=200 \
  _dyn_ps_bias:=0.50 _dyn_ps_span:=0.25 _dyn_pd_span:=0.60 \
  _chirp_f_lo:=0.2 _chirp_f_hi:=5.0 _chirp_two_sided:=true \
  _rate_ps_max:=2.0 _rate_pd_max:=4.0 \
  _publish_scaled_cmd:=true

  
"""

import math, random, signal
from collections import deque

import numpy as np
import rospy
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Vector3
from std_srvs.srv import Trigger, TriggerResponse

# ---------- utils ----------
def clamp(x, lo, hi): return lo if x < lo else (hi if x > hi else x)
def ps_pd_to_p1p2(ps, pd): return 0.5*(ps + pd), 0.5*(ps - pd)

def feasible_clip_ps_pd(ps, pd, pmax):
    # 0 ≤ (ps±pd)/2 ≤ pmax  ⇔ |pd| ≤ min(ps, 2*pmax-ps), 0≤ps≤2*pmax
    ps = clamp(ps, 0.0, 2.0*pmax)
    pd_bound = min(ps, 2.0*pmax - ps)
    pd = clamp(pd, -pd_bound, pd_bound)
    return ps, pd

def rate_limit(x, x_prev, rate_max, dt):
    if rate_max <= 0: return x
    dx = clamp(x - x_prev, -rate_max*dt, rate_max*dt)
    return x_prev + dx

def slope_of(series, times):
    n = len(series)
    if n < 2: return 0.0
    t = np.asarray(times); y = np.asarray(series)
    tmean, ymean = t.mean(), y.mean()
    denom = np.sum((t - tmean)**2)
    if denom <= 1e-12:
        dt = max(1e-6, t[-1]-t[0]); return float((y[-1]-y[0])/dt)
    return float(np.sum((t - tmean)*(y - ymean)) / denom)

# ---------- node ----------
class ExpMPAProtocolNodeMin:
    def __init__(self):
        # core
        self.rate_hz = rospy.get_param("~rate_hz", 200)
        self.dt = 1.0/float(self.rate_hz)

        # pressure limits
        self.pmax   = float(rospy.get_param("~p_max_MPa", 0.70))
        self.ps_min = float(rospy.get_param("~p_sum_min_MPa", 0.0))
        self.ps_max = float(min(rospy.get_param("~p_sum_max_MPa", 1.40), 2.0*self.pmax))

        # outputs
        self.scale_gain = float(rospy.get_param("~scale_gain", 4096/0.9))  # MPa -> regulator
        self.pub_cmd    = rospy.Publisher("mpa_cmd", Vector3, queue_size=1)
        self.pub_ps     = rospy.Publisher("exp/ps_MPa", Float32, queue_size=1)
        self.pub_pd     = rospy.Publisher("exp/pd_MPa", Float32, queue_size=1)
        self.pub_mode   = rospy.Publisher("exp/mode", String, queue_size=1, latch=True)
        self.pub_busy   = rospy.Publisher("exp/is_busy", Bool, queue_size=1, latch=True)
        self.pub_tag    = rospy.Publisher("exp/tag", String, queue_size=1)

        # sensors (optional)
        self.theta_deg = 0.0
        self.z_m = 0.0
        rospy.Subscriber(rospy.get_param("~theta_topic", "/theta_deg"), Float32, lambda m: setattr(self, "theta_deg", float(m.data)))
        rospy.Subscriber(rospy.get_param("~z_topic", "/z_m"), Float32, lambda m: setattr(self, "z_m", float(m.data)))

        # static params
        self.grid_sum_pts   = int(rospy.get_param("~grid_sum_pts", 12))
        self.grid_diff_pts  = int(rospy.get_param("~grid_diff_pts", 13))
        self.dwell_sec      = float(rospy.get_param("~dwell_sec", 1.0))
        self.settle_min_sec = float(rospy.get_param("~settle_min_sec", 0.5))
        self.settle_max_sec = float(rospy.get_param("~settle_max_sec", 3.0))
        self.settle_eps_deg = float(rospy.get_param("~settle_eps_deg", 0.20))
        self.settle_eps_z   = float(rospy.get_param("~settle_eps_z", 0.002))
        self.slope_win_sec  = float(rospy.get_param("~slope_win_sec", 0.25))
        self.static_repeat  = int(rospy.get_param("~static_repeat", 1))

        # dynamic params
        self.mode = str(rospy.get_param("~mode", "static_grid"))
        self.session_sec = float(rospy.get_param("~session_sec", 600.0))
        self.ps_bias = float(rospy.get_param("~dyn_ps_bias", 0.5))
        self.ps_span = float(rospy.get_param("~dyn_ps_span", 0.3))
        self.pd_span = float(rospy.get_param("~dyn_pd_span", 0.6))
        self.rate_ps_max = float(rospy.get_param("~rate_ps_max", 2.0))
        self.rate_pd_max = float(rospy.get_param("~rate_pd_max", 4.0))

        self.prbs_pd_interval_min = float(rospy.get_param("~prbs_pd_interval_min", 0.15))
        self.prbs_pd_interval_max = float(rospy.get_param("~prbs_pd_interval_max", 0.35))
        self.prbs_ps_interval_min = float(rospy.get_param("~prbs_ps_interval_min", 0.6))
        self.prbs_ps_interval_max = float(rospy.get_param("~prbs_ps_interval_max", 1.2))

        self.ms_pd_freqs = self._parse_freqs(rospy.get_param("~multisine_pd_freqs", "0.3,0.6,1.2,2.4"))
        self.ms_ps_freqs = self._parse_freqs(rospy.get_param("~multisine_ps_freqs", "0.2,0.4"))

        self.chirp_f_lo = float(rospy.get_param("~chirp_f_lo", 0.2))
        self.chirp_f_hi = float(rospy.get_param("~chirp_f_hi", 5.0))
        self.chirp_two_sided = bool(rospy.get_param("~chirp_two_sided", True))

        self.random_step_ps = float(rospy.get_param("~random_step_ps", 0.02))
        self.random_step_pd = float(rospy.get_param("~random_step_pd", 0.04))

        self.warmup_sec = float(rospy.get_param("~warmup_sec", 0.0))

        # states
        self.ps_cmd = clamp(self.ps_bias, self.ps_min, self.ps_max)
        self.pd_cmd = 0.0
        self.ps_prev = self.ps_cmd
        self.pd_prev = self.pd_cmd
        self._next_pd_switch = 0.0
        self._next_ps_switch = 0.0

        self.static_seq = None
        self.static_idx = 0
        self.static_phase = "init"
        self.point_start_time = None

        # settle buffers
        self.buf_time  = deque()
        self.buf_theta = deque()
        self.buf_z     = deque()
        self.buf_maxlen = max(3, int(self.slope_win_sec / self.dt))

        # control flags
        self.is_paused = False
        self.is_estop  = False

        # services
        rospy.Service("exp/pause", Trigger, self._srv_pause)
        rospy.Service("exp/resume", Trigger, self._srv_resume)
        rospy.Service("exp/skip_point", Trigger, self._srv_skip)
        rospy.Service("exp/estop", Trigger, self._srv_estop)

        # ctrl+c -> zero
        signal.signal(signal.SIGINT, self._on_sigint)

    # ---- services ----
    def _srv_pause(self, _):
        self.is_paused = True
        return TriggerResponse(success=True, message="Paused")

    def _srv_resume(self, _):
        self.is_paused = False
        return TriggerResponse(success=True, message="Resumed")

    def _srv_skip(self, _):
        if self.mode == "static_grid" and self.static_seq is not None:
            self.static_idx = min(self.static_idx + 1, len(self.static_seq))
            return TriggerResponse(success=True, message="Skipped one static point")
        return TriggerResponse(success=False, message="Skip only for static_grid")

    def _srv_estop(self, _):
        self.is_estop = True
        self._publish_cmd(0.0, 0.0, tag="estop")
        return TriggerResponse(success=True, message="E-STOP engaged")

    def _on_sigint(self, *_):
        try: self._publish_cmd(0.0, 0.0, tag="sigint_zero")
        except Exception: pass
        raise rospy.ROSInterruptException

    # ---- helpers ----
    def _parse_freqs(self, obj):
        if isinstance(obj, (list, tuple)): return [float(x) for x in obj]
        s = str(obj).strip()
        return [float(x) for x in s.split(",")] if s else []

    def _append_slopes(self, now):
        self.buf_time.append(now); self.buf_theta.append(self.theta_deg); self.buf_z.append(self.z_m)
        while len(self.buf_time) > self.buf_maxlen:
            self.buf_time.popleft(); self.buf_theta.popleft(); self.buf_z.popleft()

    def _publish_cmd(self, ps, pd, tag=""):
        # feasibility & rate limits
        ps, pd = feasible_clip_ps_pd(ps, pd, self.pmax)
        ps = rate_limit(ps, self.ps_prev, self.rate_ps_max, self.dt)
        pd = rate_limit(pd, self.pd_prev, self.rate_pd_max, self.dt)
        ps, pd = feasible_clip_ps_pd(ps, pd, self.pmax)

        p1, p2 = ps_pd_to_p1p2(ps, pd)
        self.pub_cmd.publish(Vector3(p1*self.scale_gain, p2*self.scale_gain, 0.0))
        self.pub_ps.publish(Float32(ps))
        self.pub_pd.publish(Float32(pd))
        if tag: self.pub_tag.publish(String(tag))

        self.ps_prev, self.pd_prev = ps, pd
        return ps, pd

    # ---- static sequence ----
    def _prepare_static(self):
        ps_list = np.linspace(self.ps_min, self.ps_max, max(1, self.grid_sum_pts))
        seq = []
        for direction in ["up", "down"]:
            it = ps_list if direction=="up" else ps_list[::-1]
            for ps in it:
                pd_max = min(ps, 2.0*self.pmax - ps)
                if pd_max < 1e-9:
                    for _ in range(max(1,self.static_repeat)):
                        seq.append((ps, 0.0, direction))
                    continue
                pd_list = np.linspace(-pd_max, pd_max, max(1, self.grid_diff_pts))
                for pd in pd_list:
                    for _ in range(max(1,self.static_repeat)):
                        seq.append((ps, pd, direction))
        self.static_seq = seq
        self.static_idx = 0
        self.static_phase = "step"
        self.point_start_time = None
        rospy.loginfo(f"[exp_mpa_min] static points: {len(seq)}")

    def _static_once(self, now):
        if self.static_seq is None: self._prepare_static()
        if self.static_idx >= len(self.static_seq): return False

        ps_tgt, pd_tgt, direction = self.static_seq[self.static_idx]
        tag_base = f"static[{direction}]/idx={self.static_idx}"

        if self.static_phase == "step":
            self.pub_mode.publish(String("static_step"))
            self._publish_cmd(ps_tgt, pd_tgt, tag=f"{tag_base}/step")
            self.point_start_time = now
            self.buf_time.clear(); self.buf_theta.clear(); self.buf_z.clear()
            self.static_phase = "wait"
            return True

        if self.static_phase == "wait":
            self.pub_mode.publish(String("static_wait"))
            self._publish_cmd(ps_tgt, pd_tgt, tag=f"{tag_base}/wait")
            self._append_slopes(now)

            elapsed = now - self.point_start_time
            if elapsed < max(self.dwell_sec, self.settle_min_sec): return True

            s_theta = abs(slope_of(list(self.buf_theta), list(self.buf_time)))
            s_z     = abs(slope_of(list(self.buf_z), list(self.buf_time)))
            if (s_theta < self.settle_eps_deg) and (s_z < self.settle_eps_z): 
                self.static_phase = "record"
                return True
            if elapsed > self.settle_max_sec: 
                self.static_phase = "record"
                return True
            return True

        if self.static_phase == "record":
            self.pub_mode.publish(String("static_record"))
            self._publish_cmd(ps_tgt, pd_tgt, tag=f"{tag_base}/record")
            self.static_idx += 1
            self.static_phase = "step"
            return True

        return True

    # ---- dynamic targets ----
    def _dyn_prbs(self, t):
        if t >= self._next_ps_switch:
            self.ps_cmd = clamp(self.ps_bias + random.choice([-1,1])*self.ps_span, self.ps_min, self.ps_max)
            self._next_ps_switch = t + random.uniform(self.prbs_ps_interval_min, self.prbs_ps_interval_max)
        if t >= self._next_pd_switch:
            self.pd_cmd = random.choice([-1,1]) * self.pd_span
            self._next_pd_switch = t + random.uniform(self.prbs_pd_interval_min, self.prbs_pd_interval_max)
        return self.ps_cmd, self.pd_cmd

    def _dyn_multisine(self, t):
        ps = self.ps_bias
        for i,f in enumerate(self.ms_ps_freqs):
            ps += (self.ps_span/max(1,len(self.ms_ps_freqs))) * math.sin(2*math.pi*f*t + i*0.7)
        pd = 0.0
        for i,f in enumerate(self.ms_pd_freqs):
            pd += (self.pd_span/max(1,len(self.ms_pd_freqs))) * math.sin(2*math.pi*f*t + i*1.1)
        return ps, pd

    def _dyn_chirp(self, t, T):
        tau = (t % T)/T
        if self.chirp_two_sided and int(t / T) % 2 == 1:
            tau = 1.0 - tau
        f = self.chirp_f_lo * (self.chirp_f_hi/self.chirp_f_lo)**tau
        pd = self.pd_span * math.sin(2*math.pi*f*t)
        ps = self.ps_bias + 0.5*self.ps_span * math.sin(2*math.pi*0.3*t)
        return ps, pd

    def _dyn_random(self, _t):
        self.ps_cmd = clamp(self.ps_cmd + random.uniform(-self.random_step_ps, self.random_step_ps),
                            self.ps_min, self.ps_max)
        self.pd_cmd = self.pd_cmd + random.uniform(-self.random_step_pd, self.random_step_pd)
        return self.ps_cmd, self.pd_cmd

    # ---- main ----
    def spin(self):
        r = rospy.Rate(self.rate_hz)
        self.pub_busy.publish(Bool(True))
        self.pub_mode.publish(String(self.mode))
        rospy.loginfo(f"[exp_mpa_min] mode={self.mode} rate={self.rate_hz}Hz")

        # warmup (optional)
        start = rospy.Time.now().to_sec()
        if self.warmup_sec > 1e-6:
            rospy.loginfo(f"[exp_mpa_min] warmup {self.warmup_sec:.1f}s")
            while not rospy.is_shutdown() and rospy.Time.now().to_sec() - start < self.warmup_sec:
                if self.is_estop: break
                if not self.is_paused:
                    self._publish_cmd(self.ps_bias, 0.0, tag="warmup")
                r.sleep()

        # prepare static
        if self.mode == "static_grid":
            self._prepare_static()

        # session
        sess_start = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - sess_start

            if self.is_estop:
                self._publish_cmd(0.0, 0.0, tag="estop_hold")
                self.pub_mode.publish(String("estop"))
                r.sleep(); continue

            if self.is_paused:
                self._publish_cmd(self.ps_prev, self.pd_prev, tag="pause_hold")
                self.pub_mode.publish(String("paused"))
                r.sleep(); continue

            if self.mode == "static_grid":
                cont = self._static_once(now)
                if not cont:
                    rospy.loginfo("[exp_mpa_min] static sequence finished")
                    break
            else:
                if t >= self.session_sec:
                    rospy.loginfo(f"[exp_mpa_min] dynamic session finished ({self.mode})")
                    break

                if self.mode == "dyn_prbs":
                    ps_tgt, pd_tgt = self._dyn_prbs(t)
                elif self.mode == "dyn_multisine":
                    ps_tgt, pd_tgt = self._dyn_multisine(t)
                elif self.mode == "dyn_chirp":
                    ps_tgt, pd_tgt = self._dyn_chirp(t, max(1.0, self.session_sec))
                elif self.mode == "dyn_random":
                    ps_tgt, pd_tgt = self._dyn_random(t)
                else:
                    rospy.logwarn(f"[exp_mpa_min] unknown mode={self.mode}, fallback dyn_prbs")
                    ps_tgt, pd_tgt = self._dyn_prbs(t)

                self._publish_cmd(ps_tgt, pd_tgt, tag="dyn")

            r.sleep()

        # graceful end
        self._publish_cmd(0.0, 0.0, tag="end_zero")
        self.pub_busy.publish(Bool(False))
        self.pub_mode.publish(String("end"))
        rospy.loginfo("[exp_mpa_min] done")

# ---- main ----
if __name__ == "__main__":
    rospy.init_node("exp_mpa_protocol_min")
    node = ExpMPAProtocolNodeMin()
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass
