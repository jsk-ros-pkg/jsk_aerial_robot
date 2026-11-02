#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
narx_mppi_controller_with_hold.py
目標到達時に圧力を保持するMPPI制御ノード

Key Features:
- 目標到達判定（±閾値内）
- Hold mode: 圧力固定、制御停止
- 感度低減: sigma_u を段階的に減少

Usage:
  roslaunch kinikun narx_mppi_hold.launch
"""
import os, json, time, math, threading
from collections import deque
from typing import Tuple
import numpy as np
import rospy
from std_msgs.msg import Float32, String, Bool
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
import torch
import torch.nn as nn

# ==================== Utility Classes ====================

class SimpleKalmanFilter:
    """1次元カルマンフィルタ"""
    def __init__(self, process_noise=1e-5, measurement_noise=1e-3):
        self.Q = process_noise
        self.R = measurement_noise
        self.P = 1.0
        self.x = 0.0
    
    def update(self, z):
        P_pred = self.P + self.Q
        K = P_pred / (P_pred + self.R)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * P_pred
        return self.x
    
    def reset(self, x0):
        self.x = x0
        self.P = 1.0

class SafetyMonitor:
    """安全監視"""
    def __init__(self, theta_rate_max=0.3, theta_abs_max=1.5,
                 enable_stuck_check=False, stuck_count_max=2000):
        self.last_theta = 0.0
        self.last_time = time.time()
        self.theta_rate_max = theta_rate_max
        self.theta_abs_max = theta_abs_max
        self.stuck_count = 0
        self.enable_stuck_check = enable_stuck_check
        self.stuck_count_max = stuck_count_max

    def check(self, theta):
        now = time.time()
        dt = now - self.last_time

        if abs(theta) > self.theta_abs_max:
            return False, f"Theta out of range: {theta:.3f} rad"

        if dt > 1e-6:
            rate = abs(theta - self.last_theta) / dt
            if rate > self.theta_rate_max:
                return False, f"Theta rate too high: {rate:.2f} rad/s"

        if self.enable_stuck_check:
            if abs(theta - self.last_theta) < 1e-6:
                self.stuck_count += 1
                if self.stuck_count > self.stuck_count_max:
                    return False, "Sensor appears stuck"
            else:
                self.stuck_count = 0

        self.last_theta = theta
        self.last_time = now
        return True, "OK"

# ==================== NARX Model ====================

class MLP_NARX(nn.Module):
    """NARXモデル"""
    def __init__(self, in_dim, hidden=[256, 256], out_dim=1, dropout=0.0):
        super().__init__()
        layers = []
        d = in_dim
        for h in hidden:
            layers.append(nn.Linear(d, h))
            layers.append(nn.ReLU())
            if dropout > 0:
                layers.append(nn.Dropout(dropout))
            d = h
        layers.append(nn.Linear(d, out_dim))
        self.net = nn.Sequential(*layers)
    
    def forward(self, x):
        return self.net(x)

# ==================== MPPI Controller with Hold ====================

class NARX_MPPI_Controller_WithHold:
    """目標到達時に圧力保持する MPPI 制御ノード"""
    
    def __init__(self):
        rospy.init_node('narx_mppi_controller_hold', anonymous=False)
        
        # ========== Parameters ==========
        self.model_dir = rospy.get_param("~model_dir", "models/narx_p1p2_production2")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.frame_skip = int(rospy.get_param("~frame_skip", 2))
        self.dt = float(self.frame_skip) / self.rate_hz
        
        # MPPI
        self.K = int(rospy.get_param("~K", 32))
        self.H = int(rospy.get_param("~horizon", 15))
        self.temperature = float(rospy.get_param("~lambda", 2.0))
        self.sigma_u = float(rospy.get_param("~sigma_u", 0.10))
        
        # ★ Hold mode parameters
        self.hold_threshold_deg = float(rospy.get_param("~hold_threshold_deg", 2.0))
        self.hold_duration_s = float(rospy.get_param("~hold_duration_s", 1.0))
        self.sigma_u_decay = float(rospy.get_param("~sigma_u_decay", 0.95))  # 感度減衰率
        self.sigma_u_min = float(rospy.get_param("~sigma_u_min", 0.02))
        
        # Costs
        self.w_tracking = float(rospy.get_param("~w_tracking", 30.0))
        self.w_smooth = float(rospy.get_param("~w_smooth", 0.05))
        self.w_effort = float(rospy.get_param("~w_effort", 0.01))
        self.w_constraint = float(rospy.get_param("~w_constraint", 500.0))
        
        # Physical limits
        self.p_max = float(rospy.get_param("~p_max", 0.70))
        self.dp_max = float(rospy.get_param("~dp_max", 3.5))
        self.pressure_delay_s = float(rospy.get_param("~pressure_delay_s", 0.084))
        
        # Topics
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.target_topic = rospy.get_param("~target_topic", "/theta_target_deg")
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/mpa_cmd")
        
        # Logging
        self.log_path = rospy.get_param("~log_csv", "")
        self.log_buffer = deque(maxlen=10000)
        self.log_thread = None
        self.log_file = None
        
        # ========== Load Model ==========
        rospy.loginfo("[MPPI-Hold] Loading model...")
        self.load_model()
        
        # ========== State Variables ==========
        self.lock = threading.Lock()
        
        self.theta_rad = 0.0
        self.theta_filter = SimpleKalmanFilter(process_noise=1e-5, measurement_noise=5e-4)
        
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        self.theta_ref_rad = 0.0
        
        # History buffers
        maxlen = self.lags + 10
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        self.press_buf = deque(maxlen=200)
        
        # Safety
        self.safety = SafetyMonitor(
            theta_rate_max=5.0,
            theta_abs_max=1.5,
            enable_stuck_check=False,
            stuck_count_max=2000
        )
        self.emergency_stop = False
        
        # ★ Hold mode state
        self.hold_mode = False
        self.hold_start_time = None
        self.p1_hold = 0.0
        self.p2_hold = 0.0
        self.current_sigma_u = self.sigma_u  # 動的に変化
        
        # Performance monitoring
        self.comp_time_buf = deque(maxlen=100)
        
        # ========== ROS Interface ==========
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Vector3, queue_size=1)
        self.pub_status = rospy.Publisher("/mppi/status", String, queue_size=1, latch=True)
        self.pub_hold_status = rospy.Publisher("/mppi/hold_mode", Bool, queue_size=1, latch=True)
        
        self.sub_theta = rospy.Subscriber(self.theta_topic, JointState,
                                          self.cb_theta, queue_size=10)
        self.sub_target = rospy.Subscriber(self.target_topic, Float32,
                                           self.cb_target, queue_size=1)
        self.sub_pressure = rospy.Subscriber(self.pressure_topic, Vector3,
                                             self.cb_pressure, queue_size=50)
        
        # Logging
        if self.log_path:
            self.setup_logging()
        
        rospy.loginfo("[MPPI-Hold] Initialization complete")
        rospy.loginfo(f"  Hold threshold: {self.hold_threshold_deg}°")
        rospy.loginfo(f"  Hold duration:  {self.hold_duration_s}s")
        rospy.loginfo(f"  Sigma decay:    {self.sigma_u_decay}")
    
    # ========== Model Loading (same as before) ==========
    
    def load_model(self):
        meta_path = os.path.join(self.model_dir, 'narx_meta.json')
        model_path = os.path.join(self.model_dir, 'narx_model.pt')

        with open(meta_path, 'r') as f:
            self.meta = json.load(f)

        self.lags = self.meta['lags']
        self.delay = self.meta['delay']
        self.feat_cols = self.meta['feature_names_single_slice']
        self.mu = np.array(self.meta['mu'], dtype=np.float32)
        self.std = np.array(self.meta['std'], dtype=np.float32)
        self.hidden = self.meta['hidden']
        self.dropout = self.meta.get('dropout', 0.0)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if self.device.type == 'cpu':
            rospy.logwarn("[MPPI-Hold] Running on CPU")

        in_dim = self.lags * len(self.feat_cols)
        self.model = MLP_NARX(in_dim, hidden=[self.hidden, self.hidden],
                              out_dim=1, dropout=self.dropout)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        rospy.loginfo(f"[MPPI-Hold] Model loaded: lags={self.lags}, delay={self.delay}")
    
    # ========== ROS Callbacks ==========
    
    def cb_theta(self, msg: JointState):
        if self.theta_index < len(msg.position):
            theta_raw = float(msg.position[self.theta_index])
            with self.lock:
                self.theta_rad = self.theta_filter.update(theta_raw)
                self.hist_theta.appendleft(self.theta_rad)
    
    def cb_target(self, msg: Float32):
        new_target = math.radians(float(msg.data))
        
        # 目標値が変わったら hold mode 解除
        if abs(new_target - self.theta_ref_rad) > math.radians(0.5):
            with self.lock:
                self.hold_mode = False
                self.hold_start_time = None
                self.current_sigma_u = self.sigma_u  # リセット
                rospy.loginfo(f"[MPPI-Hold] Target changed to {msg.data:.1f}°, exiting hold mode")
                self.pub_hold_status.publish(Bool(False))
        
        self.theta_ref_rad = new_target
    
    def cb_pressure(self, msg: Vector3):
        t = rospy.get_time()
        p1 = float(msg.x)
        p2 = float(msg.y)
        with self.lock:
            self.press_buf.append((t, p1, p2))
            self.p1_meas = p1
            self.p2_meas = p2
    
    # ========== Hold Mode Logic ==========
    
    def check_hold_condition(self, theta, theta_ref):
        """
        目標到達判定
        
        Returns:
            should_hold: True if entering/staying in hold mode
        """
        error_deg = abs(math.degrees(theta_ref - theta))
        
        if error_deg < self.hold_threshold_deg:
            # 閾値内
            if not self.hold_mode:
                # 新規エントリー
                if self.hold_start_time is None:
                    self.hold_start_time = rospy.get_time()
                else:
                    # 一定時間継続したら hold mode 確定
                    if rospy.get_time() - self.hold_start_time > self.hold_duration_s:
                        self.hold_mode = True
                        self.p1_hold = self.p1_cmd
                        self.p2_hold = self.p2_cmd
                        rospy.loginfo(f"[MPPI-Hold] Entering hold mode: p1={self.p1_hold:.3f}, p2={self.p2_hold:.3f}")
                        self.pub_hold_status.publish(Bool(True))
            else:
                # すでに hold mode
                pass
        else:
            # 閾値外 → hold mode 解除
            if self.hold_mode:
                rospy.loginfo("[MPPI-Hold] Exiting hold mode (error increased)")
                self.pub_hold_status.publish(Bool(False))
            self.hold_mode = False
            self.hold_start_time = None
            self.current_sigma_u = self.sigma_u  # リセット
        
        return self.hold_mode
    
    # ========== MPPI Core (with hold) ==========
    
    def enforce_constraints(self, p1, p2, p1_prev, p2_prev, dt):
        dp_max_step = self.dp_max * dt
        p1 = np.clip(p1, p1_prev - dp_max_step, p1_prev + dp_max_step)
        p2 = np.clip(p2, p2_prev - dp_max_step, p2_prev + dp_max_step)
        p1 = np.clip(p1, 0.0, self.p_max)
        p2 = np.clip(p2, 0.0, self.p_max)
        return p1, p2
    
    def cost_function(self, theta, theta_ref, p1, p2, p1_prev, p2_prev,
                      dp1, dp2, k, H):
        err = theta_ref - theta
        cost = self.w_tracking * (err ** 2)
        if k == H - 1:
            cost += self.w_tracking * 0.5 * (err ** 2)
        cost += self.w_smooth * (dp1 ** 2 + dp2 ** 2)
        cost += self.w_effort * (p1 ** 2 + p2 ** 2)
        
        viol = 0.0
        if p1 < 0:
            viol += (-p1) ** 2
        if p2 < 0:
            viol += (-p2) ** 2
        if p1 > self.p_max:
            viol += (p1 - self.p_max) ** 2
        if p2 > self.p_max:
            viol += (p2 - self.p_max) ** 2
        cost += self.w_constraint * viol
        return cost
    
    def rollout_batch(self, theta0, p1_0, p2_0, U):
        """バッチ rollout（同じ実装）"""
        K, H = U.shape[0], U.shape[1]
        dt = self.dt

        theta_seq = np.zeros((K, H), dtype=np.float32)
        p1_seq = np.zeros((K, H), dtype=np.float32)
        p2_seq = np.zeros((K, H), dtype=np.float32)

        theta_k = np.full(K, theta0, dtype=np.float32)
        p1_k = np.full(K, p1_0, dtype=np.float32)
        p2_k = np.full(K, p2_0, dtype=np.float32)

        with self.lock:
            theta_hist0 = list(self.hist_theta)[:self.lags]
            p1_hist0 = list(self.hist_p1_cmd)[:self.lags]
            p2_hist0 = list(self.hist_p2_cmd)[:self.lags]
            dp1_hist0 = list(self.hist_dp1_dt)[:self.lags]
            dp2_hist0 = list(self.hist_dp2_dt)[:self.lags]

        def pad_hist(hist, fill):
            if len(hist) == 0:
                return [fill] * self.lags
            if len(hist) < self.lags:
                last = hist[-1]
                hist = hist + [last] * (self.lags - len(hist))
            return hist

        theta_hist0 = pad_hist(theta_hist0, theta0)
        p1_hist0 = pad_hist(p1_hist0, p1_0)
        p2_hist0 = pad_hist(p2_hist0, p2_0)
        dp1_hist0 = pad_hist(dp1_hist0, 0.0)
        dp2_hist0 = pad_hist(dp2_hist0, 0.0)

        theta_hist = np.tile(np.array(theta_hist0, dtype=np.float32), (K, 1))
        p1_hist = np.tile(np.array(p1_hist0, dtype=np.float32), (K, 1))
        p2_hist = np.tile(np.array(p2_hist0, dtype=np.float32), (K, 1))
        dp1_hist = np.tile(np.array(dp1_hist0, dtype=np.float32), (K, 1))
        dp2_hist = np.tile(np.array(dp2_hist0, dtype=np.float32), (K, 1))

        n_feat_per_lag = len(self.feat_cols)

        for h in range(H):
            dp1 = U[:, h, 0]
            dp2 = U[:, h, 1]

            p1_prev = p1_k.copy()
            p2_prev = p2_k.copy()

            p1_k = p1_k + dp1
            p2_k = p2_k + dp2

            for i in range(K):
                p1_k[i], p2_k[i] = self.enforce_constraints(
                    p1_k[i], p2_k[i], p1_prev[i], p2_prev[i], dt
                )

            dp1_dt = (p1_k - p1_prev) / dt
            dp2_dt = (p2_k - p2_prev) / dt

            theta_hist = np.concatenate([theta_k[:, None], theta_hist[:, :-1]], axis=1)
            p1_hist = np.concatenate([p1_k[:, None], p1_hist[:, :-1]], axis=1)
            p2_hist = np.concatenate([p2_k[:, None], p2_hist[:, :-1]], axis=1)
            dp1_hist = np.concatenate([dp1_dt[:, None], dp1_hist[:, :-1]], axis=1)
            dp2_hist = np.concatenate([dp2_dt[:, None], dp2_hist[:, :-1]], axis=1)

            X_chunks = []
            for k in range(self.lags):
                X_chunks.append(theta_hist[:, k][:, None])
                X_chunks.append(p1_hist[:, k][:, None])
                X_chunks.append(p2_hist[:, k][:, None])
                X_chunks.append(dp1_hist[:, k][:, None])
                X_chunks.append(dp2_hist[:, k][:, None])
            X_batch = np.concatenate(X_chunks, axis=1).astype(np.float32)
            X_norm = (X_batch - self.mu) / (self.std + 1e-8)

            with torch.no_grad():
                Y_batch = self.model(torch.from_numpy(X_norm).to(self.device))
            theta_k = Y_batch.cpu().numpy().flatten()

            theta_seq[:, h] = theta_k
            p1_seq[:, h] = p1_k
            p2_seq[:, h] = p2_k

        return theta_seq, p1_seq, p2_seq
    
    def mppi_step(self):
        """MPPI制御ステップ（hold mode対応）"""
        t_start = time.time()
        
        with self.lock:
            theta = self.theta_rad
            theta_ref = self.theta_ref_rad
            p1_prev = self.p1_cmd
            p2_prev = self.p2_cmd
        
        # Safety check
        is_safe, msg = self.safety.check(theta)
        if not is_safe:
            rospy.logerr(f"[MPPI-Hold] Safety violation: {msg}")
            self.emergency_stop = True
            self.publish_cmd(0.0, 0.0)
            return
        
        # ★ Hold mode check
        is_holding = self.check_hold_condition(theta, theta_ref)
        
        if is_holding:
            # Hold mode: 圧力固定
            p1_cmd = self.p1_hold
            p2_cmd = self.p2_hold
            rospy.loginfo_throttle(5.0, f"[MPPI-Hold] Holding: p1={p1_cmd:.3f}, p2={p2_cmd:.3f}")
        else:
            # 通常制御
            # ★ 目標に近づくにつれて sigma_u を減衰
            error_deg = abs(math.degrees(theta_ref - theta))
            if error_deg < 5.0:  # 5度以内なら感度低減
                self.current_sigma_u = max(
                    self.sigma_u_min,
                    self.current_sigma_u * self.sigma_u_decay
                )
            else:
                self.current_sigma_u = self.sigma_u  # 元に戻す
            
            U = np.random.normal(
                loc=0.0,
                scale=self.current_sigma_u,
                size=(self.K, self.H, 2)
            ).astype(np.float32)
            
            theta_seq, p1_seq, p2_seq = self.rollout_batch(theta, p1_prev, p2_prev, U)
            
            dt = self.dt
            J = np.zeros(self.K, dtype=np.float32)
            
            for i in range(self.K):
                cost = 0.0
                p1_h, p2_h = p1_prev, p2_prev
                for h in range(self.H):
                    dp1 = U[i, h, 0]
                    dp2 = U[i, h, 1]
                    cost += self.cost_function(
                        theta_seq[i, h], theta_ref,
                        p1_seq[i, h], p2_seq[i, h],
                        p1_h, p2_h, dp1, dp2, h, self.H
                    )
                    p1_h = p1_seq[i, h]
                    p2_h = p2_seq[i, h]
                J[i] = cost
            
            beta = np.min(J)
            w = np.exp(-(J - beta) / max(1e-6, self.temperature))
            w_sum = np.sum(w) + 1e-9
            
            dU = np.sum(w[:, None, None] * U, axis=0) / w_sum
            
            dp1_cmd, dp2_cmd = dU[0, 0], dU[0, 1]
            p1_cmd = p1_prev + dp1_cmd
            p2_cmd = p2_prev + dp2_cmd
            
            p1_cmd, p2_cmd = self.enforce_constraints(p1_cmd, p2_cmd, p1_prev, p2_prev, dt)
        
        # Publish
        self.publish_cmd(p1_cmd, p2_cmd)
        
        # Update history
        with self.lock:
            self.p1_cmd = p1_cmd
            self.p2_cmd = p2_cmd
            self.hist_p1_cmd.appendleft(p1_cmd)
            self.hist_p2_cmd.appendleft(p2_cmd)
            if len(self.hist_p1_cmd) > 1:
                dp1_dt = (self.hist_p1_cmd[0] - self.hist_p1_cmd[1]) / self.dt
                dp2_dt = (self.hist_p2_cmd[0] - self.hist_p2_cmd[1]) / self.dt
            else:
                dp1_dt, dp2_dt = 0.0, 0.0
            self.hist_dp1_dt.appendleft(dp1_dt)
            self.hist_dp2_dt.appendleft(dp2_dt)
        
        comp_time = time.time() - t_start
        self.comp_time_buf.append(comp_time)
        
        # Logging
        if self.log_path:
            err = theta_ref - theta
            self.log_buffer.append({
                't': rospy.get_time(),
                'theta': theta,
                'theta_ref': theta_ref,
                'error': err,
                'p1_cmd': p1_cmd,
                'p2_cmd': p2_cmd,
                'p1_meas': self.p1_meas,
                'p2_meas': self.p2_meas,
                'J_min': 0.0 if is_holding else float(np.min(J)),
                'J_mean': 0.0 if is_holding else float(np.mean(J)),
                'comp_time_ms': comp_time * 1000.0,
                'hold_mode': int(is_holding),
                'sigma_u': self.current_sigma_u
            })
    
    # ========== Command Publishing ==========
    
    def publish_cmd(self, p1, p2):
        msg = Vector3()
        msg.x = float(p1) * 4096.0 / 0.9
        msg.y = float(p2) * 4096.0 / 0.9
        msg.z = 0.0
        self.pub_cmd.publish(msg)
    
    # ========== Logging ==========
    
    def setup_logging(self):
        import csv
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[
            't', 'theta', 'theta_ref', 'error',
            'p1_cmd', 'p2_cmd', 'p1_meas', 'p2_meas',
            'J_min', 'J_mean', 'comp_time_ms',
'hold_mode', 'sigma_u'
        ])
        self.log_writer.writeheader()
        self.log_thread = threading.Thread(target=self.logging_worker, daemon=True)
        self.log_thread.start()
        rospy.loginfo(f"[MPPI-Hold] Logging to: {self.log_path}")
    
    def logging_worker(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if len(self.log_buffer) > 0:
                batch = []
                while len(self.log_buffer) > 0 and len(batch) < 100:
                    batch.append(self.log_buffer.popleft())
                try:
                    self.log_writer.writerows(batch)
                    self.log_file.flush()
                except Exception as e:
                    rospy.logerr(f"[MPPI-Hold] Logging error: {e}")
            rate.sleep()
    
    # ========== Main Loop ==========
    
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        frame_count = 0
        
        rospy.loginfo("[MPPI-Hold] Starting control loop...")
        self.pub_status.publish(String("running"))
        
        # Warmup
        warmup_duration = 2.0
        warmup_start = rospy.get_time()
        rospy.loginfo(f"[MPPI-Hold] Warmup for {warmup_duration}s...")
        while not rospy.is_shutdown():
            if rospy.get_time() - warmup_start > warmup_duration:
                break
            self.publish_cmd(0.0, 0.0)
            rate.sleep()
        
        rospy.loginfo("[MPPI-Hold] Control active!")
        
        try:
            while not rospy.is_shutdown():
                if self.emergency_stop:
                    self.publish_cmd(0.0, 0.0)
                    rospy.logerr("[MPPI-Hold] Emergency stop active")
                    rate.sleep()
                    continue
                
                if frame_count % self.frame_skip == 0:
                    self.mppi_step()
                
                frame_count += 1
                
                # Performance report
                if frame_count % int(self.rate_hz * 10) == 0:
                    if len(self.comp_time_buf) > 0:
                        avg_time = np.mean(self.comp_time_buf)
                        max_time = np.max(self.comp_time_buf)
                        rospy.loginfo(
                            f"[MPPI-Hold] Comp time: avg={avg_time*1000:.1f}ms, "
                            f"max={max_time*1000:.1f}ms | "
                            f"Hold: {self.hold_mode} | Sigma: {self.current_sigma_u:.4f}"
                        )
                
                rate.sleep()
        
        except rospy.ROSInterruptException:
            pass
        
        finally:
            rospy.loginfo("[MPPI-Hold] Shutting down...")
            self.publish_cmd(0.0, 0.0)
            self.pub_status.publish(String("stopped"))
            if self.log_file:
                self.log_file.close()

def main():
    controller = NARX_MPPI_Controller_WithHold()
    controller.spin()

if __name__ == '__main__':
    main()