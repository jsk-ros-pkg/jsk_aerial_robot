#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, time, math, threading
from collections import deque
from typing import Tuple
import asyncio

import numpy as np
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import JointState

import torch
import torch.nn as nn

# ==================== Utility Classes ====================

class DualSensorKalmanFilter:
    def __init__(self, process_noise=1e-5, 
                 measurement_noise_1=5e-4, 
                 measurement_noise_2=5e-4):
        self.Q = process_noise
        self.R1 = measurement_noise_1  # センサー1のノイズ
        self.R2 = measurement_noise_2  # センサー2のノイズ
        self.P = 1.0
        self.x = 0.0
    
    def update_dual(self, z1, z2):
        P_pred = self.P + self.Q
        
        K1 = P_pred / (P_pred + self.R1)
        x_temp = self.x + K1 * (z1 - self.x)
        P_temp = (1 - K1) * P_pred
        
        K2 = P_temp / (P_temp + self.R2)
        self.x = x_temp + K2 * (z2 - x_temp)
        self.P = (1 - K2) * P_temp
        
        return self.x
    
    def reset(self, x0):
        self.x = x0
        self.P = 1.0


class SafetyMonitor:
    def __init__(self, theta_rate_max=0.3, theta_abs_max=1.5,
                 enable_stuck_check=False, stuck_count_max=2000):
        self.last_theta = 0.0
        self.last_time = time.time()
        self.theta_rate_max = theta_rate_max  # rad/s
        self.theta_abs_max = theta_abs_max    # rad
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


# ==================== MPPI Controller ====================

class NARX_MPPI_Controller:    
    def __init__(self):
        rospy.init_node('narx_mppi_controller', anonymous=False)
        
        # ========== Parameters ==========
        self.model_dir = rospy.get_param("~model_dir", "models/out_narx2")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.frame_skip = int(rospy.get_param("~frame_skip", 2))
        self.dt = float(self.frame_skip) / self.rate_hz
        
        # MPPI
        self.K = int(rospy.get_param("~K", 32))          # Population
        self.H = int(rospy.get_param("~horizon", 15))    # Prediction horizon
        self.temperature = float(rospy.get_param("~lambda", 2.0))
        self.sigma_u = float(rospy.get_param("~sigma_u", 0.10))
        
        # Costs
        self.w_tracking = float(rospy.get_param("~w_tracking", 30.0))
        self.w_smooth = float(rospy.get_param("~w_smooth", 0.05))
        self.w_effort = float(rospy.get_param("~w_effort", 0.01))
        self.w_constraint = float(rospy.get_param("~w_constraint", 500.0))
        
        # Physical limits
        self.p_max = float(rospy.get_param("~p_max", 0.70))
        self.dp_max = float(rospy.get_param("~dp_max", 3.5))  # MPa/s
        self.pressure_delay_s = float(rospy.get_param("~pressure_delay_s", 0.084))
        
        # Topics
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.theta_index_2 = int(rospy.get_param("~theta_index_2", 0))
        self.theta_diff_threshold = float(rospy.get_param("~theta_diff_threshold", 0.1))
        self.target_topic = rospy.get_param("~target_topic", "/theta_target_deg")
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/mpa_cmd")
        
        # Logging
        self.log_path = rospy.get_param("~log_csv", "")
        self.log_buffer = deque(maxlen=10000)
        self.log_thread = None
        self.log_file = None
        
        # ========== Load Model ==========
        rospy.loginfo("[MPPI] Loading model...")
        self.load_model()
        
        # ========== State Variables ==========
        self.lock = threading.Lock()
        
        # Filtered state
        self.theta_rad = 0.0
        self.theta_filter = DualSensorKalmanFilter(
            process_noise=1e-5, 
            measurement_noise_1=5e-4,
            measurement_noise_2=5e-4
        )
        
        # Command / measured pressures
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        # Target
        self.theta_ref_rad = 0.0
        
        maxlen = self.lags + 10
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        self.press_buf = deque(maxlen=200)
        
        self.safety = SafetyMonitor(
            theta_rate_max=5.0,
            theta_abs_max=1.5,
            enable_stuck_check=False,
            stuck_count_max=2000
        )
        self.emergency_stop = False
        
        # Performance monitoring
        self.comp_time_buf = deque(maxlen=100)
        
        # ========== ROS Interface ==========
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Quaternion, queue_size=1)
        self.pub_status = rospy.Publisher("/mppi/status", String, queue_size=1, latch=True)
        
        self.sub_theta = rospy.Subscriber(self.theta_topic, JointState,
                                          self.cb_theta, queue_size=10)
        self.sub_target = rospy.Subscriber(self.target_topic, Float32,
                                           self.cb_target, queue_size=1)
        self.sub_pressure = rospy.Subscriber(self.pressure_topic, Quaternion,
                                             self.cb_pressure, queue_size=50)
        
        # Logging
        if self.log_path:
            self.setup_logging()
        
        rospy.loginfo("[MPPI] Initialization complete")
        rospy.loginfo(f"  Model: {self.model_dir}")
        rospy.loginfo(f"  Rate: {self.rate_hz} Hz (frame_skip={self.frame_skip}, dt={self.dt:.4f}s)")
        rospy.loginfo(f"  MPPI: K={self.K}, H={self.H}")
        rospy.loginfo(f"  Device: {self.device}")
        rospy.loginfo(f"  Dual sensors: index {self.theta_index_2} and {self.theta_index} (diff_threshold={self.theta_diff_threshold:.4f} rad)")
    
    # ========== Model Loading ==========
    
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
            rospy.logwarn("[MPPI] Running on CPU - performance will be limited!")

        in_dim = self.lags * len(self.feat_cols)

        self.model = MLP_NARX(
            in_dim,
            hidden=[self.hidden, self.hidden],
            out_dim=1,
            dropout=self.dropout
        )

        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()

        rospy.loginfo(
            f"[MPPI] Model loaded: lags={self.lags}, delay={self.delay}, "
            f"hidden={self.hidden}, dropout={self.dropout}"
        )
    
    # ========== ROS Callbacks ==========
    
    def cb_theta(self, msg: JointState):
        if len(msg.position) > max(self.theta_index, self.theta_index_2):
            theta_0 = float(msg.position[self.theta_index_2])  # センサー1
            theta_2 = float(msg.position[self.theta_index])     # センサー2
            
            diff = abs(theta_0 - theta_2)
            
            with self.lock:
                if diff > self.theta_diff_threshold:
                    rospy.logwarn_throttle(
                        1.0, 
                        f"[MPPI] Large sensor diff: {diff:.4f} rad (threshold: {self.theta_diff_threshold:.4f})"
                    )
                    if abs(theta_0 - self.theta_rad) < abs(theta_2 - self.theta_rad):
                        self.theta_rad = self.theta_filter.update_dual(theta_0, theta_0)
                    else:
                        self.theta_rad = self.theta_filter.update_dual(theta_2, theta_2)
                else:
                    self.theta_rad = self.theta_filter.update_dual(theta_0, theta_2)
                
                self.hist_theta.appendleft(self.theta_rad)
    
    def cb_target(self, msg: Float32):
        self.theta_ref_rad = math.radians(float(msg.data))
    
    def cb_pressure(self, msg: Quaternion):
        t = rospy.get_time()
        p1 = float(msg.x)
        p2 = float(msg.y)
        with self.lock:
            self.press_buf.append((t, p1, p2))
            self.p1_meas = p1
            self.p2_meas = p2
    
    # ========== Feature Construction ==========
    
    def build_feature_vector_current(self) -> np.ndarray:
        with self.lock:
            theta_hist = list(self.hist_theta)[:self.lags]
            p1_hist = list(self.hist_p1_cmd)[:self.lags]
            p2_hist = list(self.hist_p2_cmd)[:self.lags]
            dp1_hist = list(self.hist_dp1_dt)[:self.lags]
            dp2_hist = list(self.hist_dp2_dt)[:self.lags]

        def pad_hist(hist, fill=0.0):
            if len(hist) == 0:
                return [fill] * self.lags
            if len(hist) < self.lags:
                last = hist[-1]
                hist = hist + [last] * (self.lags - len(hist))
            return hist

        theta_hist = pad_hist(theta_hist, self.theta_rad)
        p1_hist = pad_hist(p1_hist, self.p1_cmd)
        p2_hist = pad_hist(p2_hist, self.p2_cmd)
        dp1_hist = pad_hist(dp1_hist, 0.0)
        dp2_hist = pad_hist(dp2_hist, 0.0)

        x_list = []
        for k in range(self.lags):
            x_list.extend([
                theta_hist[k],
                p1_hist[k],
                p2_hist[k],
                dp1_hist[k],
                dp2_hist[k],
            ])
        x = np.array(x_list, dtype=np.float32).reshape(1, -1)
        x_norm = (x - self.mu) / (self.std + 1e-8)
        return x_norm
    
    # ========== MPPI Core ==========
    
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
        in_dim = self.lags * n_feat_per_lag

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

            theta_hist = np.concatenate(
                [theta_k[:, None], theta_hist[:, :-1]], axis=1
            )
            p1_hist = np.concatenate(
                [p1_k[:, None], p1_hist[:, :-1]], axis=1
            )
            p2_hist = np.concatenate(
                [p2_k[:, None], p2_hist[:, :-1]], axis=1
            )
            dp1_hist = np.concatenate(
                [dp1_dt[:, None], dp1_hist[:, :-1]], axis=1
            )
            dp2_hist = np.concatenate(
                [dp2_dt[:, None], dp2_hist[:, :-1]], axis=1
            )

            X_chunks = []
            for k in range(self.lags):
                X_chunks.append(theta_hist[:, k][:, None])
                X_chunks.append(p1_hist[:, k][:, None])
                X_chunks.append(p2_hist[:, k][:, None])
                X_chunks.append(dp1_hist[:, k][:, None])
                X_chunks.append(dp2_hist[:, k][:, None])
            X_batch = np.concatenate(X_chunks, axis=1).astype(np.float32)  # (K, in_dim)

            X_norm = (X_batch - self.mu) / (self.std + 1e-8)

            with torch.no_grad():
                Y_batch = self.model(torch.from_numpy(X_norm).to(self.device))
            theta_k = Y_batch.cpu().numpy().flatten()

            theta_seq[:, h] = theta_k
            p1_seq[:, h] = p1_k
            p2_seq[:, h] = p2_k

        return theta_seq, p1_seq, p2_seq
    
    def mppi_step(self):
        t_start = time.time()
        
        with self.lock:
            theta = self.theta_rad
            theta_ref = self.theta_ref_rad
            p1_prev = self.p1_cmd
            p2_prev = self.p2_cmd
        
        is_safe, msg = self.safety.check(theta)
        if not is_safe:
            rospy.logerr(f"[MPPI] Safety violation: {msg}")
            self.emergency_stop = True
            self.publish_cmd(0.0, 0.0)
            return
        
        U = np.random.normal(
            loc=0.0,
            scale=self.sigma_u,
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
        
        dU = np.sum(w[:, None, None] * U, axis=0) / w_sum  # (H, 2)
        
        dp1_cmd, dp2_cmd = dU[0, 0], dU[0, 1]
        p1_cmd = p1_prev + dp1_cmd
        p2_cmd = p2_prev + dp2_cmd
        
        p1_cmd, p2_cmd = self.enforce_constraints(p1_cmd, p2_cmd, p1_prev, p2_prev, dt)
        
        self.publish_cmd(p1_cmd, p2_cmd)
        
        with self.lock:
            self.p1_cmd = p1_cmd
            self.p2_cmd = p2_cmd
            self.hist_p1_cmd.appendleft(p1_cmd)
            self.hist_p2_cmd.appendleft(p2_cmd)
            if len(self.hist_p1_cmd) > 1:
                dp1_dt = (self.hist_p1_cmd[0] - self.hist_p1_cmd[1]) / dt
                dp2_dt = (self.hist_p2_cmd[0] - self.hist_p2_cmd[1]) / dt
            else:
                dp1_dt, dp2_dt = 0.0, 0.0
            self.hist_dp1_dt.appendleft(dp1_dt)
            self.hist_dp2_dt.appendleft(dp2_dt)
        
        comp_time = time.time() - t_start
        self.comp_time_buf.append(comp_time)

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
                'J_min': float(np.min(J)),
                'J_mean': float(np.mean(J)),
                'comp_time_ms': comp_time * 1000.0
            })
    
    # ========== Command Publishing ==========
    
    def publish_cmd(self, p1, p2):
        msg = Quaternion()
        msg.x = float(p1) * 4096.0 / 0.9
        msg.y = float(p2) * 4096.0 / 0.9
        msg.z = 0.0
        msg.w = 0.0
        self.pub_cmd.publish(msg)
    
    # ========== Logging ==========
    
    def setup_logging(self):
        import csv
        
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[
            't', 'theta', 'theta_ref', 'error',
            'p1_cmd', 'p2_cmd', 'p1_meas', 'p2_meas',
            'J_min', 'J_mean', 'comp_time_ms'
        ])
        self.log_writer.writeheader()
        
        # Start logging thread
        self.log_thread = threading.Thread(target=self.logging_worker, daemon=True)
        self.log_thread.start()
        
        rospy.loginfo(f"[MPPI] Logging to: {self.log_path}")
    
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
                    rospy.logerr(f"[MPPI] Logging error: {e}")
            rate.sleep()
    
    # ========== Main Loop ==========
    
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        frame_count = 0
        
        rospy.loginfo("[MPPI] Starting control loop...")
        self.pub_status.publish(String("running"))
        
        warmup_duration = 2.0  # seconds
        warmup_start = rospy.get_time()
        
        rospy.loginfo(f"[MPPI] Warmup for {warmup_duration}s...")
        while not rospy.is_shutdown():
            if rospy.get_time() - warmup_start > warmup_duration:
                break
            self.publish_cmd(0.0, 0.0)
            rate.sleep()
        
        rospy.loginfo("[MPPI] Control active!")
        
        try:
            while not rospy.is_shutdown():
                if self.emergency_stop:
                    self.publish_cmd(0.0, 0.0)
                    rospy.logerr("[MPPI] Emergency stop active")
                    rate.sleep()
                    continue
                
                # Control update
                if frame_count % self.frame_skip == 0:
                    self.mppi_step()
                
                frame_count += 1
                
                # Performance report (every 10s)
                if frame_count % int(self.rate_hz * 10) == 0:
                    if len(self.comp_time_buf) > 0:
                        avg_time = np.mean(self.comp_time_buf)
                        max_time = np.max(self.comp_time_buf)
                        rospy.loginfo(
                            f"[MPPI] Comp time: avg={avg_time*1000:.1f}ms, "
                            f"max={max_time*1000:.1f}ms"
                        )
                
                rate.sleep()
        
        except rospy.ROSInterruptException:
            pass
        
        finally:
            rospy.loginfo("[MPPI] Shutting down...")
            self.publish_cmd(0.0, 0.0)
            self.pub_status.publish(String("stopped"))
            if self.log_file:
                self.log_file.close()


# ==================== Main ====================

def main():
    controller = NARX_MPPI_Controller()
    controller.spin()

if __name__ == '__main__':
    main()