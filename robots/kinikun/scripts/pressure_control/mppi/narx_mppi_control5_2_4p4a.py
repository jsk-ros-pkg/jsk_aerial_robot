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
        self.R1 = measurement_noise_1
        self.R2 = measurement_noise_2
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


# ==================== Control System Class ====================

class ControlSystem:
    def __init__(self, name, model, meta, device, dt, mppi_params, 
                 theta_diff_threshold=0.1):
        self.name = name
        self.model = model
        self.meta = meta
        self.device = device
        self.dt = dt
        
        # MPPI parameters
        self.K = mppi_params['K']
        self.H = mppi_params['H']
        self.temperature = mppi_params['temperature']
        self.sigma_u = mppi_params['sigma_u']
        self.w_tracking = mppi_params['w_tracking']
        self.w_smooth = mppi_params['w_smooth']
        self.w_effort = mppi_params['w_effort']
        self.w_constraint = mppi_params['w_constraint']
        self.p_max = mppi_params['p_max']
        self.dp_max = mppi_params['dp_max']
        
        # Model parameters
        self.lags = meta['lags']
        self.feat_cols = meta['feature_names_single_slice']
        self.mu = np.array(meta['mu'], dtype=np.float32)
        self.std = np.array(meta['std'], dtype=np.float32)
        
        # State
        self.theta_rad = 0.0
        self.theta_ref_rad = 0.0
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        # Filters
        self.theta_filter = DualSensorKalmanFilter(
            process_noise=1e-5,
            measurement_noise_1=5e-4,
            measurement_noise_2=5e-4
        )
        self.theta_diff_threshold = theta_diff_threshold
        
        # History
        maxlen = self.lags + 10
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        # Safety
        self.safety = SafetyMonitor(
            theta_rate_max=5.0,
            theta_abs_max=1.5,
            enable_stuck_check=False
        )
        self.emergency_stop = False
        
        # Lock
        self.lock = threading.Lock()
    
    def update_theta(self, theta_1, theta_2):
        diff = abs(theta_1 - theta_2)
        with self.lock:
            if diff > self.theta_diff_threshold:
                rospy.logwarn_throttle(
                    1.0,
                    f"[{self.name}] Large sensor diff: {diff:.4f} rad"
                )
                if abs(theta_1 - self.theta_rad) < abs(theta_2 - self.theta_rad):
                    self.theta_rad = self.theta_filter.update_dual(theta_1, theta_1)
                else:
                    self.theta_rad = self.theta_filter.update_dual(theta_2, theta_2)
            else:
                self.theta_rad = self.theta_filter.update_dual(theta_1, theta_2)
            self.hist_theta.appendleft(self.theta_rad)
    
    def set_target(self, theta_ref_rad):
        self.theta_ref_rad = theta_ref_rad
    
    def update_pressure(self, p1, p2):
        with self.lock:
            self.p1_meas = p1
            self.p2_meas = p2
    
    def enforce_constraints(self, p1, p2, p1_prev, p2_prev):
        dp_max_step = self.dp_max * self.dt
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
        
        for h in range(H):
            dp1 = U[:, h, 0]
            dp2 = U[:, h, 1]
            
            p1_prev = p1_k.copy()
            p2_prev = p2_k.copy()
            
            p1_k = p1_k + dp1
            p2_k = p2_k + dp2
            
            for i in range(K):
                p1_k[i], p2_k[i] = self.enforce_constraints(
                    p1_k[i], p2_k[i], p1_prev[i], p2_prev[i]
                )
            
            dp1_dt = (p1_k - p1_prev) / self.dt
            dp2_dt = (p2_k - p2_prev) / self.dt
            
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
        with self.lock:
            theta = self.theta_rad
            theta_ref = self.theta_ref_rad
            p1_prev = self.p1_cmd
            p2_prev = self.p2_cmd
        
        is_safe, msg = self.safety.check(theta)
        if not is_safe:
            rospy.logerr(f"[{self.name}] Safety violation: {msg}")
            self.emergency_stop = True
            return 0.0, 0.0, {}
        
        U = np.random.normal(
            loc=0.0,
            scale=self.sigma_u,
            size=(self.K, self.H, 2)
        ).astype(np.float32)
        
        theta_seq, p1_seq, p2_seq = self.rollout_batch(theta, p1_prev, p2_prev, U)
        
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
        
        p1_cmd, p2_cmd = self.enforce_constraints(p1_cmd, p2_cmd, p1_prev, p2_prev)
        
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
        
        log_data = {
            'theta': theta,
            'theta_ref': theta_ref,
            'error': theta_ref - theta,
            'J_min': float(np.min(J)),
            'J_mean': float(np.mean(J))
        }
        
        return p1_cmd, p2_cmd, log_data


# ==================== Main Controller ====================

class NARX_MPPI_Controller:
    
    def __init__(self):
        rospy.init_node('narx_mppi_controller_dual', anonymous=False)
        
        # Parameters
        self.model_dir = rospy.get_param("~model_dir", "models/out_narx2")
        self.rate_hz = float(rospy.get_param("~rate", 100.0))
        self.frame_skip = int(rospy.get_param("~frame_skip", 2))
        self.dt = float(self.frame_skip) / self.rate_hz
        
        # MPPI parameters (共通)
        mppi_params = {
            'K': int(rospy.get_param("~K", 32)),
            'H': int(rospy.get_param("~horizon", 15)),
            'temperature': float(rospy.get_param("~lambda", 2.0)),
            'sigma_u': float(rospy.get_param("~sigma_u", 0.10)),
            'w_tracking': float(rospy.get_param("~w_tracking", 30.0)),
            'w_smooth': float(rospy.get_param("~w_smooth", 0.05)),
            'w_effort': float(rospy.get_param("~w_effort", 0.01)),
            'w_constraint': float(rospy.get_param("~w_constraint", 500.0)),
            'p_max': float(rospy.get_param("~p_max", 0.70)),
            'dp_max': float(rospy.get_param("~dp_max", 3.5))
        }
        
        # System 1 sensor indices
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.theta_index_2 = int(rospy.get_param("~theta_index_2", 0))
        self.theta_diff_threshold = float(rospy.get_param("~theta_diff_threshold", 0.1))
        
        # System 2 sensor indices
        self.theta_index_3 = int(rospy.get_param("~theta_index_3", 3))
        self.theta_index_4 = int(rospy.get_param("~theta_index_4", 4))
        self.theta_diff_threshold_2 = float(rospy.get_param("~theta_diff_threshold_2", 0.1))
        
        # Topics
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.target_topic = rospy.get_param("~target_topic", "/theta_target_deg")
        self.target_topic_2 = rospy.get_param("~target_topic_2", "/theta_target_deg_2")
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/mpa_cmd")
        
        # Logging
        self.log_path = rospy.get_param("~log_csv", "")
        self.log_buffer = deque(maxlen=10000)
        self.log_thread = None
        self.log_file = None
        
        # Load model
        rospy.loginfo("[MPPI] Loading model...")
        self.load_model()
        
        # Create two control systems
        self.system1 = ControlSystem(
            "System1", self.model, self.meta, self.device, self.dt,
            mppi_params, self.theta_diff_threshold
        )
        self.system2 = ControlSystem(
            "System2", self.model, self.meta, self.device, self.dt,
            mppi_params, self.theta_diff_threshold_2
        )
        
        self.comp_time_buf = deque(maxlen=100)
        
        # ROS Interface
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Quaternion, queue_size=1)
        self.pub_status = rospy.Publisher("/mppi/status", String, queue_size=1, latch=True)
        
        self.sub_theta = rospy.Subscriber(self.theta_topic, JointState,
                                          self.cb_theta, queue_size=10)
        self.sub_target = rospy.Subscriber(self.target_topic, Float32,
                                           self.cb_target, queue_size=1)
        self.sub_target_2 = rospy.Subscriber(self.target_topic_2, Float32,
                                             self.cb_target_2, queue_size=1)
        self.sub_pressure = rospy.Subscriber(self.pressure_topic, Quaternion,
                                             self.cb_pressure, queue_size=50)
        
        if self.log_path:
            self.setup_logging()
        
        rospy.loginfo("[MPPI] Initialization complete (Dual System)")
        rospy.loginfo(f"  Model: {self.model_dir}")
        rospy.loginfo(f"  Rate: {self.rate_hz} Hz (dt={self.dt:.4f}s)")
        rospy.loginfo(f"  System1 sensors: {self.theta_index_2}, {self.theta_index}")
        rospy.loginfo(f"  System2 sensors: {self.theta_index_3}, {self.theta_index_4}")
        rospy.loginfo(f"  Device: {self.device}")
    
    def load_model(self):
        meta_path = os.path.join(self.model_dir, 'narx_meta.json')
        model_path = os.path.join(self.model_dir, 'narx_model.pt')

        with open(meta_path, 'r') as f:
            self.meta = json.load(f)

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        if self.device.type == 'cpu':
            rospy.logwarn("[MPPI] Running on CPU - performance will be limited!")

        hidden = self.meta['hidden']
        dropout = self.meta.get('dropout', 0.0)
        in_dim = self.meta['lags'] * len(self.meta['feature_names_single_slice'])

        self.model = MLP_NARX(in_dim, hidden=[hidden, hidden], out_dim=1, dropout=dropout)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()
    
    def cb_theta(self, msg: JointState):
        if len(msg.position) > max(self.theta_index, self.theta_index_2, 
                                    self.theta_index_3, self.theta_index_4):
            # System 1
            theta_0 = float(msg.position[self.theta_index_2])
            theta_2 = float(msg.position[self.theta_index])
            theta_0_corrected = -theta_0
            self.system1.update_theta(theta_0_corrected, theta_2)
            
            # System 2
            theta_3 = float(msg.position[self.theta_index_3])
            theta_4 = float(msg.position[self.theta_index_4])
            theta_4_corrected = -theta_4
            self.system2.update_theta(theta_3, theta_4_corrected)
    
    def cb_target(self, msg: Float32):
        self.system1.set_target(math.radians(float(msg.data)))
    
    def cb_target_2(self, msg: Float32):
        self.system2.set_target(math.radians(float(msg.data)))
    
    def cb_pressure(self, msg: Quaternion):
        p1 = float(msg.x)
        p2 = float(msg.y)
        p3 = float(msg.z)
        p4 = float(msg.w)
        self.system1.update_pressure(p1, p2)
        self.system2.update_pressure(p3, p4)
    
    def publish_cmd(self, p1, p2, p3, p4):
        msg = Quaternion()
        msg.x = float(p1) * 4096.0 / 0.9
        msg.y = float(p2) * 4096.0 / 0.9
        msg.z = float(p3) * 4096.0 / 0.9
        msg.w = float(p4) * 4096.0 / 0.9
        self.pub_cmd.publish(msg)
    
    def setup_logging(self):
        import csv
        
        os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
        
        self.log_file = open(self.log_path, 'w', newline='')
        self.log_writer = csv.DictWriter(self.log_file, fieldnames=[
            't',
            'theta1', 'theta_ref1', 'error1', 'p1_cmd', 'p2_cmd', 'p1_meas', 'p2_meas', 'J1_min', 'J1_mean',
            'theta2', 'theta_ref2', 'error2', 'p3_cmd', 'p4_cmd', 'p3_meas', 'p4_meas', 'J2_min', 'J2_mean',
            'comp_time_ms'
        ])
        self.log_writer.writeheader()
        
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
    
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        frame_count = 0
        
        rospy.loginfo("[MPPI] Starting control loop...")
        self.pub_status.publish(String("running"))
        
        # Warmup
        warmup_duration = 2.0
        warmup_start = rospy.get_time()
        
        rospy.loginfo(f"[MPPI] Warmup for {warmup_duration}s...")
        while not rospy.is_shutdown():
            if rospy.get_time() - warmup_start > warmup_duration:
                break
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            rate.sleep()
        
        rospy.loginfo("[MPPI] Control active!")
        
        try:
            while not rospy.is_shutdown():
                if self.system1.emergency_stop or self.system2.emergency_stop:
                    self.publish_cmd(0.0, 0.0, 0.0, 0.0)
                    rospy.logerr("[MPPI] Emergency stop active")
                    rate.sleep()
                    continue
                
                if frame_count % self.frame_skip == 0:
                    t_start = time.time()
                    
                    p1, p2, log1 = self.system1.mppi_step()
                    p3, p4, log2 = self.system2.mppi_step()
                    
                    self.publish_cmd(p1, p2, p3, p4)
                    
                    comp_time = time.time() - t_start
                    self.comp_time_buf.append(comp_time)
                    
                    if self.log_path:
                        self.log_buffer.append({
                            't': rospy.get_time(),
                            'theta1': log1['theta'],
                            'theta_ref1': log1['theta_ref'],
                            'error1': log1['error'],
                            'p1_cmd': p1,
                            'p2_cmd': p2,
                            'p1_meas': self.system1.p1_meas,
                            'p2_meas': self.system1.p2_meas,
                            'J1_min': log1['J_min'],
                            'J1_mean': log1['J_mean'],
                            'theta2': log2['theta'],
                            'theta_ref2': log2['theta_ref'],
                            'error2': log2['error'],
                            'p3_cmd': p3,
                            'p4_cmd': p4,
                            'p3_meas': self.system2.p1_meas,
                            'p4_meas': self.system2.p2_meas,
                            'J2_min': log2['J_min'],
                            'J2_mean': log2['J_mean'],
                            'comp_time_ms': comp_time * 1000.0
                        })
                
                frame_count += 1
                
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
            self.publish_cmd(0.0, 0.0, 0.0, 0.0)
            self.pub_status.publish(String("stopped"))
            if self.log_file:
                self.log_file.close()


# ==================== Main ====================

def main():
    controller = NARX_MPPI_Controller()
    controller.spin()

if __name__ == '__main__':
    main()