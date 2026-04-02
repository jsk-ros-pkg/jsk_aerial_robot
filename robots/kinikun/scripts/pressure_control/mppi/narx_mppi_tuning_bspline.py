#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mppi_tuning_integrated.py
MPPI制御とチューニングを統合したスクリプト

1つのスクリプトで制御とチューニングを完結
- パラメータを変更して即座に制御実行
- 再起動不要
- 結果を自動保存
"""

import os
import time
import json
import math
import threading
from datetime import datetime
from collections import deque
from typing import Tuple
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Vector3, Quaternion
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
    """簡易安全監視"""
    def __init__(self, theta_rate_max=5.0, theta_abs_max=1.5):
        self.last_theta = 0.0
        self.last_time = time.time()
        self.theta_rate_max = theta_rate_max
        self.theta_abs_max = theta_abs_max

    def check(self, theta):
        now = time.time()
        dt = now - self.last_time

        if abs(theta) > self.theta_abs_max:
            return False, f"Theta out of range: {theta:.3f} rad"

        if dt > 1e-6:
            rate = abs(theta - self.last_theta) / dt
            if rate > self.theta_rate_max:
                return False, f"Theta rate too high: {rate:.2f} rad/s"

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


# ==================== Trajectory Generator ====================

class TrajectoryGenerator:
    """軌道生成"""
    
    @staticmethod
    def smooth_transition(start, end, n_points):
        """5次多項式で滑らかな遷移"""
        if n_points < 2:
            return np.array([start, end])
        t = np.linspace(0, 1, n_points)
        s = 10 * t**3 - 15 * t**4 + 6 * t**5
        return start + (end - start) * s
    
    @staticmethod
    def generate_stepwise(waypoints, dt=0.01):
        """停止あり軌道: [(angle_deg, trans_time_s, hold_time_s), ...]"""
        t_arr, theta_arr = [], []
        current_time, current_angle = 0.0, 0.0
        
        for target_deg, trans_time, hold_time in waypoints:
            # 遷移
            n_points = max(10, int(trans_time / dt))
            smooth = TrajectoryGenerator.smooth_transition(current_angle, target_deg, n_points)
            times = current_time + np.arange(n_points) * dt
            t_arr.extend(times)
            theta_arr.extend(smooth)
            current_time = times[-1] + dt
            current_angle = target_deg
            
            # 保持
            n_hold = int(hold_time / dt)
            if n_hold > 0:
                times_hold = current_time + np.arange(n_hold) * dt
                t_arr.extend(times_hold)
                theta_arr.extend([target_deg] * n_hold)
                current_time = times_hold[-1] + dt
        
        return np.array(t_arr), np.array(theta_arr)
    
    @staticmethod
    def generate_continuous(waypoints, dt=0.01):
        """連続軌道: [(angle_deg, trans_time_s), ...]"""
        t_arr, theta_arr = [], []
        current_time, current_angle = 0.0, 0.0
        
        for target_deg, trans_time in waypoints:
            n_points = max(10, int(trans_time / dt))
            smooth = TrajectoryGenerator.smooth_transition(current_angle, target_deg, n_points)
            times = current_time + np.arange(n_points) * dt
            t_arr.extend(times)
            theta_arr.extend(smooth)
            current_time = times[-1] + dt
            current_angle = target_deg
        
        return np.array(t_arr), np.array(theta_arr)


# ==================== Integrated MPPI Controller with Tuning ====================

class IntegratedMPPITuner:
    """MPPI制御とチューニングを統合"""
    
    def __init__(self, model_dir, output_dir="tuning_results"):
        rospy.init_node('integrated_mppi_tuner', anonymous=False)
        
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # ========== Load Model ==========
        self.load_model(model_dir)
        
        # ========== ROS Topics ==========
        self.theta_topic = rospy.get_param("~theta_topic", "/kinikun1/joint_states")
        self.theta_index = int(rospy.get_param("~theta_index", 2))
        self.pressure_topic = rospy.get_param("~pressure_topic", "/mpa_pressure")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/mpa_cmd")
        self.target_topic = rospy.get_param("~target_topic", "/theta_target_deg")
        
        # ========== State Variables ==========
        self.lock = threading.Lock()
        
        self.theta_rad = 0.0
        self.theta_filter = SimpleKalmanFilter(process_noise=1e-5, measurement_noise=5e-4)
        
        self.p1_cmd = 0.0
        self.p2_cmd = 0.0
        self.p1_meas = 0.0
        self.p2_meas = 0.0
        
        self.theta_ref_rad = 0.0
        self.current_target_deg = 0.0
        
        # History buffers
        maxlen = self.lags + 10
        self.hist_theta = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p1_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_p2_cmd = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp1_dt = deque([0.0] * maxlen, maxlen=maxlen)
        self.hist_dp2_dt = deque([0.0] * maxlen, maxlen=maxlen)
        
        # Safety
        self.safety = SafetyMonitor(theta_rate_max=5.0, theta_abs_max=1.5)
        self.emergency_stop = False
        
        # Data collection
        self.time_data = deque(maxlen=100000)
        self.theta_data = deque(maxlen=100000)
        self.target_data = deque(maxlen=100000)
        self.p1_cmd_data = deque(maxlen=100000)
        self.p2_cmd_data = deque(maxlen=100000)
        self.start_time = None
        
        # ========== ROS Interface ==========
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Quaternion, queue_size=1)
        self.pub_target = rospy.Publisher(self.target_topic, Float32, queue_size=1)
        
        self.sub_theta = rospy.Subscriber(self.theta_topic, JointState,
                                          self.cb_theta, queue_size=10)
        self.sub_pressure = rospy.Subscriber(self.pressure_topic, Quaternion,
                                             self.cb_pressure, queue_size=50)
        
        rospy.loginfo("[Tuner] Integrated MPPI Tuner initialized")
        rospy.loginfo(f"  Model: {model_dir}")
        rospy.loginfo(f"  Device: {self.device}")
        rospy.loginfo(f"  Output: {output_dir}")
    
    # ========== Model Loading ==========
    
    def load_model(self, model_dir):
        meta_path = os.path.join(model_dir, 'narx_meta.json')
        model_path = os.path.join(model_dir, 'narx_model.pt')

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
        in_dim = self.lags * len(self.feat_cols)

        self.model = MLP_NARX(in_dim, hidden=[self.hidden, self.hidden],
                              out_dim=1, dropout=self.dropout)
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))
        self.model.to(self.device)
        self.model.eval()
    
    # ========== ROS Callbacks ==========
    
    def cb_theta(self, msg: JointState):
        if self.theta_index < len(msg.position):
            theta_raw = float(msg.position[self.theta_index])
            with self.lock:
                self.theta_rad = self.theta_filter.update(theta_raw)
                self.hist_theta.appendleft(self.theta_rad)
                
                # データ収集
                if self.start_time is not None:
                    t = rospy.get_time() - self.start_time
                    self.time_data.append(t)
                    self.theta_data.append(np.degrees(self.theta_rad))
                    self.target_data.append(self.current_target_deg)
                    self.p1_cmd_data.append(self.p1_cmd)
                    self.p2_cmd_data.append(self.p2_cmd)
    
    def cb_pressure(self, msg: Quaternion):
        with self.lock:
            self.p1_meas = float(msg.x)
            self.p2_meas = float(msg.y)
    
    # ========== MPPI Core ==========
    
    def enforce_constraints(self, p1, p2, p1_prev, p2_prev, dt, p_max, dp_max):
        """物理制約適用"""
        dp_max_step = dp_max * dt
        p1 = np.clip(p1, p1_prev - dp_max_step, p1_prev + dp_max_step)
        p2 = np.clip(p2, p2_prev - dp_max_step, p2_prev + dp_max_step)
        p1 = np.clip(p1, 0.0, p_max)
        p2 = np.clip(p2, 0.0, p_max)
        return p1, p2
    
    def cost_function(self, theta, theta_ref, p1, p2, p1_prev, p2_prev,
                      dp1, dp2, k, H, w_tracking, w_smooth, w_effort, w_constraint, p_max):
        """コスト関数"""
        err = theta_ref - theta
        cost = w_tracking * (err ** 2)
        
        if k == H - 1:
            cost += w_tracking * 0.5 * (err ** 2)
        
        cost += w_smooth * (dp1 ** 2 + dp2 ** 2)
        cost += w_effort * (p1 ** 2 + p2 ** 2)
        
        viol = 0.0
        if p1 < 0:
            viol += (-p1) ** 2
        if p2 < 0:
            viol += (-p2) ** 2
        if p1 > p_max:
            viol += (p1 - p_max) ** 2
        if p2 > p_max:
            viol += (p2 - p_max) ** 2
        
        cost += w_constraint * viol
        return cost
    
    def rollout_batch(self, theta0, p1_0, p2_0, U, dt, params):
        """バッチロールアウト"""
        K, H = U.shape[0], U.shape[1]
        
        theta_seq = np.zeros((K, H), dtype=np.float32)
        p1_seq = np.zeros((K, H), dtype=np.float32)
        p2_seq = np.zeros((K, H), dtype=np.float32)
        
        theta_k = np.full(K, theta0, dtype=np.float32)
        p1_k = np.full(K, p1_0, dtype=np.float32)
        p2_k = np.full(K, p2_0, dtype=np.float32)
        
        # 履歴の複製
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
                    p1_k[i], p2_k[i], p1_prev[i], p2_prev[i], dt,
                    params['p_max'], params['dp_max']
                )
            
            dp1_dt = (p1_k - p1_prev) / dt
            dp2_dt = (p2_k - p2_prev) / dt
            
            theta_hist = np.concatenate([theta_k[:, None], theta_hist[:, :-1]], axis=1)
            p1_hist = np.concatenate([p1_k[:, None], p1_hist[:, :-1]], axis=1)
            p2_hist = np.concatenate([p2_k[:, None], p2_hist[:, :-1]], axis=1)
            dp1_hist = np.concatenate([dp1_dt[:, None], dp1_hist[:, :-1]], axis=1)
            dp2_hist = np.concatenate([dp2_dt[:, None], dp2_hist[:, :-1]], axis=1)
            
            # 特徴量構築
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
    
    def mppi_step(self, params):
        """MPPIステップ"""
        with self.lock:
            theta = self.theta_rad
            theta_ref = self.theta_ref_rad
            p1_prev = self.p1_cmd
            p2_prev = self.p2_cmd
        
        is_safe, msg = self.safety.check(theta)
        if not is_safe:
            rospy.logerr(f"[MPPI] Safety: {msg}")
            self.emergency_stop = True
            self.publish_cmd(0.0, 0.0)
            return
        
        K = params['K']
        H = params['horizon']
        dt = params['dt']
        
        U = np.random.normal(loc=0.0, scale=params['sigma_u'],
                            size=(K, H, 2)).astype(np.float32)
        
        theta_seq, p1_seq, p2_seq = self.rollout_batch(theta, p1_prev, p2_prev, U, dt, params)
        
        J = np.zeros(K, dtype=np.float32)
        for i in range(K):
            cost = 0.0
            p1_h, p2_h = p1_prev, p2_prev
            for h in range(H):
                dp1, dp2 = U[i, h, 0], U[i, h, 1]
                cost += self.cost_function(
                    theta_seq[i, h], theta_ref,
                    p1_seq[i, h], p2_seq[i, h],
                    p1_h, p2_h, dp1, dp2, h, H,
                    params['w_tracking'], params['w_smooth'],
                    params['w_effort'], params['w_constraint'],
                    params['p_max']
                )
                p1_h, p2_h = p1_seq[i, h], p2_seq[i, h]
            J[i] = cost
        
        beta = np.min(J)
        w = np.exp(-(J - beta) / max(1e-6, params['temperature']))
        w_sum = np.sum(w) + 1e-9
        
        dU = np.sum(w[:, None, None] * U, axis=0) / w_sum
        
        dp1_cmd, dp2_cmd = dU[0, 0], dU[0, 1]
        p1_cmd = p1_prev + dp1_cmd
        p2_cmd = p2_prev + dp2_cmd
        
        p1_cmd, p2_cmd = self.enforce_constraints(
            p1_cmd, p2_cmd, p1_prev, p2_prev, dt,
            params['p_max'], params['dp_max']
        )
        
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
    
    def publish_cmd(self, p1, p2):
        """圧力指令を出力"""
        msg = Quaternion()
        msg.x = float(p1) * 4096.0 / 0.9
        msg.y = float(p2) * 4096.0 / 0.9
        msg.z = 0.0
        msg.w = 0.0
        self.pub_cmd.publish(msg)
    
    # ========== Trajectory Execution ==========
    
    def execute_trajectory(self, t_arr, theta_arr, params, duration):
        """軌道を実行"""
        rate = rospy.Rate(params['rate_hz'])
        self.start_time = rospy.get_time()
        
        start = time.time()
        idx = 0
        frame_count = 0
        
        rospy.loginfo(f"[Tuner] Executing trajectory ({duration:.1f}s)...")
        
        while not rospy.is_shutdown() and (time.time() - start) < duration:
            if self.emergency_stop:
                self.publish_cmd(0.0, 0.0)
                rospy.logerr("[Tuner] Emergency stop!")
                break
            
            elapsed = time.time() - start
            
            # 目標更新
            while idx < len(t_arr) - 1 and t_arr[idx] < elapsed:
                idx += 1
            target_deg = theta_arr[min(idx, len(theta_arr) - 1)]
            
            self.current_target_deg = target_deg
            self.theta_ref_rad = math.radians(target_deg)
            
            # 制御更新
            if frame_count % params['frame_skip'] == 0:
                self.mppi_step(params)
            
            frame_count += 1
            rate.sleep()
        
        rospy.loginfo("[Tuner] Trajectory execution complete")
    
    # ========== Evaluation ==========
    
    def evaluate_performance(self):
        """性能評価"""
        if len(self.time_data) < 10:
            return {
                'rmse': float('inf'),
                'max_error': float('inf'),
                'mean_error': float('inf'),
                'settling_ratio': 0.0,
                'success': False
            }
        
        theta = np.array(list(self.theta_data))
        target = np.array(list(self.target_data))
        
        error = target - theta
        rmse = np.sqrt(np.mean(error**2))
        max_error = np.max(np.abs(error))
        mean_error = np.mean(np.abs(error))
        
        settled = np.abs(error) < 2.0
        settling_ratio = np.sum(settled) / len(settled)
        
        rospy.loginfo(f"[Tuner] RMSE={rmse:.2f}°, Max={max_error:.2f}°, "
                     f"Mean={mean_error:.2f}°, Settled={settling_ratio*100:.1f}%")
        
        return {
            'rmse': float(rmse),
            'max_error': float(max_error),
            'mean_error': float(mean_error),
            'settling_ratio': float(settling_ratio),
            'success': True
        }
    
    def save_results(self, params, metrics, trajectory_type):
        """結果保存"""
        param_str = (f"K{params['K']}_H{params['horizon']}_"
                    f"sig{params['sigma_u']:.3f}_wt{params['w_tracking']:.1f}")
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        base_name = f"{trajectory_type}_{param_str}_{timestamp}"
        
        # CSV
        csv_path = os.path.join(self.output_dir, f"{base_name}.csv")
        df = pd.DataFrame({
            'time': list(self.time_data),
            'theta': list(self.theta_data),
            'target': list(self.target_data),
            'p1_cmd': list(self.p1_cmd_data),
            'p2_cmd': list(self.p2_cmd_data)
        })
        df['error'] = df['target'] - df['theta']
        df.to_csv(csv_path, index=False)
        
        # Plot
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        # Tracking
        axes[0].plot(df['time'], df['target'], 'r--', label='Target', linewidth=2.5, alpha=0.9)
        axes[0].plot(df['time'], df['theta'], 'b-', label='Actual', linewidth=1.5)
        axes[0].set_ylabel('Angle [deg]', fontsize=12, fontweight='bold')
        axes[0].legend(fontsize=11)
        axes[0].grid(True, alpha=0.3, linestyle='--')
        axes[0].set_title(f'{trajectory_type} - K={params["K"]}, H={params["horizon"]}, '
                         f'σ={params["sigma_u"]:.3f}, w_t={params["w_tracking"]:.1f}',
                         fontsize=13, fontweight='bold')
        
        # Error
        axes[1].plot(df['time'], df['error'], 'k-', linewidth=1.5)
        axes[1].axhline(y=0, color='r', linestyle='--', linewidth=1.5, alpha=0.7)
        axes[1].fill_between(df['time'], -2, 2, alpha=0.25, color='green', label='±2°')
        axes[1].set_ylabel('Error [deg]', fontsize=12, fontweight='bold')
        axes[1].legend(fontsize=10)
        axes[1].grid(True, alpha=0.3)
        axes[1].set_title(f'RMSE: {metrics["rmse"]:.2f}°, Max: {metrics["max_error"]:.2f}°',
                         fontsize=12)
        
        # Pressures
        axes[2].plot(df['time'], df['p1_cmd'], 'b-', label='P1', linewidth=1.5, alpha=0.8)
        axes[2].plot(df['time'], df['p2_cmd'], 'r-', label='P2', linewidth=1.5, alpha=0.8)
        axes[2].set_ylabel('Pressure [MPa]', fontsize=12, fontweight='bold')
        axes[2].set_xlabel('Time [s]', fontsize=12, fontweight='bold')
        axes[2].legend(fontsize=10)
        axes[2].grid(True, alpha=0.3)
        
        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, f"{base_name}.png")
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        # Metrics JSON
        with open(os.path.join(self.output_dir, f"{base_name}_metrics.json"), 'w') as f:
            json.dump({'parameters': params, 'metrics': metrics, 'timestamp': timestamp}, f, indent=2)
        
        rospy.loginfo(f"[Tuner] Saved: {base_name}")
        return metrics
    
    # ========== Main Tuning ==========
    
    def run_experiment(self, params, trajectory_type, traj_data, duration):
        """1実験実行"""
        rospy.loginfo(f"\n{'='*70}")
        rospy.loginfo(f"[Tuner] Experiment: {trajectory_type}")
        rospy.loginfo(f"  K={params['K']}, H={params['horizon']}, σ={params['sigma_u']:.3f}")
        rospy.loginfo(f"  w_track={params['w_tracking']}, w_smooth={params['w_smooth']}")
        rospy.loginfo(f"{'='*70}")
        
        # データクリア
        self.time_data.clear()
        self.theta_data.clear()
        self.target_data.clear()
        self.p1_cmd_data.clear()
        self.p2_cmd_data.clear()
        
        # 履歴リセット
        with self.lock:
            for _ in range(len(self.hist_theta)):
                self.hist_theta[_] = 0.0
            for _ in range(len(self.hist_p1_cmd)):
                self.hist_p1_cmd[_] = 0.0
            for _ in range(len(self.hist_p2_cmd)):
                self.hist_p2_cmd[_] = 0.0
            for _ in range(len(self.hist_dp1_dt)):
                self.hist_dp1_dt[_] = 0.0
            for _ in range(len(self.hist_dp2_dt)):
                self.hist_dp2_dt[_] = 0.0
            self.p1_cmd = 0.0
            self.p2_cmd = 0.0
        
        self.emergency_stop = False
        self.theta_filter.reset(0.0)
        self.safety = SafetyMonitor()
        
        # ウォームアップ
        rospy.loginfo("[Tuner] Warmup (3s)...")
        rate = rospy.Rate(params['rate_hz'])
        for _ in range(int(params['rate_hz'] * 3)):
            self.current_target_deg = 0.0
            self.theta_ref_rad = 0.0
            self.publish_cmd(0.0, 0.0)
            rate.sleep()
        
        # 実行
        t_arr, theta_arr = traj_data
        self.execute_trajectory(t_arr, theta_arr, params, duration)
        
        # クールダウン
        rospy.sleep(1.0)
        
        # 評価・保存
        metrics = self.evaluate_performance()
        self.save_results(params, metrics, trajectory_type)
        
        return metrics
    
    def tune_staged(self):
        """段階的チューニング"""
        base_params = {
            'rate_hz': 100,
            'frame_skip': 2,
            'dt': 2 / 100.0,
            'K': 16,
            'horizon': 14,
            'temperature': 3.0,
            'sigma_u': 0.12,
            'w_tracking': 50.0,
            'w_smooth': 0.35,
            'w_effort': 0.03,
            'w_constraint': 500.0,
            'p_max': 0.70,
            'dp_max': 3.5,
        }
        
        results = []
        
        # ========== Stage 1: K ==========
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("STAGE 1: Population Size (K)")
        rospy.loginfo("="*70)
        
        best_K, best_K_rmse = 32, float('inf')
        for K in [12, 14, 16, 18, 20, 24]:
            params = base_params.copy()
            params['K'] = K
            traj = TrajectoryGenerator.generate_stepwise([(-10, 12, 8), (10, 12, 8), (-5, 12, 8)])
            metrics = self.run_experiment(params, f"stage1_K{K}", traj, 45)
            results.append({'stage': 1, 'K': K, **metrics})
            if metrics['rmse'] < best_K_rmse:
                best_K_rmse, best_K = metrics['rmse'], K
            rospy.sleep(2.0)
        
        rospy.loginfo(f"\n✓ Best K: {best_K} (RMSE: {best_K_rmse:.2f}°)")
        base_params['K'] = best_K
        
        # ========== Stage 2: H ==========
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("STAGE 2: Horizon (H)")
        rospy.loginfo("="*70)
        
        best_H, best_H_rmse = 14, float('inf')
        for H in [10, 12, 14, 16, 18, 20]:
            params = base_params.copy()
            params['horizon'] = H
            traj = TrajectoryGenerator.generate_stepwise([(-15, 12, 8), (15, 12, 8), (0, 12, 8)])
            metrics = self.run_experiment(params, f"stage2_H{H}", traj, 45)
            results.append({'stage': 2, 'H': H, **metrics})
            if metrics['rmse'] < best_H_rmse:
                best_H_rmse, best_H = metrics['rmse'], H
            rospy.sleep(2.0)
        
        rospy.loginfo(f"\n✓ Best H: {best_H} (RMSE: {best_H_rmse:.2f}°)")
        base_params['horizon'] = best_H
        
        # ========== Stage 3: sigma ==========
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("STAGE 3: Control Noise (σ)")
        rospy.loginfo("="*70)
        
        best_sigma, best_sigma_rmse = 0.04, float('inf')
        for sigma in [0.08, 0.10, 0.12, 0.14, 0.16]:
            params = base_params.copy()
            params['sigma_u'] = sigma
            traj = TrajectoryGenerator.generate_stepwise([(-20, 12, 10), (20, 12, 10), (0, 12, 10), (-15, 12, 10)])
            metrics = self.run_experiment(params, f"stage3_sig{sigma:.3f}", traj, 65)
            results.append({'stage': 3, 'sigma_u': sigma, **metrics})
            if metrics['rmse'] < best_sigma_rmse:
                best_sigma_rmse, best_sigma = metrics['rmse'], sigma
            rospy.sleep(2.0)
        
        rospy.loginfo(f"\n✓ Best σ: {best_sigma:.3f} (RMSE: {best_sigma_rmse:.2f}°)")
        base_params['sigma_u'] = best_sigma
        
        # ========== Stage 4: Weights ==========
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("STAGE 4: Cost Weights")
        rospy.loginfo("="*70)
        
        weight_sets = [
            {'w_tracking': 45.0, 'w_smooth': 0.30, 'w_effort': 0.025},
            {'w_tracking': 50.0, 'w_smooth': 0.30, 'w_effort': 0.03},
            {'w_tracking': 50.0, 'w_smooth': 0.35, 'w_effort': 0.03},
            {'w_tracking': 50.0, 'w_smooth': 0.40, 'w_effort': 0.03},
            {'w_tracking': 55.0, 'w_smooth': 0.35, 'w_effort': 0.03},
            {'w_tracking': 55.0, 'w_smooth': 0.35, 'w_effort': 0.035},
        ]
        best_weights, best_w_rmse = weight_sets[1], float('inf')
        for weights in weight_sets:
            params = base_params.copy()
            params.update(weights)
            traj = TrajectoryGenerator.generate_stepwise([(-25, 15, 12), (25, 15, 12), (-15, 15, 12), (20, 15, 12)])
            w_str = f"wt{weights['w_tracking']:.0f}"
            metrics = self.run_experiment(params, f"stage4_{w_str}", traj, 75)
            results.append({'stage': 4, **weights, **metrics})
            if metrics['rmse'] < best_w_rmse:
                best_w_rmse, best_weights = metrics['rmse'], weights
            rospy.sleep(2.0)
        
        rospy.loginfo(f"\n✓ Best weights: {best_weights} (RMSE: {best_w_rmse:.2f}°)")
        base_params.update(best_weights)
        
        # ========== Final ==========
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("FINAL EVALUATION")
        rospy.loginfo("="*70)
        
        # Stepwise
        traj1 = TrajectoryGenerator.generate_stepwise([(-10, 15, 15), (20, 15, 15), (-10, 15, 15), (15, 15, 15), (-20, 15, 15), (25, 15, 15),(0, 15, 15)])
        m1 = self.run_experiment(base_params, "final_stepwise", traj1, 150)
        results.append({'stage': 'final_stepwise', **m1})
        rospy.sleep(3.0)
        
        # Continuous
        traj2 = TrajectoryGenerator.generate_continuous([(-15, 12), (25, 12), (-20, 12), (15, 12), (-25, 12), (20, 12), (-10, 12), (0 ,12)])
        m2 = self.run_experiment(base_params, "final_continuous", traj2, 100)
        results.append({'stage': 'final_continuous', **m2})

        # 追加：ランダムステップ軌道（より実用的）
        rospy.loginfo("\n[Tuner] Test 3: Random step trajectory")
        random_waypoints = []
        for i in range(8):  # 8回のランダムステップ
            angle = np.random.uniform(-25, 25)
            trans_time = np.random.uniform(10, 15)
            hold_time = np.random.uniform(8, 12)
            random_waypoints.append((angle, trans_time, hold_time))
        traj3 = TrajectoryGenerator.generate_stepwise(random_waypoints)
        m3 = self.run_experiment(base_params, "final_random", traj3, 180)  # ← 180秒（3分）
        results.append({'stage': 'final_random', **m3})

        # Save summary
        with open(os.path.join(self.output_dir, "tuning_summary.json"), 'w') as f:
            json.dump({'best_parameters': base_params, 'all_results': results,
                      'timestamp': datetime.now().isoformat()}, f, indent=2)
        
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("✓ Tuning Complete!")
        rospy.loginfo(f"  Output: {self.output_dir}")
        rospy.loginfo("="*70)


# ==================== Main ====================

def main():
    model_dir = os.path.expanduser("~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/models/out_narx2")
    output_dir = os.path.expanduser("~/mppi_tuning_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    
    rospy.loginfo("="*70)
    rospy.loginfo("Integrated MPPI Tuning Script")
    rospy.loginfo("="*70)
    
    tuner = IntegratedMPPITuner(model_dir, output_dir)
    tuner.tune_staged()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error: {e}")
        import traceback
        traceback.print_exc()