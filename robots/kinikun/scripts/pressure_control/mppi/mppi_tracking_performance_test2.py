#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
enhanced_tracking_test.py
拡張版トラッキング性能評価

多様な軌道パターンに対応：
- ステップ応答
- 正弦波追従
- チャープ信号
- 多周波数重畳
- ランダムウォーク
"""
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
from scipy import signal
import time
import os


class EnhancedTrackingTest:
    def __init__(self):
        self.test_name = rospy.get_param("~test_name", "enhanced_tracking_test")
        self.test_type = rospy.get_param("~test_type", "step")  # step, sine, chirp, multi_sine, random
        self.rate_hz = rospy.get_param("~rate_hz", 50.0)
        self.duration = rospy.get_param("~duration", 60.0)
        self.use_prediction = rospy.get_param("~use_prediction", False)
        
        # Test parameters
        self.setup_test_parameters()
        
        # Data recording
        self.log_data = []
        self.start_time = None
        
        # Current state
        self.theta_cur_deg = 0.0
        self.theta_target_deg = 0.0
        self.p1_cmd_MPa = 0.0
        self.p2_cmd_MPa = 0.0
        
        # Publishers
        self.pub_target = rospy.Publisher("/theta_target_deg", Float32, queue_size=1)
        self.pub_trajectory = rospy.Publisher("/theta_trajectory", Float32MultiArray, queue_size=1)
        self.pub_log_enable = rospy.Publisher("/pid_log_enable", Bool, queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.cb_joint)
        rospy.Subscriber("/theta_target_deg", Float32, self.cb_target)
        rospy.Subscriber("/mpa_cmd", Vector3, self.cb_cmd)
        
        rospy.loginfo(f"Enhanced Tracking Test: {self.test_type}")
    
    def setup_test_parameters(self):
        """テストタイプごとのパラメータ設定"""
        if self.test_type == "step":
            self.step_sequence = [
                (0.0, 5.0), (20.0, 5.0), (-20.0, 5.0), 
                (30.0, 5.0), (-30.0, 5.0), (0.0, 5.0)
            ]
            
        elif self.test_type == "sine":
            self.sine_amplitude = rospy.get_param("~sine_amplitude", 20.0)  # degrees
            self.sine_frequency = rospy.get_param("~sine_frequency", 0.2)   # Hz
            
        elif self.test_type == "chirp":
            self.chirp_f0 = rospy.get_param("~chirp_f0", 0.1)  # Hz
            self.chirp_f1 = rospy.get_param("~chirp_f1", 0.5)  # Hz
            self.chirp_amplitude = rospy.get_param("~chirp_amplitude", 15.0)  # degrees
            
        elif self.test_type == "multi_sine":
            self.frequencies = [0.1, 0.25, 0.4]  # Hz
            self.amplitudes = [10.0, 5.0, 3.0]   # degrees
            
        elif self.test_type == "random":
            self.random_bandwidth = rospy.get_param("~random_bandwidth", 0.5)  # Hz
            self.random_amplitude = rospy.get_param("~random_amplitude", 15.0)  # degrees
    
    def cb_joint(self, msg):
        try:
            self.theta_cur_deg = np.degrees(float(msg.position[2]))
        except Exception:
            pass
    
    def cb_target(self, msg):
        self.theta_target_deg = float(msg.data)
    
    def cb_cmd(self, msg):
        scale = 0.9 / 4096.0
        self.p1_cmd_MPa = float(msg.x) * scale
        self.p2_cmd_MPa = float(msg.y) * scale
    
    def generate_trajectory(self, t):
        """時刻tでの目標値と予測軌道を生成"""
        target = 0.0
        prediction = None
        
        if self.test_type == "step":
            # ステップ応答
            cumulative_time = 0
            for angle, duration in self.step_sequence:
                if t < cumulative_time + duration:
                    target = angle
                    if self.use_prediction:
                        # ステップの予測軌道（単純に維持）
                        prediction = np.full(30, angle)
                    break
                cumulative_time += duration
                
        elif self.test_type == "sine":
            # 正弦波
            target = self.sine_amplitude * np.sin(2 * np.pi * self.sine_frequency * t)
            if self.use_prediction:
                # 将来30ステップ分の正弦波
                dt = 1.0 / self.rate_hz
                future_t = t + np.arange(30) * dt
                prediction = self.sine_amplitude * np.sin(2 * np.pi * self.sine_frequency * future_t)
                
        elif self.test_type == "chirp":
            # チャープ信号
            target = self.chirp_amplitude * signal.chirp(t, self.chirp_f0, 
                                                         self.duration, self.chirp_f1)
            if self.use_prediction:
                dt = 1.0 / self.rate_hz
                future_t = t + np.arange(30) * dt
                prediction = self.chirp_amplitude * signal.chirp(
                    future_t, self.chirp_f0, self.duration, self.chirp_f1
                )
                
        elif self.test_type == "multi_sine":
            # 多周波数重畳
            target = sum(A * np.sin(2 * np.pi * f * t) 
                        for A, f in zip(self.amplitudes, self.frequencies))
            if self.use_prediction:
                dt = 1.0 / self.rate_hz
                future_t = t + np.arange(30) * dt
                prediction = np.array([
                    sum(A * np.sin(2 * np.pi * f * ft) 
                        for A, f in zip(self.amplitudes, self.frequencies))
                    for ft in future_t
                ])
                
        elif self.test_type == "random":
            # バンド制限ランダム信号
            np.random.seed(42)  # 再現性のため
            N = int(self.duration * self.rate_hz)
            noise = np.random.randn(N)
            # ローパスフィルタ
            b, a = signal.butter(2, self.random_bandwidth / (self.rate_hz / 2))
            filtered = signal.filtfilt(b, a, noise)
            # 正規化してスケーリング
            filtered = self.random_amplitude * filtered / np.std(filtered)
            idx = int(t * self.rate_hz) % N
            target = filtered[idx]
            
            if self.use_prediction:
                future_idx = np.arange(idx, idx + 30) % N
                prediction = filtered[future_idx]
        
        return target, prediction
    
    def run_test(self):
        """テスト実行"""
        rospy.loginfo("\n" + "=" * 70)
        rospy.loginfo(f"  Enhanced Tracking Test: {self.test_type.upper()}")
        rospy.loginfo("=" * 70)
        
        rospy.sleep(1.0)
        self.pub_log_enable.publish(Bool(True))
        rospy.sleep(0.5)
        
        self.start_time = rospy.get_time()
        rate = rospy.Rate(self.rate_hz)
        
        while (rospy.get_time() - self.start_time) < self.duration and not rospy.is_shutdown():
            t = rospy.get_time() - self.start_time
            
            # Generate trajectory
            target_deg, prediction = self.generate_trajectory(t)
            
            # Publish target
            self.pub_target.publish(Float32(target_deg))
            
            # Publish prediction if available
            if self.use_prediction and prediction is not None:
                traj_msg = Float32MultiArray()
                traj_msg.data = np.radians(prediction).tolist()
                self.pub_trajectory.publish(traj_msg)
            
            # Log data
            error = target_deg - self.theta_cur_deg
            self.log_data.append({
                'time': t,
                'target_deg': target_deg,
                'actual_deg': self.theta_cur_deg,
                'error_deg': error,
                'p1_cmd_MPa': self.p1_cmd_MPa,
                'p2_cmd_MPa': self.p2_cmd_MPa,
                'ps_cmd_MPa': self.p1_cmd_MPa + self.p2_cmd_MPa,
                'pd_cmd_MPa': self.p1_cmd_MPa - self.p2_cmd_MPa,
            })
            
            # Progress report
            if int(t) % 10 == 0 and abs(t - int(t)) < 0.1:
                rospy.loginfo(f"[{int(t):3d}s] Target: {target_deg:6.2f}°, "
                            f"Actual: {self.theta_cur_deg:6.2f}°, "
                            f"Error: {error:6.2f}°")
            
            rate.sleep()
        
        rospy.sleep(0.5)
        self.pub_log_enable.publish(Bool(False))
        
        rospy.loginfo("\n" + "=" * 70)
        rospy.loginfo("  Test complete! Analyzing results...")
        rospy.loginfo("=" * 70 + "\n")
        
        self.save_and_plot_results()
    
    def compute_metrics(self, df):
        """性能指標の計算"""
        error = df['error_deg'].values
        
        metrics = {
            'RMSE': np.sqrt(np.mean(error**2)),
            'MAE': np.mean(np.abs(error)),
            'Max_Error': np.max(np.abs(error)),
            'Std_Dev': np.std(error),
            'Mean_Error': np.mean(error)
        }
        
        # 周波数応答解析（sine/chirpの場合）
        if self.test_type in ['sine', 'chirp', 'multi_sine']:
            # FFT解析
            from scipy.fft import fft, fftfreq
            N = len(error)
            dt = df['time'].iloc[1] - df['time'].iloc[0]
            
            yf = fft(error)
            xf = fftfreq(N, dt)[:N//2]
            
            power = 2.0/N * np.abs(yf[0:N//2])
            peak_freq_idx = np.argmax(power[1:]) + 1
            metrics['Peak_Error_Freq'] = xf[peak_freq_idx]
            metrics['Peak_Error_Power'] = power[peak_freq_idx]
        
        # 整定時間（stepの場合）
        if self.test_type == 'step':
            # 各ステップごとの整定時間を計算
            settling_times = []
            threshold = 0.05  # 5%以内
            # ここは簡略化。実際のステップ変化点を検出して計算する必要がある
        
        rospy.loginfo("\n=== Performance Metrics ===")
        for key, value in metrics.items():
            rospy.loginfo(f"  {key:15s}: {value:10.3f}")
        
        return metrics
    
    def plot_results(self, df, output_dir, ts):
        """結果のプロット"""
        t = df['time'].values
        target = df['target_deg'].values
        actual = df['actual_deg'].values
        error = df['error_deg'].values
        
        # Create figure
        fig = plt.figure(figsize=(16, 12))
        
        # Layout based on test type
        if self.test_type in ['sine', 'chirp', 'multi_sine']:
            gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        else:
            gs = fig.add_gridspec(3, 1, hspace=0.3)
        
        # 1. Tracking performance
        ax1 = fig.add_subplot(gs[0, :] if self.test_type in ['sine', 'chirp', 'multi_sine'] 
                              else gs[0])
        ax1.plot(t, target, 'r--', linewidth=2, label='Target', alpha=0.8)
        ax1.plot(t, actual, 'b-', linewidth=1.5, label='Actual')
        ax1.fill_between(t, target - 2, target + 2, alpha=0.2, 
                         color='green', label='±2° tolerance')
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Angle [deg]')
        ax1.set_title(f'{self.test_type.upper()} Tracking Performance')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Error
        ax2 = fig.add_subplot(gs[1, :] if self.test_type in ['sine', 'chirp', 'multi_sine'] 
                              else gs[1])
        ax2.plot(t, error, 'r-', linewidth=1)
        ax2.axhline(0, color='k', linestyle='-', linewidth=0.5)
        ax2.fill_between(t, -2, 2, alpha=0.1, color='green')
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Error [deg]')
        ax2.set_title('Tracking Error')
        ax2.grid(True, alpha=0.3)
        
        # 3. Control inputs
        ax3 = fig.add_subplot(gs[2, :] if self.test_type not in ['sine', 'chirp', 'multi_sine']
                              else gs[2, 0])
        ax3.plot(t, df['p1_cmd_MPa'].values, label='p1_cmd')
        ax3.plot(t, df['p2_cmd_MPa'].values, label='p2_cmd')
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Pressure [MPa]')
        ax3.set_title('Control Inputs')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. Frequency analysis (if applicable)
        if self.test_type in ['sine', 'chirp', 'multi_sine']:
            from scipy.fft import fft, fftfreq
            ax4 = fig.add_subplot(gs[2, 1])
            
            N = len(error)
            dt_sample = t[1] - t[0]
            yf = fft(error)
            xf = fftfreq(N, dt_sample)[:N//2]
            power = 2.0/N * np.abs(yf[0:N//2])
            
            ax4.semilogy(xf[1:], power[1:])
            ax4.set_xlabel('Frequency [Hz]')
            ax4.set_ylabel('Error Power')
            ax4.set_title('Error Frequency Spectrum')
            ax4.grid(True, alpha=0.3)
            ax4.set_xlim([0, 1])
        
        plt.suptitle(f'Enhanced MPPI Tracking - {self.test_name}', fontsize=16)
        
        plot_path = os.path.join(output_dir, f"{self.test_name}_{self.test_type}_{ts}.png")
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        rospy.loginfo(f"Saved plot to: {plot_path}")
        
        try:
            plt.show(block=False)
        except:
            pass
        
        plt.close()
    
    def save_and_plot_results(self):
        """結果の保存と可視化"""
        if len(self.log_data) == 0:
            rospy.logwarn("No data recorded!")
            return
        
        df = pd.DataFrame(self.log_data)
        
        # Save directory
        output_dir = os.path.expanduser(
            "~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/mppi/results/1125"
        )
        os.makedirs(output_dir, exist_ok=True)
        
        # Save CSV
        ts = int(time.time())
        csv_path = os.path.join(output_dir, 
                               f"{self.test_name}_{self.test_type}_{ts}.csv")
        df.to_csv(csv_path, index=False)
        rospy.loginfo(f"Saved data to: {csv_path}")
        
        # Compute metrics
        metrics = self.compute_metrics(df)
        
        # Save metrics
        metrics_path = os.path.join(output_dir, 
                                   f"{self.test_name}_{self.test_type}_{ts}_metrics.json")
        import json
        with open(metrics_path, 'w') as f:
            json.dump(metrics, f, indent=2)
        
        # Plot results
        self.plot_results(df, output_dir, ts)


def main():
    rospy.init_node("enhanced_tracking_test")
    
    tester = EnhancedTrackingTest()
    
    rospy.loginfo("\nStarting enhanced tracking test in 3 seconds...")
    rospy.sleep(3.0)
    
    try:
        tester.run_test()
        rospy.loginfo("\n✓ Enhanced tracking test completed successfully!")
    except KeyboardInterrupt:
        rospy.loginfo("\nTest interrupted by user")
    except Exception as e:
        rospy.logerr(f"\nTest failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()