#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze_mppi_log.py
MPPI制御ログの完全解析・可視化（修正版）
"""
import sys
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import json

class MPPILogAnalyzer:
    def __init__(self, log_path):
        self.log_path = log_path
        self.df = None
        self.segments = []
        
    def load(self):
        """CSVロード"""
        print("\n" + "="*70)
        print(" MPPI Control Log Analysis")
        print("="*70)
        print(f"\nLog file: {self.log_path}")
        
        self.df = pd.read_csv(self.log_path)
        
        # Convert to degrees
        self.df['theta_deg'] = np.degrees(self.df['theta'].values)
        self.df['theta_ref_deg'] = np.degrees(self.df['theta_ref'].values)
        self.df['error_deg'] = np.degrees(self.df['error'].values)
        
        print(f"Duration: {self.df['t'].iloc[-1] - self.df['t'].iloc[0]:.1f} seconds")
        print(f"Samples: {len(self.df)}")
        
        # Sampling rate
        dt = np.median(np.diff(self.df['t'].values))
        print(f"Sampling rate: {1.0/dt:.1f} Hz (dt={dt*1000:.1f}ms)")
        
    def overall_statistics(self):
        """全体統計"""
        print(f"\n{'='*70}")
        print(" Overall Statistics")
        print("="*70)
        
        # Tracking error
        print(f"\n[Tracking Performance]")
        print(f"  Mean error:    {self.df['error_deg'].mean():+.3f}° (std: {self.df['error_deg'].std():.3f}°)")
        print(f"  Max |error|:   {self.df['error_deg'].abs().max():.3f}°")
        print(f"  RMSE:          {np.sqrt((self.df['error_deg']**2).mean()):.3f}°")
        print(f"  MAE:           {self.df['error_deg'].abs().mean():.3f}°")
        
        # Percentiles
        percentiles = [50, 75, 90, 95, 99]
        print(f"\n[Error Percentiles]")
        for p in percentiles:
            val = np.percentile(np.abs(self.df['error_deg'].values), p)
            print(f"  {p:2d}th: {val:.3f}°")
        
        # Computation time
        print(f"\n[Computation Time]")
        print(f"  Mean:   {self.df['comp_time_ms'].mean():.2f} ms")
        print(f"  Std:    {self.df['comp_time_ms'].std():.2f} ms")
        print(f"  Max:    {self.df['comp_time_ms'].max():.2f} ms")
        print(f"  95th %: {self.df['comp_time_ms'].quantile(0.95):.2f} ms")
        
        # Check real-time constraint
        violations = (self.df['comp_time_ms'].values > 20).sum()
        print(f"  Violations (>20ms): {violations} / {len(self.df)} ({violations/len(self.df)*100:.2f}%)")
        
        # Pressure usage
        print(f"\n[Pressure Usage]")
        print(f"  p1_cmd: mean={self.df['p1_cmd'].mean():.3f} MPa, "
              f"max={self.df['p1_cmd'].max():.3f} MPa, "
              f"min={self.df['p1_cmd'].min():.3f} MPa")
        print(f"  p2_cmd: mean={self.df['p2_cmd'].mean():.3f} MPa, "
              f"max={self.df['p2_cmd'].max():.3f} MPa, "
              f"min={self.df['p2_cmd'].min():.3f} MPa")
        
        # Pressure rate
        dt = np.diff(self.df['t'].values)
        dp1_dt = np.abs(np.diff(self.df['p1_cmd'].values) / dt)
        dp2_dt = np.abs(np.diff(self.df['p2_cmd'].values) / dt)
        
        print(f"\n[Pressure Rate]")
        print(f"  |dp1/dt|: mean={np.mean(dp1_dt):.3f} MPa/s, max={np.max(dp1_dt):.3f} MPa/s")
        print(f"  |dp2/dt|: mean={np.mean(dp2_dt):.3f} MPa/s, max={np.max(dp2_dt):.3f} MPa/s")
        
        # Rate limit violations
        rate_limit = 3.5
        viol1 = (dp1_dt > rate_limit).sum()
        viol2 = (dp2_dt > rate_limit).sum()
        print(f"  Rate violations (>{rate_limit} MPa/s):")
        print(f"    p1: {viol1} / {len(dp1_dt)} ({viol1/len(dp1_dt)*100:.2f}%)")
        print(f"    p2: {viol2} / {len(dp2_dt)} ({viol2/len(dp2_dt)*100:.2f}%)")
        
        # MPPI cost
        print(f"\n[MPPI Optimization]")
        print(f"  J_min:  mean={self.df['J_min'].mean():.4f}, std={self.df['J_min'].std():.4f}")
        print(f"  J_mean: mean={self.df['J_mean'].mean():.4f}, std={self.df['J_mean'].std():.4f}")
    
    def detect_step_changes(self, threshold_deg=5.0):
        """ステップ変化を検出"""
        theta_ref_deg = self.df['theta_ref_deg'].values
        diff = np.abs(np.diff(theta_ref_deg))
        
        step_indices = np.where(diff > threshold_deg)[0] + 1
        
        segments = []
        for i in range(len(step_indices) - 1):
            start = step_indices[i]
            end = step_indices[i + 1]
            
            if end - start > 50:
                segments.append((start, end))
        
        # 最後のセグメント
        if len(step_indices) > 0:
            start = step_indices[-1]
            end = len(self.df)
            if end - start > 50:
                segments.append((start, end))
        
        return segments
    
    def analyze_step_response(self, start_idx, end_idx):
        """ステップ応答の性能指標を計算"""
        segment = self.df.iloc[start_idx:end_idx]
        
        # 目標値
        target = segment['theta_ref_deg'].iloc[-100:].mean()
        initial = segment['theta_deg'].iloc[0]
        
        theta_vals = segment['theta_deg'].values
        t_vals = segment['t'].values - segment['t'].iloc[0]
        
        # 立ち上がり時間（10%-90%）
        target_10 = initial + (target - initial) * 0.1
        target_90 = initial + (target - initial) * 0.9
        
        idx_10 = np.where((theta_vals - initial) >= (target_10 - initial))[0]
        idx_90 = np.where((theta_vals - initial) >= (target_90 - initial))[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            rise_time = t_vals[idx_90[0]] - t_vals[idx_10[0]]
        else:
            rise_time = np.nan
        
        # 整定時間（±2%以内）
        final_band = abs(target) * 0.02 if abs(target) > 0.1 else 0.1
        settled_mask = np.abs(segment['theta_deg'].values - target) <= final_band
        
        if np.any(settled_mask):
            settled_idx = np.where(settled_mask)[0][0]
            settling_time = t_vals[settled_idx]
        else:
            settling_time = np.nan
        
        # オーバーシュート
        if abs(target - initial) > 0.1:
            if target > initial:
                overshoot = (np.max(theta_vals) - target) / abs(target - initial) * 100
            else:
                overshoot = (target - np.min(theta_vals)) / abs(target - initial) * 100
        else:
            overshoot = 0.0
        
        # 定常偏差
        steady_state_error = segment['error_deg'].iloc[-100:].mean()
        
        # RMSE
        rmse = np.sqrt(np.mean(segment['error_deg'].values**2))
        
        return {
            'target': target,
            'initial': initial,
            'rise_time': rise_time,
            'settling_time': settling_time,
            'overshoot_pct': overshoot,
            'steady_state_error': steady_state_error,
            'rmse': rmse
        }
    
    def step_response_analysis(self):
        """ステップ応答解析"""
        self.segments = self.detect_step_changes(threshold_deg=5.0)
        
        if len(self.segments) == 0:
            print(f"\n[Step Response Analysis]")
            print("  No significant step changes detected.")
            return
        
        print(f"\n{'='*70}")
        print(" Step Response Analysis")
        print("="*70)
        print(f"\nDetected {len(self.segments)} step changes:\n")
        
        for i, (start, end) in enumerate(self.segments):
            metrics = self.analyze_step_response(start, end)
            
            print(f"Step {i+1}: {metrics['initial']:.1f}° → {metrics['target']:.1f}°")
            print(f"  Rise time:        {metrics['rise_time']:.3f} s")
            print(f"  Settling time:    {metrics['settling_time']:.3f} s")
            print(f"  Overshoot:        {metrics['overshoot_pct']:.2f} %")
            print(f"  Steady error:     {metrics['steady_state_error']:.3f}°")
            print(f"  RMSE:             {metrics['rmse']:.3f}°")
            print()
    
    def generate_plots(self):
        """詳細プロット生成（NumPy配列に変換）"""
        print(f"\n{'='*70}")
        print(" Generating Plots")
        print("="*70)
        
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(4, 2, hspace=0.3, wspace=0.3)
        
        # ★ NumPy配列に変換
        t = self.df['t'].values - self.df['t'].values[0]
        theta_deg = self.df['theta_deg'].values
        theta_ref_deg = self.df['theta_ref_deg'].values
        error_deg = self.df['error_deg'].values
        p1_cmd = self.df['p1_cmd'].values
        p2_cmd = self.df['p2_cmd'].values
        comp_time_ms = self.df['comp_time_ms'].values
        J_min = self.df['J_min'].values
        J_mean = self.df['J_mean'].values
        
        # Plot 1: Tracking performance
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(t, theta_ref_deg, 'b--', label='Reference', linewidth=2, alpha=0.7)
        ax1.plot(t, theta_deg, 'r-', label='Actual', linewidth=1.5)
        ax1.set_ylabel('Angle [°]', fontsize=11)
        ax1.set_title('Tracking Performance', fontsize=12, fontweight='bold')
        ax1.legend(fontsize=10)
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Tracking error
        ax2 = fig.add_subplot(gs[1, :])
        ax2.plot(t, error_deg, 'r-', linewidth=1.5)
        ax2.axhline(0, color='k', linestyle='--', alpha=0.3)
        ax2.fill_between(t, -2, 2, color='g', alpha=0.2, label='±2° (acceptable)')
        ax2.fill_between(t, -5, 5, color='y', alpha=0.1, label='±5°')
        ax2.set_ylabel('Error [°]', fontsize=11)
        ax2.set_title('Tracking Error', fontsize=12, fontweight='bold')
        ax2.legend(fontsize=9)
        ax2.grid(True, alpha=0.3)
        
        # Plot 3: Control inputs
        ax3 = fig.add_subplot(gs[2, :])
        ax3.plot(t, p1_cmd, 'b-', label='p1_cmd', linewidth=1.5, alpha=0.8)
        ax3.plot(t, p2_cmd, 'r-', label='p2_cmd', linewidth=1.5, alpha=0.8)
        ax3.axhline(0.7, color='k', linestyle='--', alpha=0.3, label='Limit (0.7 MPa)')
        ax3.set_ylabel('Pressure [MPa]', fontsize=11)
        ax3.set_title('Control Inputs', fontsize=12, fontweight='bold')
        ax3.legend(fontsize=10)
        ax3.grid(True, alpha=0.3)
        ax3.set_ylim([0, 0.75])
        
        # Plot 4: Computation time
        ax4 = fig.add_subplot(gs[3, 0])
        ax4.plot(t, comp_time_ms, 'g-', linewidth=1, alpha=0.7)
        ax4.axhline(comp_time_ms.mean(), color='b', linestyle='--',
                    label=f'Mean: {comp_time_ms.mean():.1f}ms')
        ax4.axhline(20, color='r', linestyle='--', alpha=0.5, label='Limit: 20ms (50Hz)')
        ax4.set_xlabel('Time [s]', fontsize=11)
        ax4.set_ylabel('Comp. Time [ms]', fontsize=11)
        ax4.set_title('Computation Time', fontsize=12, fontweight='bold')
        ax4.legend(fontsize=9)
        ax4.grid(True, alpha=0.3)
        
        # Plot 5: MPPI cost
        ax5 = fig.add_subplot(gs[3, 1])
        ax5.plot(t, J_min, 'b-', linewidth=1.5, alpha=0.7, label='J_min')
        ax5.plot(t, J_mean, 'r-', linewidth=1.5, alpha=0.7, label='J_mean')
        ax5.set_xlabel('Time [s]', fontsize=11)
        ax5.set_ylabel('Cost', fontsize=11)
        ax5.set_title('MPPI Cost Function', fontsize=12, fontweight='bold')
        ax5.legend(fontsize=10)
        ax5.grid(True, alpha=0.3)
        ax5.set_yscale('log')
        
        plt.suptitle(f'MPPI Control Analysis - {os.path.basename(self.log_path)}',
                     fontsize=14, fontweight='bold', y=0.995)
        
        # Save
        output_path = self.log_path.replace('.csv', '_analysis.png')
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"\n  Saved: {output_path}")
        
        # Show (optional)
        # plt.show()
    
    def generate_summary_json(self):
        """サマリーJSON生成"""
        summary = {
            'log_file': self.log_path,
            'duration_s': float(self.df['t'].iloc[-1] - self.df['t'].iloc[0]),
            'num_samples': int(len(self.df)),
            'tracking': {
                'rmse_deg': float(np.sqrt((self.df['error_deg'].values**2).mean())),
                'mae_deg': float(self.df['error_deg'].abs().mean()),
                'max_error_deg': float(self.df['error_deg'].abs().max()),
                'mean_error_deg': float(self.df['error_deg'].mean())
            },
            'computation': {
                'mean_ms': float(self.df['comp_time_ms'].mean()),
                'max_ms': float(self.df['comp_time_ms'].max()),
                'p95_ms': float(self.df['comp_time_ms'].quantile(0.95)),
                'violations_20ms': int((self.df['comp_time_ms'].values > 20).sum())
            },
            'pressure': {
                'p1_mean': float(self.df['p1_cmd'].mean()),
                'p1_max': float(self.df['p1_cmd'].max()),
                'p2_mean': float(self.df['p2_cmd'].mean()),
                'p2_max': float(self.df['p2_cmd'].max())
            },
            'step_responses': []
        }
        
        # Step responses
        for i, (start, end) in enumerate(self.segments):
            metrics = self.analyze_step_response(start, end)
            summary['step_responses'].append({
                'step_num': i + 1,
                'initial_deg': float(metrics['initial']),
                'target_deg': float(metrics['target']),
                'rise_time_s': float(metrics['rise_time']),
                'settling_time_s': float(metrics['settling_time']),
                'overshoot_pct': float(metrics['overshoot_pct']),
                'steady_error_deg': float(metrics['steady_state_error']),
                'rmse_deg': float(metrics['rmse'])
            })
        
        output_path = self.log_path.replace('.csv', '_summary.json')
        with open(output_path, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"  Saved: {output_path}")
    
    def run(self):
        """完全解析実行"""
        self.load()
        self.overall_statistics()
        self.step_response_analysis()
        self.generate_plots()
        self.generate_summary_json()
        
        print("\n" + "="*70)
        print(" Analysis Complete")
        print("="*70 + "\n")

def main():
    if len(sys.argv) < 2:
        print("Usage: python analyze_mppi_log.py <log_csv>")
        print("\nExample:")
        print("  python analyze_mppi_log.py logs/mppi_log.csv")
        sys.exit(1)
    
    analyzer = MPPILogAnalyzer(sys.argv[1])
    analyzer.run()

if __name__ == '__main__':
    main()