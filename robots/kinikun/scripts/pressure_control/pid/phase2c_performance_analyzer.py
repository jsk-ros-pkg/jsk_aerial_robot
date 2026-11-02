#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 2C: ステップ応答の性能解析
CSVログから以下の指標を計算:
- 整定時間 (settling time)
- オーバーシュート (overshoot)
- 定常偏差 (steady-state error)
- IAE, ISE, ITAE (積分誤差指標)
- 立ち上がり時間 (rise time)
"""
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

def analyze_step_response(csv_path, settle_threshold=2.0, plot=True):
    """
    csv_path: PID制御のログCSV
    settle_threshold: 整定判定の閾値 [deg]
    """
    df = pd.read_csv(csv_path)
    
    # 目標変更点を検出（目標が変わったタイミング）
    target_diff = df['target_deg'].diff().abs()
    step_indices = np.where(target_diff > 1.0)[0]  # 1度以上の変化
    
    if len(step_indices) == 0:
        print("No step changes detected in data")
        return None
    
    results = []
    
    for i, start_idx in enumerate(step_indices):
        # 次のステップまで、またはデータ終端まで
        if i + 1 < len(step_indices):
            end_idx = step_indices[i + 1]
        else:
            end_idx = len(df)
        
        segment = df.iloc[start_idx:end_idx].copy()
        segment['time'] = segment['time'] - segment['time'].iloc[0]
        
        target = segment['target_deg'].iloc[10]  # 少し後の値を取る（ノイズ回避）
        initial = segment['actual_deg'].iloc[0]
        
        if abs(target - initial) < 1.0:
            continue  # 小さすぎるステップはスキップ
        
        error = segment['error_deg'].values
        time = segment['time'].values
        actual = segment['actual_deg'].values
        
        # 1. 整定時間 (Settling Time)
        settled_mask = np.abs(error) < settle_threshold
        if np.any(settled_mask):
            first_settled = np.where(settled_mask)[0][0]
            # その後ずっと範囲内かチェック
            if np.all(settled_mask[first_settled:]):
                settling_time = time[first_settled]
            else:
                settling_time = np.nan
        else:
            settling_time = np.nan
        
        # 2. オーバーシュート (Overshoot)
        step_direction = np.sign(target - initial)
        if step_direction > 0:
            peak_value = np.max(actual)
            overshoot = max(0, peak_value - target)
        else:
            peak_value = np.min(actual)
            overshoot = max(0, target - peak_value)
        
        overshoot_percent = (overshoot / abs(target - initial)) * 100 if abs(target - initial) > 0 else 0
        
        # 3. 立ち上がり時間 (Rise Time: 10% -> 90%)
        step_size = abs(target - initial)
        thresh_10 = initial + 0.1 * step_direction * step_size
        thresh_90 = initial + 0.9 * step_direction * step_size
        
        if step_direction > 0:
            t10 = time[np.where(actual >= thresh_10)[0][0]] if np.any(actual >= thresh_10) else np.nan
            t90 = time[np.where(actual >= thresh_90)[0][0]] if np.any(actual >= thresh_90) else np.nan
        else:
            t10 = time[np.where(actual <= thresh_10)[0][0]] if np.any(actual <= thresh_10) else np.nan
            t90 = time[np.where(actual <= thresh_90)[0][0]] if np.any(actual <= thresh_90) else np.nan
        
        rise_time = t90 - t10 if (not np.isnan(t10) and not np.isnan(t90)) else np.nan
        
        # 4. 定常偏差 (Steady-State Error)
        # 最後の1秒の平均
        last_1s = segment[segment['time'] > (segment['time'].max() - 1.0)]
        if len(last_1s) > 5:
            ss_error = last_1s['error_deg'].mean()
        else:
            ss_error = error[-1]
        
        # 5. 積分誤差指標
        dt = np.median(np.diff(time))
        IAE = np.sum(np.abs(error)) * dt
        ISE = np.sum(error**2) * dt
        ITAE = np.sum(time * np.abs(error)) * dt
        
        result = {
            'step': i,
            'initial_deg': initial,
            'target_deg': target,
            'step_size_deg': target - initial,
            'settling_time_s': settling_time,
            'overshoot_deg': overshoot,
            'overshoot_percent': overshoot_percent,
            'rise_time_s': rise_time,
            'ss_error_deg': ss_error,
            'IAE': IAE,
            'ISE': ISE,
            'ITAE': ITAE,
            'duration_s': time[-1]
        }
        results.append(result)
        
        # プロット（オプション）
        if plot:
            plt.figure(figsize=(10, 6))
            plt.subplot(2, 1, 1)
            plt.plot(time, actual, 'b-', label='Actual')
            plt.axhline(target, color='r', linestyle='--', label='Target')
            plt.axhline(target + settle_threshold, color='g', linestyle=':', alpha=0.5)
            plt.axhline(target - settle_threshold, color='g', linestyle=':', alpha=0.5)
            if not np.isnan(settling_time):
                plt.axvline(settling_time, color='orange', linestyle='--', label=f'Settling time: {settling_time:.2f}s')
            plt.ylabel('Angle [deg]')
            plt.legend()
            plt.title(f'Step {i}: {initial:.1f}° → {target:.1f}°')
            plt.grid(True)
            
            plt.subplot(2, 1, 2)
            plt.plot(time, error, 'r-')
            plt.axhline(settle_threshold, color='g', linestyle=':', alpha=0.5)
            plt.axhline(-settle_threshold, color='g', linestyle=':', alpha=0.5)
            plt.ylabel('Error [deg]')
            plt.xlabel('Time [s]')
            plt.grid(True)
            
            plt.tight_layout()
            plt.savefig(f'/home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/pid/step_{i}.png')
            plt.close()
    
    return pd.DataFrame(results)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv', help='PID log CSV file')
    parser.add_argument('--threshold', type=float, default=2.0, help='Settling threshold [deg]')
    parser.add_argument('--plot', action='store_true', help='Generate plots')
    args = parser.parse_args()
    
    results = analyze_step_response(args.csv, settle_threshold=args.threshold, plot=args.plot)
    
    if results is not None and len(results) > 0:
        print("\n=== Performance Metrics ===")
        print(results.to_string(index=False))
        
        print("\n=== Average Metrics ===")
        print(f"Avg Settling Time: {results['settling_time_s'].mean():.3f} s (±{results['settling_time_s'].std():.3f})")
        print(f"Avg Overshoot:     {results['overshoot_percent'].mean():.1f} % (±{results['overshoot_percent'].std():.1f})")
        print(f"Avg Rise Time:     {results['rise_time_s'].mean():.3f} s (±{results['rise_time_s'].std():.3f})")
        print(f"Avg SS Error:      {results['ss_error_deg'].mean():.2f} deg (±{results['ss_error_deg'].std():.2f})")
        print(f"Total IAE:         {results['IAE'].sum():.2f}")
        print(f"Total ITAE:        {results['ITAE'].sum():.2f}")
        
        # Save summary
        output_path = args.csv.replace('.csv', '_metrics.csv')
        results.to_csv(output_path, index=False)
        print(f"\nMetrics saved to: {output_path}")

if __name__ == "__main__":
    main()