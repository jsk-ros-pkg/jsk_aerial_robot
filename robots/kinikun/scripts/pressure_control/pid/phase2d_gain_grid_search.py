#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 2D: PIDゲインのグリッドサーチ最適化
実機で複数のゲイン組み合わせをテストし、最適値を探索

使い方:
1. roslaunch でPIDコントローラを起動（dynamic_reconfigure対応版が望ましい）
2. このスクリプトを実行してゲインを自動変更しながらテスト
3. 各ゲインセットの性能指標を記録
4. 最適ゲインを出力
"""
import rospy
import numpy as np
import pandas as pd
import subprocess
import time
from std_msgs.msg import Float32, Bool

class GainGridSearch:
    def __init__(self):
        # 探索範囲の定義
        self.Kp_range = np.linspace(0.004, 0.012, 5)  # 5点
        self.Ki_range = np.linspace(0.001, 0.005, 4)  # 4点
        self.Kd_range = np.linspace(0.0002, 0.001, 4) # 4点
        
        # テスト目標（簡易版: 少数のステップ）
        self.test_targets = [0, 20, -20, 0]  # deg
        self.hold_time = 4.0  # seconds per step
        
        # 評価用の閾値
        self.settle_threshold = 2.0  # deg
        
        self.pub_target = rospy.Publisher("/theta_target_deg", Float32, queue_size=1)
        self.pub_log_enable = rospy.Publisher("/pid_log_enable", Bool, queue_size=1)
        
        self.results = []
        
        rospy.loginfo("Grid Search initialized")
        rospy.loginfo("Kp: %s", self.Kp_range)
        rospy.loginfo("Ki: %s", self.Ki_range)
        rospy.loginfo("Kd: %s", self.Kd_range)
        rospy.loginfo("Total combinations: %d", 
                      len(self.Kp_range) * len(self.Ki_range) * len(self.Kd_range))
    
    def set_gains(self, Kp, Ki, Kd):
        """
        ROSパラメータでゲインを変更
        注意: dynamic_reconfigure使用時はrosparam setの代わりに
        dynamic_reconfigure clientを使用すること
        """
        rospy.set_param("/pid_controller/Kp", float(Kp))
        rospy.set_param("/pid_controller/Ki", float(Ki))
        rospy.set_param("/pid_controller/Kd", float(Kd))
        rospy.loginfo("Set gains: Kp=%.4f Ki=%.4f Kd=%.4f", Kp, Ki, Kd)
        rospy.sleep(0.5)  # パラメータ反映待ち
    
    def run_single_test(self, Kp, Ki, Kd):
        """単一ゲインセットでテスト実行"""
        self.set_gains(Kp, Ki, Kd)
        
        # ログ開始
        self.pub_log_enable.publish(Bool(True))
        rospy.sleep(0.5)
        
        # ステップ応答テスト
        for target in self.test_targets:
            self.pub_target.publish(Float32(target))
            rospy.sleep(self.hold_time)
        
        # ログ停止
        rospy.sleep(0.5)
        self.pub_log_enable.publish(Bool(False))
        rospy.sleep(0.5)
        
        # ログファイルを取得（最新のもの）
        import glob
        log_files = glob.glob("/tmp/pid_log_*.csv")
        if not log_files:
            rospy.logwarn("No log file found!")
            return None
        
        latest_log = max(log_files, key=lambda x: int(x.split('_')[-1].replace('.csv', '')))
        
        # 性能解析
        try:
            df = pd.read_csv(latest_log)
            
            # 簡易評価指標
            error = df['error_deg'].values
            time_arr = df['time'].values
            dt = np.median(np.diff(time_arr))
            
            metrics = {
                'Kp': Kp,
                'Ki': Ki,
                'Kd': Kd,
                'rmse': np.sqrt(np.mean(error**2)),
                'mae': np.mean(np.abs(error)),
                'max_error': np.max(np.abs(error)),
                'IAE': np.sum(np.abs(error)) * dt,
                'ISE': np.sum(error**2) * dt,
                'ITAE': np.sum(time_arr * np.abs(error)) * dt,
                'overshoot_count': np.sum(np.abs(error) > 10.0),  # 10度以上の誤差
                'log_file': latest_log
            }
            
            rospy.loginfo("Test result: RMSE=%.2f, MAE=%.2f, IAE=%.2f",
                          metrics['rmse'], metrics['mae'], metrics['IAE'])
            
            return metrics
        
        except Exception as e:
            rospy.logerr("Failed to analyze log: %s", e)
            return None
    
    def run_grid_search(self):
        """全組み合わせをテスト"""
        rospy.sleep(2.0)
        
        total = len(self.Kp_range) * len(self.Ki_range) * len(self.Kd_range)
        count = 0
        
        for Kp in self.Kp_range:
            for Ki in self.Ki_range:
                for Kd in self.Kd_range:
                    count += 1
                    rospy.loginfo("=== Test %d/%d ===", count, total)
                    
                    metrics = self.run_single_test(Kp, Ki, Kd)
                    if metrics is not None:
                        self.results.append(metrics)
                    
                    # 安全のため少し待機
                    rospy.sleep(1.0)
        
        # 結果保存
        if self.results:
            df_results = pd.DataFrame(self.results)
            output_file = f"/tmp/gain_search_results_{int(time.time())}.csv"
            df_results.to_csv(output_file, index=False)
            rospy.loginfo("Results saved to: %s", output_file)
            
            # 最適ゲインの出力
            self.print_best_gains(df_results)
    
    def print_best_gains(self, df):
        """各評価指標での最適ゲインを表示"""
        print("\n=== Best Gains by Metric ===")
        
        metrics_to_minimize = ['rmse', 'mae', 'IAE', 'ISE', 'ITAE', 'max_error']
        
        for metric in metrics_to_minimize:
            if metric in df.columns:
                best_idx = df[metric].idxmin()
                best_row = df.loc[best_idx]
                print(f"\nBest by {metric} ({best_row[metric]:.3f}):")
                print(f"  Kp={best_row['Kp']:.4f}, Ki={best_row['Ki']:.4f}, Kd={best_row['Kd']:.4f}")
        
        # 複合指標（重み付き）
        # 重み: RMSE重視、IAE次、オーバーシュート考慮
        df['composite'] = (df['rmse'] * 1.0 + 
                          df['IAE'] * 0.1 + 
                          df['max_error'] * 0.2)
        
        best_idx = df['composite'].idxmin()
        best = df.loc[best_idx]
        print(f"\n*** Overall Best (Composite Score: {best['composite']:.3f}) ***")
        print(f"  Kp={best['Kp']:.4f}, Ki={best['Ki']:.4f}, Kd={best['Kd']:.4f}")
        print(f"  RMSE={best['rmse']:.2f}°, MAE={best['mae']:.2f}°, IAE={best['IAE']:.2f}")

def main():
    rospy.init_node("gain_grid_search")
    
    searcher = GainGridSearch()
    
    # ユーザー確認
    response = input(f"\nAbout to test {len(searcher.Kp_range) * len(searcher.Ki_range) * len(searcher.Kd_range)} gain combinations.\n"
                    f"Each test takes ~{len(searcher.test_targets) * searcher.hold_time:.0f}s.\n"
                    f"Total time: ~{len(searcher.Kp_range) * len(searcher.Ki_range) * len(searcher.Kd_range) * len(searcher.test_targets) * searcher.hold_time / 60:.0f} minutes.\n"
                    f"Continue? (y/n): ")
    
    if response.lower() == 'y':
        searcher.run_grid_search()
    else:
        rospy.loginfo("Search cancelled")

if __name__ == "__main__":
    main()