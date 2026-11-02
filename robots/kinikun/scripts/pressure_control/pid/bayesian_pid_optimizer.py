#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
完全版: ベイズ最適化によるPIDゲイン自動調整
- 実機テスト自動実行
- CSVログから性能指標を自動計算
- 安全機構付き（異常検出で中止）
- 進捗の可視化

必要なパッケージ:
  pip install scikit-optimize pandas numpy matplotlib

使い方:
  roscore を起動後、別ターミナルで:
  python3 bayesian_pid_optimizer_complete.py --n_calls 25 --dry_run
"""
import rospy
import numpy as np
import pandas as pd
import time
import argparse
import os
import glob
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState

try:
    from skopt import gp_minimize
    from skopt.space import Real
    from skopt.utils import use_named_args
    from skopt.plots import plot_convergence
    import matplotlib.pyplot as plt
    SKOPT_AVAILABLE = True
except ImportError:
    SKOPT_AVAILABLE = False
    print("ERROR: scikit-optimize not installed!")
    print("Install with: pip install scikit-optimize")
    exit(1)

class SafeBayesianOptimizer:
    def __init__(self, n_calls=25, dry_run=False, output_dir="bayesian_results"):
        self.n_calls = n_calls
        self.dry_run = dry_run
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # 探索空間（より広範囲かつ現実的な範囲）
        self.space = [
            Real(0.002, 3.520, name='Kp', prior='log-uniform'),
            Real(0.0001, 8.010, name='Ki', prior='log-uniform'),
            Real(0.0001, 1.102, name='Kd', prior='log-uniform')
        ]
        
        # テスト設定（短めで効率的）
        self.test_targets = [0, 15, 25, 15, -15, -25, 0]  # 7ステップ
        self.hold_time = 4.0  # 各目標で4秒保持
        self.total_test_time = len(self.test_targets) * self.hold_time + 5  # 約33秒
        
        # 安全設定
        self.max_angle_limit = 50.0  # deg
        self.emergency_stop_flag = False
        
        # ROS初期化（dry_runでない場合）
        if not self.dry_run:
            rospy.init_node("bayesian_pid_optimizer", anonymous=True)
            self.pub_target = rospy.Publisher("/theta_target_deg", Float32, queue_size=1)
            self.pub_log_enable = rospy.Publisher("/pid_log_enable", Bool, queue_size=1)
            
            # 安全監視用
            self.current_angle = 0.0
            rospy.Subscriber("/kinikun1/joint_states", JointState, self._cb_joint)
        
        self.iteration = 0
        self.history = []
        self.best_gains = None
        self.best_score = float('inf')
        
        print(f"\n{'='*70}")
        print(f" ベイズ最適化 PIDゲインチューニング")
        print(f"{'='*70}")
        print(f"探索回数: {n_calls}")
        print(f"テスト時間: 約{self.total_test_time:.0f}秒/回")
        print(f"合計推定時間: 約{n_calls * self.total_test_time / 60:.0f}分")
        print(f"出力ディレクトリ: {output_dir}")
        if dry_run:
            print(f"[DRY RUN MODE] 実機テストなし、シミュレーション")
        print(f"{'='*70}\n")
    
    def _cb_joint(self, msg):
        """安全監視用コールバック"""
        try:
            self.current_angle = np.degrees(float(msg.position[2]))
            if abs(self.current_angle) > self.max_angle_limit:
                if not self.emergency_stop_flag:
                    rospy.logerr(f"EMERGENCY: angle {self.current_angle:.1f}° exceeds limit!")
                    self.emergency_stop_flag = True
        except:
            pass
    
    def set_pid_gains(self, Kp, Ki, Kd):
        """PIDコントローラのゲインを設定"""
        if self.dry_run:
            return
        
        try:
            rospy.set_param("/pid_controller/Kp", float(Kp))
            rospy.set_param("/pid_controller/Ki", float(Ki))
            rospy.set_param("/pid_controller/Kd", float(Kd))
            rospy.sleep(0.5)  # パラメータ反映待ち
        except Exception as e:
            rospy.logerr(f"Failed to set gains: {e}")
    
    def run_test_sequence(self):
        """テストシーケンスを実行してログを取得"""
        if self.dry_run:
            # ダミーログ生成（dry run用）
            return self._generate_dummy_log()
        
        # ログ開始
        self.pub_log_enable.publish(Bool(True))
        rospy.sleep(0.5)
        
        # ステップ応答テスト
        for i, target in enumerate(self.test_targets):
            if self.emergency_stop_flag:
                rospy.logerr("Emergency stop triggered! Aborting test.")
                break
            
            self.pub_target.publish(Float32(target))
            rospy.loginfo(f"  Step {i+1}/{len(self.test_targets)}: {target:.0f}°")
            rospy.sleep(self.hold_time)
        
        # ログ停止
        rospy.sleep(0.5)
        self.pub_log_enable.publish(Bool(False))
        rospy.sleep(0.5)
        
        # 最新ログファイルを取得
        log_files = glob.glob("/home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/pid/logs/pid_log_*.csv")
        if not log_files:
            rospy.logwarn("No log file found!")
            return None
        
        latest_log = max(log_files, key=lambda x: int(x.split('_')[-1].replace('.csv', '')))
        return latest_log
    
    def _generate_dummy_log(self):
        """Dry run用のダミーログ生成"""
        dummy_path = os.path.join(self.output_dir, "dummy_log.csv")
        n_samples = 500
        t = np.linspace(0, 30, n_samples)
        
        # ダミーデータ生成
        target = np.zeros(n_samples)
        target[100:200] = 15
        target[200:300] = 25
        target[300:400] = -15
        
        actual = target + np.random.randn(n_samples) * 2.0  # ノイズ付加
        error = target - actual
        
        df = pd.DataFrame({
            'time': t,
            'target_deg': target,
            'actual_deg': actual,
            'error_deg': error,
            'p_term': np.random.randn(n_samples) * 0.1,
            'i_term': np.random.randn(n_samples) * 0.05,
            'd_term': np.random.randn(n_samples) * 0.01,
            'pd_cmd': np.random.randn(n_samples) * 0.2,
            'p1': 0.3 + np.random.randn(n_samples) * 0.05,
            'p2': 0.2 + np.random.randn(n_samples) * 0.05,
            'error_integral': np.cumsum(error) * 0.02
        })
        df.to_csv(dummy_path, index=False)
        return dummy_path
    
    def analyze_log(self, log_path):
        """ログから性能指標を計算"""
        try:
            df = pd.read_csv(log_path)
            
            if len(df) < 50:
                rospy.logwarn("Log too short!")
                return None
            
            error = df['error_deg'].values
            time_arr = df['time'].values
            target = df['target_deg'].values
            actual = df['actual_deg'].values
            
            dt = np.median(np.diff(time_arr))
            
            # 基本統計
            rmse = float(np.sqrt(np.mean(error**2)))
            mae = float(np.mean(np.abs(error)))
            max_error = float(np.max(np.abs(error)))
            
            # 積分誤差
            iae = float(np.sum(np.abs(error)) * dt)
            ise = float(np.sum(error**2) * dt)
            itae = float(np.sum(time_arr * np.abs(error)) * dt)
            
            # 大誤差カウント（10度以上）
            large_error_count = int(np.sum(np.abs(error) > 10.0))
            large_error_ratio = float(large_error_count / len(error))
            
            # 整定性評価（最後の20%の標準偏差）
            tail_start = int(len(error) * 0.8)
            settling_std = float(np.std(error[tail_start:]))
            
            # オーバーシュート検出（簡易）
            target_changes = np.where(np.abs(np.diff(target)) > 5.0)[0]
            overshoot_count = 0
            for tc in target_changes[:5]:  # 最初の5ステップで評価
                if tc + 50 < len(actual):
                    window = actual[tc:tc+50]
                    tgt = target[tc+25]
                    if tgt > 0:
                        overshoot = np.max(window) - tgt
                    else:
                        overshoot = tgt - np.min(window)
                    if overshoot > 5.0:  # 5度以上のオーバーシュート
                        overshoot_count += 1
            
            metrics = {
                'rmse': rmse,
                'mae': mae,
                'max_error': max_error,
                'iae': iae,
                'ise': ise,
                'itae': itae,
                'large_error_ratio': large_error_ratio,
                'settling_std': settling_std,
                'overshoot_count': overshoot_count,
                'n_samples': len(error)
            }
            
            return metrics
        
        except Exception as e:
            rospy.logerr(f"Failed to analyze log: {e}")
            return None
    
    def compute_score(self, metrics):
        """
        複合スコアを計算（最小化）
        - RMSE: 主要指標
        - 大誤差比率: 安定性
        - 整定偏差: 定常性能
        - オーバーシュート: 過渡応答
        """
        if metrics is None:
            return 1000.0  # ペナルティ
        
        # 重み付き複合スコア
        score = (
            metrics['rmse'] * 2.0 +           # RMSE重視
            metrics['mae'] * 1.0 +            # 平均誤差
            metrics['large_error_ratio'] * 50.0 +  # 大誤差ペナルティ
            metrics['settling_std'] * 3.0 +   # 整定偏差
            metrics['overshoot_count'] * 2.0  # オーバーシュート
        )
        
        # 制約違反にペナルティ
        if metrics['max_error'] > 40.0:
            score += 100.0
        
        if metrics['rmse'] > 15.0:
            score += 50.0
        
        return float(score)
    
    def evaluate_gains(self, gains):
        """
        ゲインセットを評価
        Returns: スコア（低いほど良い）
        """
        Kp, Ki, Kd = gains
        self.iteration += 1
        
        print(f"\n{'='*70}")
        print(f"反復 {self.iteration}/{self.n_calls}")
        print(f"{'='*70}")
        print(f"テストゲイン: Kp={Kp:.5f}, Ki={Ki:.5f}, Kd={Kd:.5f}")
        
        if not self.dry_run:
            # 緊急停止フラグリセット
            self.emergency_stop_flag = False
        
        # ゲイン設定
        self.set_pid_gains(Kp, Ki, Kd)
        
        # テスト実行
        print(f"実機テスト実行中... (約{self.total_test_time:.0f}秒)")
        log_path = self.run_test_sequence()
        
        if log_path is None or self.emergency_stop_flag:
            print("  → テスト失敗（ログなしまたは緊急停止）")
            score = 1000.0
            metrics = None
        else:
            # ログ解析
            metrics = self.analyze_log(log_path)
            score = self.compute_score(metrics)
            
            if metrics:
                print(f"  結果:")
                print(f"    RMSE:      {metrics['rmse']:.2f}°")
                print(f"    MAE:       {metrics['mae']:.2f}°")
                print(f"    Max Error: {metrics['max_error']:.2f}°")
                print(f"    IAE:       {metrics['iae']:.2f}")
                print(f"    大誤差率:   {metrics['large_error_ratio']*100:.1f}%")
                print(f"    整定偏差:   {metrics['settling_std']:.2f}°")
                print(f"  スコア: {score:.3f}")
            else:
                print("  → 解析失敗")
        
        # 履歴記録
        result = {
            'iteration': self.iteration,
            'Kp': Kp,
            'Ki': Ki,
            'Kd': Kd,
            'score': score
        }
        if metrics:
            result.update(metrics)
        
        self.history.append(result)
        
        # ベスト更新
        if score < self.best_score:
            self.best_score = score
            self.best_gains = (Kp, Ki, Kd)
            print(f"  ★ NEW BEST! (score={score:.3f})")
        
        # 中間結果保存
        if self.iteration % 5 == 0:
            self._save_intermediate_results()
        
        return score
    
    def _save_intermediate_results(self):
        """中間結果を保存"""
        df = pd.DataFrame(self.history)
        csv_path = os.path.join(self.output_dir, f"results_iter{self.iteration}.csv")
        df.to_csv(csv_path, index=False)
        print(f"  [中間保存] {csv_path}")
    
    def optimize(self):
        """ベイズ最適化を実行"""
        print("\n最適化開始...")
        print(f"予想終了時刻: {time.strftime('%H:%M:%S', time.localtime(time.time() + self.n_calls * self.total_test_time))}")
        
        @use_named_args(self.space)
        def objective(**params):
            gains = [params['Kp'], params['Ki'], params['Kd']]
            return self.evaluate_gains(gains)
        
        # 最適化実行
        result = gp_minimize(
            objective,
            self.space,
            n_calls=self.n_calls,
            random_state=42,
            n_initial_points=5,  # 最初の5回はランダム探索
            verbose=False
        )
        
        # 最終結果
        self.best_gains = tuple(result.x)
        self.best_score = result.fun
        
        self._print_final_results(result)
        self._save_final_results(result)
        self._plot_results(result)
        
        return result
    
    def _print_final_results(self, result):
        """最終結果を表示"""
        print(f"\n{'='*70}")
        print(f" 最適化完了")
        print(f"{'='*70}")
        print(f"\n最適ゲイン:")
        print(f"  Kp = {result.x[0]:.6f}")
        print(f"  Ki = {result.x[1]:.6f}")
        print(f"  Kd = {result.x[2]:.6f}")
        print(f"\n最良スコア: {result.fun:.3f}")
        
        # トップ5
        df = pd.DataFrame(self.history).sort_values('score')
        print(f"\nTop 5 結果:")
        print(df[['iteration', 'Kp', 'Ki', 'Kd', 'score', 'rmse', 'mae']].head(5).to_string(index=False))
        print(f"\n{'='*70}\n")
    
    def _save_final_results(self, result):
        """最終結果を保存"""
        # 全履歴CSV
        df = pd.DataFrame(self.history)
        csv_path = os.path.join(self.output_dir, "bayesian_optimization_results.csv")
        df.to_csv(csv_path, index=False)
        print(f"[保存] 全結果: {csv_path}")
        
        # 最適ゲインJSON
        import json
        best_gains_dict = {
            'Kp': float(result.x[0]),
            'Ki': float(result.x[1]),
            'Kd': float(result.x[2]),
            'score': float(result.fun),
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'n_iterations': len(self.history)
        }
        json_path = os.path.join(self.output_dir, "best_gains.json")
        with open(json_path, 'w') as f:
            json.dump(best_gains_dict, f, indent=2)
        print(f"[保存] 最適ゲイン: {json_path}")
    
    def _plot_results(self, result):
        """結果をプロット"""
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        
        df = pd.DataFrame(self.history)
        
        # 収束プロット
        ax = axes[0, 0]
        plot_convergence(result, ax=ax)
        ax.set_title('Convergence Plot')
        ax.set_ylabel('Score (lower is better)')
        
        # スコア履歴
        ax = axes[0, 1]
        ax.plot(df['iteration'].values, df['score'].values, 'b.-', alpha=0.7)
        ax.axhline(result.fun, color='r', linestyle='--', label=f'Best: {result.fun:.2f}')
        ax.set_xlabel('Iteration')
        ax.set_ylabel('Score')
        ax.set_title('Score History')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # ゲイン探索空間
        ax = axes[1, 0]
        scatter = ax.scatter(df['Kp'].values, df['Ki'].values, c=df['score'].values, 
                            cmap='viridis_r', s=100, alpha=0.6)
        ax.scatter(result.x[0], result.x[1], color='red', s=200, 
                  marker='*', edgecolors='black', linewidths=2, label='Best')
        ax.set_xlabel('Kp')
        ax.set_ylabel('Ki')
        ax.set_title('Search Space (Kp vs Ki)')
        ax.legend()
        plt.colorbar(scatter, ax=ax, label='Score')
        
        # RMSE分布
        ax = axes[1, 1]
        if 'rmse' in df.columns:
            rmse_vals = df['rmse'].dropna().values
            ax.hist(rmse_vals, bins=15, alpha=0.7, edgecolor='black')
            best_rmse = df.loc[df['score'].idxmin(), 'rmse']
            ax.axvline(best_rmse, 
                      color='r', linestyle='--', linewidth=2, label='Best')
            ax.set_xlabel('RMSE [deg]')
            ax.set_ylabel('Count')
            ax.set_title('RMSE Distribution')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plot_path = os.path.join(self.output_dir, 'optimization_summary.png')
        plt.savefig(plot_path, dpi=150)
        print(f"[保存] プロット: {plot_path}")
        plt.close()

def main():
    parser = argparse.ArgumentParser(description='PIDゲイン ベイズ最適化')
    parser.add_argument('--n_calls', type=int, default=20,
                       help='最適化の反復回数 (default: 20)')
    parser.add_argument('--dry_run', action='store_true',
                       help='ドライラン（実機なし、シミュレーション）')
    parser.add_argument('--output_dir', type=str, default='bayesian_results',
                       help='出力ディレクトリ (default: bayesian_results)')
    args = parser.parse_args()
    
    # 確認プロンプト
    if not args.dry_run:
        response = input(
            f"\n{'='*70}\n"
            f" 実機でベイズ最適化を開始します\n"
            f"{'='*70}\n"
            f"反復回数: {args.n_calls}\n"
            f"推定時間: 約{args.n_calls * 33 / 60:.0f}分\n"
            f"\n実行前に以下を確認してください:\n"
            f"  1. PIDコントローラが起動している\n"
            f"  2. ロボットが安全な初期位置にある\n"
            f"  3. 緊急停止ボタンが手の届く位置にある\n"
            f"\n続行しますか? (yes/no): "
        )
        if response.lower() not in ['yes', 'y']:
            print("中止しました")
            return
    
    # 最適化実行
    optimizer = SafeBayesianOptimizer(
        n_calls=args.n_calls,
        dry_run=args.dry_run,
        output_dir=args.output_dir
    )
    
    try:
        result = optimizer.optimize()
        print("\n最適化が正常に完了しました！")
        print(f"結果は {args.output_dir}/ に保存されています")
    except KeyboardInterrupt:
        print("\n\nユーザーによって中断されました")
        optimizer._save_intermediate_results()
        print(f"中間結果を保存しました: {args.output_dir}/")
    except Exception as e:
        print(f"\nエラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
        optimizer._save_intermediate_results()

if __name__ == "__main__":
    main()