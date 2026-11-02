#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MPPI 追従性能テスト＆可視化

- 様々な目標値に対する追従をテスト
- リアルタイムに近いログ取得
- 性能指標の自動計算
- MPPI コントローラの /theta_target_deg, /mpa_cmd をそのまま利用

Usage:
  rosrun kinikun mppi_tracking_performance_test.py \
      _test_name:=mppi_step_test_001
"""
import rospy
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3
import time
import os


class TrackingPerformanceTestMPPI:
    def __init__(self):
        self.test_name = rospy.get_param("~test_name", "mppi_tracking_test")
        self.rate_hz = rospy.get_param("~rate_hz", 50.0)

        # テストシーケンス（目標角度[deg]と保持時間[s]）
        self.test_sequence = [
            (0.0,   3.0),
            (15.0,  5.0),
            (25.0,  5.0),
            (10.0,  5.0),
            (-10.0, 5.0),
            (-25.0, 5.0),
            (-15.0, 5.0),
            (0.0,   5.0),
            (20.0,  5.0),
            (0.0,   3.0),
        ]

        # データ記録
        self.log_data = []
        self.start_time = None

        # 現在の状態
        self.theta_cur_deg = 0.0
        self.theta_target_deg = 0.0
        self.p1_cmd_MPa = 0.0
        self.p2_cmd_MPa = 0.0

        # Publishers
        self.pub_target = rospy.Publisher("/theta_target_deg", Float32, queue_size=1)

        # ★ 制御側のログON/OFF用（narx_mppi_controlX.py 側で use_external_log_enable:=true のとき有効）
        self.pub_log_enable = rospy.Publisher("/pid_log_enable", Bool, queue_size=1)

        # Subscribers
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.cb_joint)
        rospy.Subscriber("/theta_target_deg", Float32, self.cb_target)
        rospy.Subscriber("/mpa_cmd", Vector3, self.cb_cmd)

        rospy.loginfo("MPPI Tracking Performance Test initialized")
        rospy.loginfo("Test sequence: %d steps, total time: %.1f s",
                      len(self.test_sequence),
                      sum([t for _, t in self.test_sequence]))

    # ===== Callbacks =====

    def cb_joint(self, msg):
        """joint_states から arm1_joint (index 2) の角度[deg]を取得"""
        try:
            self.theta_cur_deg = np.degrees(float(msg.position[2]))
        except Exception:
            pass

    def cb_target(self, msg):
        """現在の目標角度[deg]を取得（参考用にログする）"""
        self.theta_target_deg = float(msg.data)

    def cb_cmd(self, msg):
        """MPPI からの /mpa_cmd を MPa に戻す

        制御側 publish_cmd:
            msg.x = p1_MPa * 4096 / 0.9
            msg.y = p2_MPa * 4096 / 0.9
        なので、その逆変換。
        """
        scale = 0.9 / 4096.0
        self.p1_cmd_MPa = float(msg.x) * scale
        self.p2_cmd_MPa = float(msg.y) * scale

    # ===== Test main =====

    def run_test(self):
        """ステップシーケンスを実行しつつログを取る"""
        rospy.loginfo("\n" + "=" * 70)
        rospy.loginfo("  MPPI Tracking Performance Test")
        rospy.loginfo("=" * 70)

        # 少し待ってからログ有効化（外部トリガ）
        rospy.sleep(1.0)
        self.pub_log_enable.publish(Bool(True))
        rospy.sleep(0.5)

        self.start_time = rospy.get_time()
        rate = rospy.Rate(self.rate_hz)

        for i, (target_deg, duration) in enumerate(self.test_sequence):
            rospy.loginfo(
                f"\n[Step {i+1}/{len(self.test_sequence)}] "
                f"Target: {target_deg:.1f}° for {duration:.1f}s"
            )

            # 目標角度を publish
            self.pub_target.publish(Float32(target_deg))

            t_start = rospy.get_time()
            while (rospy.get_time() - t_start) < duration and not rospy.is_shutdown():
                t = rospy.get_time() - self.start_time
                error = self.theta_target_deg - self.theta_cur_deg

                self.log_data.append({
                    'time': t,
                    'target_deg': self.theta_target_deg,
                    'actual_deg': self.theta_cur_deg,
                    'error_deg': error,
                    'p1_cmd_MPa': self.p1_cmd_MPa,
                    'p2_cmd_MPa': self.p2_cmd_MPa,
                    'ps_cmd_MPa': self.p1_cmd_MPa + self.p2_cmd_MPa,
                    'pd_cmd_MPa': self.p1_cmd_MPa - self.p2_cmd_MPa,
                })

                rate.sleep()

        # ログ無効化（外部トリガ）
        rospy.sleep(0.5)
        self.pub_log_enable.publish(Bool(False))

        rospy.loginfo("\n" + "=" * 70)
        rospy.loginfo("  Test complete! Analyzing results...")
        rospy.loginfo("=" * 70 + "\n")

        self.save_and_plot_results()

    # ===== Post-processing =====

    def save_and_plot_results(self):
        if len(self.log_data) == 0:
            rospy.logwarn("No data recorded!")
            return

        df = pd.DataFrame(self.log_data)

        # CSV 保存
        output_dir = os.path.expanduser(
            "~/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/mppi/results"
        )
        os.makedirs(output_dir, exist_ok=True)

        ts = int(time.time())
        csv_path = os.path.join(output_dir, f"{self.test_name}_{ts}.csv")
        df.to_csv(csv_path, index=False)
        rospy.loginfo(f"Saved data to: {csv_path}")

        # 性能指標
        self.compute_metrics(df)

        # プロット生成
        self.plot_tracking_performance(df, output_dir, ts)

    def compute_metrics(self, df):
        error = df['error_deg'].values

        rospy.loginfo("\n=== Performance Metrics (MPPI) ===")
        rospy.loginfo(f"  RMSE:       {np.sqrt(np.mean(error**2)):.3f}°")
        rospy.loginfo(f"  MAE:        {np.mean(np.abs(error)):.3f}°")
        rospy.loginfo(f"  Max Error:  {np.max(np.abs(error)):.3f}°")
        rospy.loginfo(f"  Std Dev:    {np.std(error):.3f}°")

        # 整定性能（最後の30%）
        tail_idx = int(len(error) * 0.7)
        tail_error = error[tail_idx:]
        rospy.loginfo(
            f"  Steady-state error (last 30%): {np.mean(np.abs(tail_error)):.3f}°"
        )

        # 大誤差の割合
        large_error_ratio = np.sum(np.abs(error) > 10.0) / len(error) * 100.0
        rospy.loginfo(f"  Large error ratio (>10°): {large_error_ratio:.1f}%\n")

    def plot_tracking_performance(self, df, output_dir, ts):
        t = df['time'].values
        target = df['target_deg'].values
        actual = df['actual_deg'].values
        error = df['error_deg'].values

        fig = plt.figure(figsize=(16, 10))
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)

        # 1. Target vs Actual
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(t, target, 'r--', linewidth=2, label='Target', alpha=0.8)
        ax1.plot(t, actual, 'b-', linewidth=1.5, label='Actual')
        ax1.fill_between(t, target - 2, target + 2, alpha=0.2,
                         color='green', label='±2° tolerance')
        ax1.set_xlabel('Time [s]', fontsize=12)
        ax1.set_ylabel('Angle [deg]', fontsize=12)
        ax1.set_title('MPPI Tracking: Target vs Actual',
                      fontsize=14, fontweight='bold')
        ax1.legend(loc='upper right', fontsize=11)
        ax1.grid(True, alpha=0.3)

        # 2. Error vs Time
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.plot(t, error, 'r-', linewidth=1)
        ax2.axhline(0, color='k', linestyle='-', linewidth=0.5)
        ax2.axhline(2, color='g', linestyle='--', alpha=0.5)
        ax2.axhline(-2, color='g', linestyle='--', alpha=0.5)
        ax2.fill_between(t, -2, 2, alpha=0.1, color='green')
        ax2.set_xlabel('Time [s]', fontsize=11)
        ax2.set_ylabel('Error [deg]', fontsize=11)
        ax2.set_title('Tracking Error', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)

        # 3. Error histogram
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.hist(error, bins=50, alpha=0.7, edgecolor='black')
        ax3.axvline(0, color='r', linestyle='--', linewidth=2)
        ax3.axvline(np.mean(error), color='orange', linestyle='--', linewidth=2,
                    label=f'Mean: {np.mean(error):.2f}°')
        ax3.set_xlabel('Error [deg]', fontsize=11)
        ax3.set_ylabel('Count', fontsize=11)
        ax3.set_title('Error Distribution', fontsize=12, fontweight='bold')
        ax3.legend(fontsize=10)
        ax3.grid(True, alpha=0.3, axis='y')

        # 4. Pressure commands
        ax4 = fig.add_subplot(gs[2, :])
        ax4.plot(t, df['p1_cmd_MPa'].values, linewidth=1.5, label='p1_cmd [MPa]', alpha=0.8)
        ax4.plot(t, df['p2_cmd_MPa'].values, linewidth=1.5, label='p2_cmd [MPa]', alpha=0.8)
        ax4.plot(t, df['ps_cmd_MPa'].values, '--', linewidth=1, label='p_sum_cmd [MPa]', alpha=0.6)
        ax4.set_xlabel('Time [s]', fontsize=12)
        ax4.set_ylabel('Pressure [MPa]', fontsize=12)
        ax4.set_title('Pressure Commands', fontsize=12, fontweight='bold')
        ax4.legend(loc='upper right', fontsize=11)
        ax4.grid(True, alpha=0.3)

        # Stats box
        stats_text = (
            f"RMSE: {np.sqrt(np.mean(error**2)):.2f}°\n"
            f"MAE:  {np.mean(np.abs(error)):.2f}°\n"
            f"Max:  {np.max(np.abs(error)):.2f}°\n"
            f"Std:  {np.std(error):.2f}°"
        )
        ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
                 fontsize=11, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        plt.suptitle(f'MPPI Tracking Test - {self.test_name}',
                     fontsize=16, fontweight='bold')

        plot_path = os.path.join(output_dir, f"{self.test_name}_{ts}.png")
        plt.savefig(plot_path, dpi=150, bbox_inches='tight')
        rospy.loginfo(f"Saved plot to: {plot_path}")

        try:
            plt.show(block=False)
            rospy.loginfo("Plot displayed (close window to continue)")
        except Exception:
            rospy.loginfo("Could not display plot (no display available)")

        plt.close()


def main():
    rospy.init_node("mppi_tracking_performance_test")

    tester = TrackingPerformanceTestMPPI()

    rospy.loginfo("\nStarting MPPI test in 3 seconds...")
    rospy.sleep(3.0)

    try:
        tester.run_test()
        rospy.loginfo("\n✓ MPPI test completed successfully!")
    except KeyboardInterrupt:
        rospy.loginfo("\nTest interrupted by user")
    except Exception as e:
        rospy.logerr(f"\nTest failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
