#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 2B: PID制御（動的パラメータ読み込み対応版）
- 積分項でオフセット除去
- 条件付き積分（conditional integration）
- ゲイン評価用の詳細ログ
- ★ 実行中のゲイン変更に対応（ベイズ最適化用）
"""
import rospy, math, time
import numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Bool

class PIDController:
    def __init__(self):
        # Parameters
        self.rate_hz = rospy.get_param("~rate", 50.0)
        self.Kp = rospy.get_param("~Kp", 1.225)
        self.Ki = rospy.get_param("~Ki", 3.010)
        self.Kd = rospy.get_param("~Kd", 0.041)
        
        self.p_bias = rospy.get_param("~p_bias", 0.35)
        self.p_max = rospy.get_param("~p_max", 0.55)
        self.p_min = rospy.get_param("~p_min", 0.15)
        
        # 積分項の制限
        self.i_max = rospy.get_param("~i_max", 0.1)  # MPa
        self.deadzone = rospy.get_param("~deadzone", 1.0)  # deg
        self.enable_integral = rospy.get_param("~enable_integral", True)
        
        # レート制限
        self.rate_limit = rospy.get_param("~rate_limit", 0.08)  # MPa/step
        
        # 安全
        self.safety_angle = rospy.get_param("~safety_angle", 70.0)  # deg
        
        self.cmd_gain = 4551.11
        self.cmd_clip = 4096.0
        self.dt = 1.0 / self.rate_hz
        
        # State
        self.theta_cur = 0.0
        self.theta_target = 0.0
        self.error_prev = 0.0
        self.error_integral = 0.0
        self.p1_prev = self.p_bias / 2
        self.p2_prev = self.p_bias / 2
        self.emergency_stop = False
        
        # Logging
        self.log_data = []
        self.log_enabled = False
        self.start_time = None
        
        # Publishers/Subscribers
        self.pub_cmd = rospy.Publisher("/mpa_cmd", Vector3, queue_size=1)
        self.pub_error = rospy.Publisher("/pid_error", Float32, queue_size=1)
        self.pub_control = rospy.Publisher("/pid_control", Vector3, queue_size=1)
        
        rospy.Subscriber("/kinikun1/joint_states", JointState, self.cb_joint)
        rospy.Subscriber("/theta_target_deg", Float32, self.cb_target)
        rospy.Subscriber("/pid_log_enable", Bool, self.cb_log_enable)
        
        rospy.loginfo("PID Controller initialized:")
        rospy.loginfo("  Kp=%.4f, Ki=%.4f, Kd=%.4f", self.Kp, self.Ki, self.Kd)
        rospy.loginfo("  i_max=%.3f, deadzone=%.1f deg", self.i_max, self.deadzone)
        rospy.loginfo("  rate=%.1f Hz", self.rate_hz)
        
        # ★ 動的パラメータ読み込みタイマー（1秒ごと）
        rospy.Timer(rospy.Duration(1.0), self._reload_params_callback)
        rospy.loginfo("  Dynamic parameter reload: ENABLED")
    
    def _reload_params_callback(self, event):
        """
        パラメータを定期的に再読み込み
        ベイズ最適化などで実行中にゲインを変更する場合に使用
        """
        try:
            # グローバルパラメータから読み込み
            new_Kp = rospy.get_param("/pid_controller/Kp", self.Kp)
            new_Ki = rospy.get_param("/pid_controller/Ki", self.Ki)
            new_Kd = rospy.get_param("/pid_controller/Kd", self.Kd)
            
            # ゲインが変更された場合のみ更新
            if (abs(new_Kp - self.Kp) > 1e-6 or 
                abs(new_Ki - self.Ki) > 1e-6 or 
                abs(new_Kd - self.Kd) > 1e-6):
                
                rospy.logwarn("★ Gains updated: Kp=%.5f, Ki=%.5f, Kd=%.5f", 
                              new_Kp, new_Ki, new_Kd)
                
                self.Kp = new_Kp
                self.Ki = new_Ki
                self.Kd = new_Kd
                
                # ゲイン変更時に積分項をリセット（過渡応答を改善）
                self.error_integral = 0.0
                rospy.loginfo("  Integral term reset")
        except Exception as e:
            # パラメータが存在しない場合は無視
            pass
    
    def cb_joint(self, msg):
        try:
            self.theta_cur = float(msg.position[2])
            if abs(math.degrees(self.theta_cur)) > self.safety_angle:
                if not self.emergency_stop:
                    rospy.logerr("EMERGENCY STOP: angle %.1f° exceeds limit!",
                                 math.degrees(self.theta_cur))
                    self.emergency_stop = True
        except:
            pass
    
    def cb_target(self, msg):
        new_target = math.radians(float(msg.data))
        new_target = np.clip(new_target,
                             math.radians(-self.safety_angle * 0.9),
                             math.radians(self.safety_angle * 0.9))
        
        # 目標変更時に積分項をリセット（オプション）
        if abs(new_target - self.theta_target) > math.radians(5.0):
            rospy.loginfo("Target changed, resetting integral term")
            self.error_integral = 0.0
        
        self.theta_target = new_target
    
    def cb_log_enable(self, msg):
        self.log_enabled = msg.data
        if self.log_enabled:
            self.log_data = []
            self.start_time = rospy.get_time()
            rospy.loginfo("Logging enabled")
        else:
            self.save_log()
            rospy.loginfo("Logging disabled, data saved")
    
    def save_log(self):
        if len(self.log_data) == 0:
            return
        
        import pandas as pd
        import os
        
        # ログディレクトリを確認・作成
        log_dir = "/tmp"
        try:
            log_dir = "/home/kan/ros/jsk_aerial_robot_ws/src/jsk_aerial_robot/robots/kinikun/scripts/pressure_control/pid/logs"
            os.makedirs(log_dir, exist_ok=True)
        except:
            log_dir = "/tmp"
        
        df = pd.DataFrame(self.log_data)
        filename = f"{log_dir}/pid_log_{int(time.time())}.csv"
        df.to_csv(filename, index=False)
        rospy.loginfo("Saved %d samples to %s", len(df), filename)
        
        # 簡易統計
        if 'error_deg' in df.columns:
            rospy.loginfo("Performance summary:")
            rospy.loginfo("  Mean error: %.2f deg", df['error_deg'].mean())
            rospy.loginfo("  RMS error:  %.2f deg", np.sqrt((df['error_deg']**2).mean()))
            rospy.loginfo("  Max error:  %.2f deg", df['error_deg'].abs().max())
    
    def control_step(self):
        if self.emergency_stop:
            p1, p2 = self.p_bias / 2, self.p_bias / 2
            self.error_integral = 0.0
        else:
            # 誤差計算
            error = self.theta_target - self.theta_cur
            error_deg = math.degrees(error)
            
            # P項
            p_term = self.Kp * error
            
            # I項（条件付き積分: デッドゾーン外かつ飽和していない）
            i_term = 0.0
            if self.enable_integral:
                if abs(error_deg) > self.deadzone:
                    # 積分項更新
                    self.error_integral += error * self.dt
                    # アンチワインドアップ
                    self.error_integral = np.clip(self.error_integral,
                                                  -self.i_max / (self.Ki + 1e-9),
                                                  self.i_max / (self.Ki + 1e-9))
                i_term = self.Ki * self.error_integral
            
            # D項
            d_error = (error - self.error_prev) / self.dt
            d_term = self.Kd * d_error
            
            # PID出力（差圧指令）
            pd_cmd = p_term + i_term + d_term
            
            # 和圧は一定（簡略化）または目標角度に応じて調整
            ps = self.p_bias
            
            # (ps, pd) -> (p1, p2)
            pd = np.clip(pd_cmd, -(self.p_max - self.p_min), (self.p_max - self.p_min))
            p1 = np.clip(0.5 * (ps + pd), self.p_min, self.p_max)
            p2 = np.clip(0.5 * (ps - pd), self.p_min, self.p_max)
            
            # レート制限
            p1 = np.clip(p1, self.p1_prev - self.rate_limit,
                         self.p1_prev + self.rate_limit)
            p2 = np.clip(p2, self.p2_prev - self.rate_limit,
                         self.p2_prev + self.rate_limit)
            
            # ログ記録
            if self.log_enabled and self.start_time is not None:
                self.log_data.append({
                    'time': rospy.get_time() - self.start_time,
                    'target_deg': math.degrees(self.theta_target),
                    'actual_deg': math.degrees(self.theta_cur),
                    'error_deg': error_deg,
                    'p_term': p_term,
                    'i_term': i_term,
                    'd_term': d_term,
                    'pd_cmd': pd_cmd,
                    'p1': p1,
                    'p2': p2,
                    'error_integral': self.error_integral
                })
            
            # Publish diagnostics
            self.pub_error.publish(Float32(error_deg))
            self.pub_control.publish(Vector3(p_term, i_term, d_term))
            
            self.error_prev = error
            self.p1_prev = p1
            self.p2_prev = p2
        
        # 送信
        c1 = int(np.clip(p1 * self.cmd_gain, 0, self.cmd_clip))
        c2 = int(np.clip(p2 * self.cmd_gain, 0, self.cmd_clip))
        self.pub_cmd.publish(Vector3(c1, c2, 0.0))
        
        rospy.loginfo_throttle(0.5,
            "θ: %.1f°→%.1f° (e=%.2f°) | PID=(%.3f,%.3f,%.3f) | p1=%.3f p2=%.3f",
            math.degrees(self.theta_cur), math.degrees(self.theta_target),
            math.degrees(self.theta_target - self.theta_cur),
            p_term if not self.emergency_stop else 0,
            i_term if not self.emergency_stop else 0,
            d_term if not self.emergency_stop else 0,
            p1, p2
        )
    
    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.control_step()
            rate.sleep()

def main():
    rospy.init_node("pid_controller")
    controller = PIDController()
    rospy.on_shutdown(controller.save_log)
    rospy.sleep(1.0)
    controller.spin()

if __name__ == "__main__":
    main()