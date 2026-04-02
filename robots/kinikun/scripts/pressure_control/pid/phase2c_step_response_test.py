#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase 2C: 自動ステップ応答テスト
複数の目標角度に対してステップ応答を記録し、
性能指標を自動計算する
"""
import rospy, math, time
import numpy as np
from std_msgs.msg import Float32, Bool

class StepResponseTester:
    def __init__(self):
        self.test_targets = rospy.get_param("~test_targets", [0, 10, 20, 30, -10, -20])  # deg
        self.settle_time_threshold = rospy.get_param("~settle_threshold", 2.0)  # deg
        self.hold_time = rospy.get_param("~hold_time", 5.0)  # seconds
        self.wait_time = rospy.get_param("~wait_time", 1.0)  # seconds between steps
        
        self.pub_target = rospy.Publisher("/theta_target_deg", Float32, queue_size=1)
        self.pub_log_enable = rospy.Publisher("/pid_log_enable", Bool, queue_size=1)
        
        rospy.loginfo("Step Response Tester initialized")
        rospy.loginfo("Targets: %s", self.test_targets)
    
    def run_test_sequence(self):
        rospy.sleep(2.0)  # 起動待ち
        
        # ログ開始
        self.pub_log_enable.publish(Bool(True))
        rospy.sleep(0.5)
        
        for i, target in enumerate(self.test_targets):
            rospy.loginfo("=== Step %d/%d: Target %.1f deg ===", 
                          i+1, len(self.test_targets), target)
            
            # 目標設定
            self.pub_target.publish(Float32(target))
            
            # 保持
            rospy.sleep(self.hold_time)
            
            # 次のステップまで待機
            if i < len(self.test_targets) - 1:
                rospy.sleep(self.wait_time)
        
        # ログ停止
        rospy.sleep(1.0)
        self.pub_log_enable.publish(Bool(False))
        rospy.loginfo("Test sequence complete!")

def main():
    rospy.init_node("step_response_tester")
    tester = StepResponseTester()
    tester.run_test_sequence()

if __name__ == "__main__":
    main()