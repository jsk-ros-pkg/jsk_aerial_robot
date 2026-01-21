#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState
from scipy.optimize import minimize

class SoftLinkAnalyticalSolver:
    def __init__(self):
        print("Soft Link IK Solver Node Started")
        # ノードの初期化
        
        self.joint_states_sub = rospy.Subscriber("joint_states", JointState, self.joint_states_cb, queue_size=1)

        print("Determining simulation mode...")
        
        if self.is_simulation():
            self.joint_states_pub = rospy.Publisher("joints_ctrl", JointState, queue_size=1)
        else:
            self.joint_states_pub = rospy.Publisher("joint_states", JointState, queue_size=1)

        # --- リンク長パラメータ (単位: m) ---
        # ユーザー指定: link1 ~ link4
        self.L_RIGID = 0.5275 
        
        # URDFより抽出した柔軟リンク長
        # link4末端からsoft_joint2まで: soft_joint2 origin x=0.1175 (soft_link1内)
        self.L_S1 = 0.1175 
        # soft_joint2からsoft_joint3まで: soft_joint3 origin x=0.235
        self.L_S2 = 0.235  
        # soft_joint3からsoft_joint5まで: 
        #   soft3->gimbal(0.128) + gimbal->soft4(0.147) + soft4->soft5(0.1175)
        self.L_S3_to_S5 = 0.128 + 0.147 + 0.1175 # = 0.3925
        # soft_joint5からsoft_joint6まで: soft_joint6 origin x=0.235
        self.L_S6 = 0.235
        # soft_joint6から先端まで: soft_link6の長さ(推定)
        # 閉ループとして自然な接続のため、始点と同じ0.1175と仮定
        self.L_END = 0.1175 

        # 前回の解を保持（次回の初期値にして計算を安定させる）
        self.last_sol = [0.0, 0.0, 0.0, 0.0]

        rospy.loginfo("Soft Link IK Solver Initialized with fixed lengths.")
        rospy.loginfo(f"Rigid Len: {self.L_RIGID}, Soft Lens: {self.L_S1}, {self.L_S2}, {self.L_S3_to_S5}, {self.L_S6}, {self.L_END}")

    def is_simulation(self) -> bool:
        return rospy.get_param('/use_sim_time', False)

    def joint_states_cb(self, msg: JointState):
        # 関節角度の取得
        name_map = {name: i for i, name in enumerate(msg.name)}
        try:
            rigid_angles = [None, None, None]  # joint1, joint2, joint3
            for n, j in zip(msg.name, msg.position):
                if n == 'joint1':
                    rigid_angles[0] = j
                elif n == 'joint2':
                    rigid_angles[1] = j
                elif n == 'joint3':
                    rigid_angles[2] = j
            if None in rigid_angles:
                rospy.logwarn("Not all rigid joints found in joint states")
                return
        except KeyError:
            return

        # IKを解く
        soft_angles = self.solve_closed_loop_ik(rigid_angles)

        # Publish
        out_msg = JointState()
        out_msg.header.stamp = rospy.Time.now()
        out_msg.name = ["soft_joint2", "soft_joint3", "soft_joint5", "soft_joint6"]
        out_msg.position = soft_angles
        print("Published Soft Joint Angles:", soft_angles)
        self.joint_states_pub.publish(out_msg)

    def solve_closed_loop_ik(self, rigid_joints):
        """
        剛体関節(j1, j2, j3)を入力とし、ループが閉じる柔軟関節(s2, s3, s5, s6)を求める。
        対称性は仮定せず、位置と角度が一致するように全変数を最適化する。
        """
        j1, j2, j3 = rigid_joints

        # 最適化の目的関数
        def objective_func(soft_joints):
            s2, s3, s5, s6 = soft_joints
            
            # --- Forward Kinematics (2D平面モデル) ---
            # 座標系: Rootを(0,0), 角度0 とする
            # 順序: Root -> (L_r) -> J1 -> (L_r) -> J2 -> (L_r) -> J3 -> (L_r) -> SoftStart
            #      -> (L_s1) -> sJ2 -> (L_s2) -> sJ3 -> (L_s3_5) -> sJ5 -> (L_s6) -> sJ6 -> (L_end) -> Tip
            
            x, y, theta = 0.0, 0.0, 0.0
            
            # Rigid Chain
            # Link1 (Root -> J1)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            theta += j1
            
            # Link2 (J1 -> J2)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            theta += j2
            
            # Link3 (J2 -> J3)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            theta += j3
            
            # Link4 (J3 -> SoftStart)
            x += self.L_RIGID * math.cos(theta)
            y += self.L_RIGID * math.sin(theta)
            # ここから柔軟関節セクション。角度変化なしで接続と仮定
            
            # Soft Chain
            # Soft Link 1 (SoftStart -> sJ2)
            x += self.L_S1 * math.cos(theta)
            y += self.L_S1 * math.sin(theta)
            theta += s2
            
            # Soft Link 2 (sJ2 -> sJ3)
            x += self.L_S2 * math.cos(theta)
            y += self.L_S2 * math.sin(theta)
            theta += s3
            
            # Soft Link Middle (sJ3 -> sJ5, skipping gimbal joint as rigid block)
            x += self.L_S3_to_S5 * math.cos(theta)
            y += self.L_S3_to_S5 * math.sin(theta)
            theta += s5
            
            # Soft Link 6 (sJ5 -> sJ6)
            x += self.L_S6 * math.cos(theta)
            y += self.L_S6 * math.sin(theta)
            theta += s6
            
            # End Link (sJ6 -> Tip)
            x += self.L_END * math.cos(theta)
            y += self.L_END * math.sin(theta)
            
            # --- 誤差計算 ---
            # ターゲット: 
            # 位置: (0, 0) に戻るべき
            # 角度: 全周回って 2pi (360度) になるべき
            
            pos_error = x**2 + y**2
            ang_error = (theta - 2 * math.pi)**2
            
            # 正則化項: なるべく自然な姿勢（極端な値を避ける）
            reg = 0.001 * (s2**2 + s3**2 + s5**2 + s6**2)
            
            # 角度誤差の重みを大きくして閉ループ形状を優先
            return pos_error + 2.0 * ang_error + reg

        # 初期値: 前回の解 または 適当な初期値
        x0 = self.last_sol
        
        # 最適化実行 (SLSQP法は制約付き問題に強い)
        # 探索範囲は -pi ~ pi
        bounds = [(-np.pi, np.pi) for _ in range(4)]
        
        res = minimize(objective_func, x0, method='SLSQP', bounds=bounds, tol=1e-4)
        
        if res.success:
            self.last_sol = res.x
            return res.x.tolist()
        else:
            # 収束しなかった場合は前回の値を返すか、Warnを出す
            # rospy.logwarn("IK not converged")
            return self.last_sol

def main():
    rospy.init_node("pub_soft_link_joint_states_ik")
    try:
        SoftLinkAnalyticalSolver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()