#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy, random, numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

"""
完全版: 離散グリッド + 連続励起（サイン/マルチサイン/チャープ） + 微小PRBS
- 和圧Σ・差圧Δのスルーレートでランプ（個別圧p1,p2は和差から都度計算）
- 常に可行領域 |Δ| ≤ min(Σ, 2*pmax - Σ) を満たすようクリップ
- 出力:
    /mpa_cmd (Vector3): x=p1_cnt, y=p2_cnt, z=0  [DACカウント: 4096/0.9倍]
    ~p_sum_MPa, ~p_diff_MPa : 和・差（MPa）
    ~dp_sum_MPa_s, ~ddp_sum_MPa_s2, ~dp_diff_MPa_s, ~ddp_diff_MPa_s2 : 時間微分
    （任意）~z : z(ps,pd) をオンライン計算・Publish（--z-enable 相当のパラメータ）

推奨: 40–100 Hz で動かし、/θ（角度）等は rosbag で別途記録（学習時に結合）。
"""

def clip_feasible(ps, pd, pmax):
    pd_max = min(ps, 2.0*pmax - ps)
    pd = float(np.clip(pd, -pd_max, +pd_max))
    ps = float(np.clip(ps, 0.0, 2.0*pmax))  # 和圧は理論上[0, 2 pmax]
    return ps, pd

def ps_pd_to_p1p2(ps, pd):
    p1 = 0.5*(ps + pd)
    p2 = 0.5*(ps - pd)
    return float(p1), float(p2)

def p1p2_to_pspd(p1, p2):
    return float(p1 + p2), float(p1 - p2)

def ramp_step(ps_now, pd_now, ps_tgt, pd_tgt, dt, Ssum, Sdiff, pmax):
    dps = np.clip(ps_tgt-ps_now, -Ssum*dt, +Ssum*dt)
    dpd = np.clip(pd_tgt-pd_now, -Sdiff*dt, +Sdiff*dt)
    ps_new = ps_now + dps
    pd_new = pd_now + dpd
    return clip_feasible(ps_new, pd_new, pmax)

def z_eval(ps, pd, coef, feat):
    if not coef or not feat: return None
    vals = {"1":1.0, "ps":ps, "pd":pd, "ps*pd":ps*pd, "ps^2":ps*ps, "pd^2":pd*pd}
    return float(sum(coef[i]*vals[feat[i]] for i in range(len(feat)) if feat[i] in vals))

class ExpDiffGridMPA_Full:
    def __init__(self):
        # ===== 基本パラメータ =====
        self.p_max = rospy.get_param("~p_max_MPa", 0.70)       # 各pの上限
        self.p_sum_min = rospy.get_param("~p_sum_min_MPa", 0.0)
        self.p_sum_max = min(rospy.get_param("~p_sum_max_MPa", 1.00), 2.0*self.p_max)

        self.grid_sum_pts  = rospy.get_param("~grid_sum_pts", 12)
        self.grid_diff_pts = rospy.get_param("~grid_diff_pts", 13)
        self.randomize_order = rospy.get_param("~randomize_order", True)

        self.rate_hz  = rospy.get_param("~rate_hz", 40.0)
        self.hold_sec = rospy.get_param("~hold_sec", 0.7)
        self.ramp_sec = rospy.get_param("~ramp_sec", 0.3)
        self.cycles   = rospy.get_param("~cycles", 3)
        self.seed     = rospy.get_param("~seed", 123)
        random.seed(int(self.seed))

        # スルー（和/差の速度上限）と進入方向
        self.slew_sum_list  = rospy.get_param("~slew_sum_list_MPa_s",  [0.06, 0.12])
        self.slew_diff_list = rospy.get_param("~slew_diff_list_MPa_s", [0.12, 0.25])
        self.dir_modes = rospy.get_param("~dir_modes", ["up","down"])  # up:低側→目標, down:高側→目標
        self.settle_sec = rospy.get_param("~settle_sec", 1.0)          # 未使用（拡張用）

        # PRBS（微小励起）
        self.probe_enable = rospy.get_param("~probe_enable", True)
        self.probe_bit_sec = rospy.get_param("~probe_bit_sec", 0.1)
        self.probe_amp_sum  = rospy.get_param("~probe_amp_sum_MPa",  0.01)
        self.probe_amp_diff = rospy.get_param("~probe_amp_diff_MPa", 0.01)

        # ===== 連続励起オプション =====
        self.enable_sine   = rospy.get_param("~enable_sine", True)
        self.sine_sec      = rospy.get_param("~sine_sec", 12.0)
        self.sine_freqs_sum  = rospy.get_param("~sine_freqs_sum_Hz",  [0.08, 0.3])
        self.sine_freqs_diff = rospy.get_param("~sine_freqs_diff_Hz", [0.12, 0.5])
        self.sine_amp_sum   = rospy.get_param("~sine_amp_sum_MPa",  0.08)
        self.sine_amp_diff  = rospy.get_param("~sine_amp_diff_MPa", 0.08)
        self.sine_center_sum= rospy.get_param("~sine_center_sum_MPa", 0.5)

        self.enable_multisine = rospy.get_param("~enable_multisine", True)
        self.ms_sec           = rospy.get_param("~multisine_sec", 12.0)
        self.ms_freqs_sum     = rospy.get_param("~multisine_freqs_sum_Hz",  [0.07, 0.11, 0.23])
        self.ms_freqs_diff    = rospy.get_param("~multisine_freqs_diff_Hz", [0.09, 0.17, 0.31])
        self.ms_amp_sum       = rospy.get_param("~multisine_amp_sum_MPa",  0.06)
        self.ms_amp_diff      = rospy.get_param("~multisine_amp_diff_MPa", 0.06)
        self.ms_center_sum    = rospy.get_param("~multisine_center_sum_MPa", 0.5)

        self.enable_chirp   = rospy.get_param("~enable_chirp", True)
        self.chirp_sec      = rospy.get_param("~chirp_sec", 12.0)
        self.chirp_sum_f0   = rospy.get_param("~chirp_sum_f0_Hz",  0.05)
        self.chirp_sum_f1   = rospy.get_param("~chirp_sum_f1_Hz",  1.00)
        self.chirp_diff_f0  = rospy.get_param("~chirp_diff_f0_Hz", 0.07)
        self.chirp_diff_f1  = rospy.get_param("~chirp_diff_f1_Hz", 0.80)
        self.chirp_amp_sum  = rospy.get_param("~chirp_amp_sum_MPa",  0.08)
        self.chirp_amp_diff = rospy.get_param("~chirp_amp_diff_MPa", 0.08)
        self.chirp_center_sum= rospy.get_param("~chirp_center_sum_MPa", 0.5)

        # ===== z 出力（任意）=====
        self.z_enable = rospy.get_param("~z_enable", False)
        self.z_feat   = rospy.get_param("~z_feat", ["1","ps","pd","ps*pd","ps^2","pd^2"])
        self.z_coef   = rospy.get_param("~z_coef", [])  # 係数と特徴名の長さ一致に注意

        # ===== Publisher =====
        self.pub_mpa  = rospy.Publisher("/mpa_cmd", Vector3, queue_size=10)
        self.pub_sum  = rospy.Publisher("~p_sum_MPa",  Float32, queue_size=10)
        self.pub_diff = rospy.Publisher("~p_diff_MPa", Float32, queue_size=10)
        self.pub_dsum  = rospy.Publisher("~dp_sum_MPa_s",  Float32, queue_size=10)
        self.pub_ddsum = rospy.Publisher("~ddp_sum_MPa_s2", Float32, queue_size=10)
        self.pub_ddiff = rospy.Publisher("~dp_diff_MPa_s",  Float32, queue_size=10)
        self.pub_dddif2= rospy.Publisher("~ddp_diff_MPa_s2", Float32, queue_size=10)
        self.pub_z     = rospy.Publisher("~z", Float32, queue_size=10) if self.z_enable else None

        # ===== スケジュール生成 =====
        rospy.loginfo("[exp_full] sum in [%.2f, %.2f] MPa, pmax=%.2f MPa", self.p_sum_min, self.p_sum_max, self.p_max)
        self.schedule = self._make_schedule()  # list of dict

        # 状態
        self.prev_ps = 0.0
        self.prev_pd = 0.0
        self.prev2_ps = 0.0
        self.prev2_pd = 0.0

    # ---------- スケジュール生成 ----------
    def _make_schedule(self):
        sch = []

        # (A) 離散グリッド（ランプ→ホールド、進入方向/スルー水準の組合せ）
        sums = np.linspace(self.p_sum_min, self.p_sum_max, max(2, int(self.grid_sum_pts)))
        for ps in sums:
            pdiff_max = min(ps, 2.0*self.p_max - ps)
            diffs = [0.0] if pdiff_max < 1e-6 else np.linspace(-pdiff_max, +pdiff_max, max(2, int(self.grid_diff_pts)))
            for pd in diffs:
                for Ssum in self.slew_sum_list:
                    for Sdiff in self.slew_diff_list:
                        for dm in self.dir_modes:
                            margin = 0.02  # 和圧のプリスタート・マージン
                            if dm == "up":
                                ps_start = max(self.p_sum_min, min(ps, self.p_sum_max) - margin)
                            else:
                                ps_start = min(self.p_sum_max, max(ps, self.p_sum_min) + margin)
                            # 差圧は開始時点の可行域でクリップ
                            _, pd_start = clip_feasible(ps_start, pd, self.p_max)
                            sch.append(dict(
                                kind="grid",
                                ps=float(ps), pd=float(pd),
                                ps_start=float(ps_start), pd_start=float(pd_start),
                                Ssum=float(Ssum), Sdiff=float(Sdiff),
                                dir=dm,
                                ramp_sec=float(self.ramp_sec),
                                hold_sec=float(self.hold_sec)
                            ))

        # (B) 連続：サイン
        if self.enable_sine:
            sch.append(dict(
                kind="sine",
                duration=float(self.sine_sec),
                freqs_sum=[float(f) for f in self.sine_freqs_sum],
                freqs_diff=[float(f) for f in self.sine_freqs_diff],
                amp_sum=float(self.sine_amp_sum),
                amp_diff=float(self.sine_amp_diff),
                center_sum=float(self.sine_center_sum)
            ))

        # (C) 連続：マルチサイン
        if self.enable_multisine:
            sch.append(dict(
                kind="multisine",
                duration=float(self.ms_sec),
                freqs_sum=[float(f) for f in self.ms_freqs_sum],
                freqs_diff=[float(f) for f in self.ms_freqs_diff],
                amp_sum=float(self.ms_amp_sum),
                amp_diff=float(self.ms_amp_diff),
                center_sum=float(self.ms_center_sum)
            ))

        # (D) 連続：チャープ
        if self.enable_chirp:
            sch.append(dict(
                kind="chirp",
                duration=float(self.chirp_sec),
                sum_f0=float(self.chirp_sum_f0), sum_f1=float(self.chirp_sum_f1),
                diff_f0=float(self.chirp_diff_f0), diff_f1=float(self.chirp_diff_f1),
                amp_sum=float(self.chirp_amp_sum),
                amp_diff=float(self.chirp_amp_diff),
                center_sum=float(self.chirp_center_sum)
            ))

        if self.randomize_order:
            random.shuffle(sch)

        sch = sch * max(1, int(self.cycles))
        rospy.loginfo("[exp_full] schedule items: %d", len(sch))
        return sch

    # ---------- 実行 ----------
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / max(1e-6, self.rate_hz)

        i = 0
        phase = "idle"
        t0 = rospy.Time.now().to_sec()
        t_phase = t0

        # 現在の和・差（ランプで更新、連続でも逐次決定）
        ps_now, pd_now = 0.0, 0.0

        while not rospy.is_shutdown():
            if i >= len(self.schedule):
                self._publish(0.0, 0.0, dt)
                break

            item = self.schedule[i]
            kind = item["kind"]
            now = rospy.Time.now().to_sec()

            if kind == "grid":
                # ランプ→ホールド
                if phase == "idle":
                    # 進入方向に合わせて開始点から
                    ps_now = float(item["ps_start"])
                    pd_now = float(item["pd_start"])
                    phase = "ramp"
                    t_phase = now

                elif phase == "ramp":
                    ps_tgt = float(item["ps"])
                    pd_tgt = float(item["pd"])
                    ps_now, pd_now = ramp_step(
                        ps_now, pd_now, ps_tgt, pd_tgt,
                        dt, item["Ssum"], item["Sdiff"], self.p_max
                    )
                    if (now - t_phase) >= item["ramp_sec"] or (abs(ps_tgt-ps_now)<1e-4 and abs(pd_tgt-pd_now)<1e-4):
                        phase = "hold"; t_phase = now

                elif phase == "hold":
                    ps_cmd, pd_cmd = float(item["ps"]), float(item["pd"])
                    # PRBS（微小）を和/差に注入
                    if self.probe_enable:
                        bit = 1 if int(((now - t_phase) // self.probe_bit_sec) % 2) == 1 else -1
                        ps_cmd += bit * self.probe_amp_sum
                        pd_cmd += bit * self.probe_amp_diff
                        ps_cmd, pd_cmd = clip_feasible(ps_cmd, pd_cmd, self.p_max)
                    ps_now, pd_now = ps_cmd, pd_cmd
                    if (now - t_phase) >= item["hold_sec"]:
                        # 次アイテムへ
                        i += 1
                        phase = "idle"
                        continue

            elif kind in ("sine", "multisine", "chirp"):
                # 連続励起セグメント
                if phase == "idle":
                    phase = "cont"
                    t_phase = now

                t = now - t_phase
                dur = float(item["duration"])
                if kind == "sine":
                    fs = random.choice(item["freqs_sum"]) if item["freqs_sum"] else 0.2
                    fd = random.choice(item["freqs_diff"]) if item["freqs_diff"] else 0.3
                    ps_cmd = item["center_sum"] + item["amp_sum"] * np.sin(2*np.pi*fs*t)
                    pd_cmd = item["amp_diff"] * np.sin(2*np.pi*fd*t + np.pi/3)
                elif kind == "multisine":
                    ps_cmd = item["center_sum"] + sum(item["amp_sum"] * np.sin(2*np.pi*f*t + ph)
                                                      for f,ph in zip(item["freqs_sum"], np.linspace(0, np.pi, len(item["freqs_sum"])) ))
                    pd_cmd = sum(item["amp_diff"] * np.sin(2*np.pi*f*t + ph)
                                 for f,ph in zip(item["freqs_diff"], np.linspace(np.pi/4, 7*np.pi/4, len(item["freqs_diff"])) ))
                else:  # chirp
                    # 線形チャープ：f(t) = f0 + (f1-f0)*(t/dur)
                    fs = item["sum_f0"]  + (item["sum_f1"]  - item["sum_f0"] ) * min(1.0, t/dur)
                    fd = item["diff_f0"] + (item["diff_f1"] - item["diff_f0"]) * min(1.0, t/dur)
                    ps_cmd = item["center_sum"] + item["amp_sum"]  * np.sin(2*np.pi*fs*t)
                    pd_cmd = item["amp_diff"]   * np.sin(2*np.pi*fd*t)

                ps_cmd, pd_cmd = clip_feasible(float(ps_cmd), float(pd_cmd), self.p_max)
                ps_now, pd_now = ps_cmd, pd_cmd

                if t >= dur:
                    i += 1
                    phase = "idle"
                    continue

            else:
                rospy.logwarn_throttle(5.0, "[exp_full] unknown kind=%s", kind)
                i += 1
                phase = "idle"
                continue

            # 出力＆派生量
            self._publish_from_pspd(ps_now, pd_now, dt)
            rate.sleep()

    # ---------- 出力 ----------
    def _publish_from_pspd(self, ps, pd, dt):
        # 1) h=有限差分のため履歴を更新
        self.prev2_ps, self.prev_ps = self.prev_ps, ps
        self.prev2_pd, self.prev_pd = self.prev_pd, pd
        # 2) 個別圧へ
        p1, p2 = ps_pd_to_p1p2(ps, pd)
        # 3) クリップ（個別圧）
        p1 = float(np.clip(p1, 0.0, self.p_max))
        p2 = float(np.clip(p2, 0.0, self.p_max))
        # 4) Publish（DACカウントへ換算: 4096/0.9）
        self._publish(p1, p2)
        # 5) 和・差の派生量（一次/二次微分）
        dps  = (self.prev_ps - self.prev2_ps)/max(1e-6, dt)
        ddps = (self.prev_ps - 2*self.prev2_ps + 0.0)/max(1e-6, dt*dt)  # 粗い二次（必要ならSGで）
        dpd  = (self.prev_pd - self.prev2_pd)/max(1e-6, dt)
        ddpd = (self.prev_pd - 2*self.prev2_pd + 0.0)/max(1e-6, dt*dt)
        self.pub_sum.publish(Float32(self.prev_ps))
        self.pub_diff.publish(Float32(self.prev_pd))
        self.pub_dsum.publish(Float32(dps))
        self.pub_ddsum.publish(Float32(ddps))
        self.pub_ddiff.publish(Float32(dpd))
        self.pub_dddif2.publish(Float32(ddpd))
        # 6) z（任意）
        if self.z_enable and self.pub_z:
            try:
                z = z_eval(self.prev_ps, self.prev_pd, self.z_coef, self.z_feat)
                if z is not None: self.pub_z.publish(Float32(z))
            except Exception as e:
                rospy.logwarn_throttle(5.0, "[exp_full] z_eval error: %s", e)

    def _publish(self, p1, p2):
        # MPa→DAC counts（0..4096）
        scale = 4096.0/0.9
        cnt1 = float(np.clip(p1*scale, 0.0, 4096.0))
        cnt2 = float(np.clip(p2*scale, 0.0, 4096.0))
        msg = Vector3(); msg.x = cnt1; msg.y = cnt2; msg.z = 0.0
        self.pub_mpa.publish(msg)

if __name__ == "__main__":
    rospy.init_node("exp_diff_grid_mpa_full")
    ExpDiffGridMPA_Full().run()
