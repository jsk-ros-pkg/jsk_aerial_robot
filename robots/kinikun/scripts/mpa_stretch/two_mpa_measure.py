#!/usr/bin/env python3
import rospy, itertools, random, numpy as np
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32

class ExpDiffGridMPA:
    def __init__(self):
        # 上限
        self.p_max = rospy.get_param("~p_max_MPa", 0.70)  # 個別圧の上限
        # 和圧の範囲（0〜min(2*pmax, param)）
        self.p_sum_min = rospy.get_param("~p_sum_min_MPa", 0.0)
        self.p_sum_max = min(rospy.get_param("~p_sum_max_MPa", 1.00), 2.0*self.p_max)
        # グリッドかランダムか
        self.grid_sum_pts  = rospy.get_param("~grid_sum_pts", 12)   # 和圧方向ポイント数
        self.grid_diff_pts = rospy.get_param("~grid_diff_pts", 13)  # 差圧方向ポイント数（各和圧で対称に）
        self.randomize_order = rospy.get_param("~randomize_order", True)

        # タイミング
        self.rate_hz  = rospy.get_param("~rate_hz", 40.0)
        self.hold_sec = rospy.get_param("~hold_sec", 0.7)   # 滞留
        self.ramp_sec = rospy.get_param("~ramp_sec", 0.3)   # ランプ
        self.cycles   = rospy.get_param("~cycles", 3)       # グリッドを何周するか
        self.seed     = rospy.get_param("~seed", 123)
        random.seed(int(self.seed))

        # 出力
        self.pub_mpa  = rospy.Publisher("/mpa_cmd", Vector3, queue_size=10)
        self.pub_sum  = rospy.Publisher("~p_sum_MPa", Float32, queue_size=10)
        self.pub_diff = rospy.Publisher("~p_diff_MPa", Float32, queue_size=10)

        self.prev_p1 = 0.0
        self.prev_p2 = 0.0

        rospy.loginfo("[exp_diff_grid_mpa] sum in [%.2f, %.2f] MPa, pmax=%.2f MPa",
                      self.p_sum_min, self.p_sum_max, self.p_max)

        # スケジュール作成
        self.schedule = self._make_schedule()

    def _make_schedule(self):
        sums = np.linspace(self.p_sum_min, self.p_sum_max, self.grid_sum_pts)
        schedule = []
        for ps in sums:
            # 差圧の許容最大値（p1,p2>=0 & <=pmax を満たす）
            pdiff_max = min(ps, 2.0*self.p_max - ps)
            if pdiff_max < 1e-6:
                diffs = [0.0]
            else:
                diffs = np.linspace(-pdiff_max, +pdiff_max, self.grid_diff_pts)
            for pd in diffs:
                p1 = 0.5*(ps + pd)
                p2 = 0.5*(ps - pd)
                # 念のためクリップ
                if 0.0 <= p1 <= self.p_max and 0.0 <= p2 <= self.p_max:
                    schedule.append((float(ps), float(pd), float(p1), float(p2)))
        if self.randomize_order:
            random.shuffle(schedule)
        # 複数周回
        schedule = schedule * max(1, int(self.cycles))
        rospy.loginfo("[exp_diff_grid_mpa] %d setpoints generated.", len(schedule))
        return schedule

    def _slew(self, prev, cmd, dt, slew):
        dv = slew * dt
        return float(np.clip(cmd, prev - dv, prev + dv))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        i = 0
        # スルーレート（個別圧）を和圧ランプ時間に合わせて算出
        slew = self.p_sum_max / max(self.ramp_sec, 1e-3)
        t_state = rospy.Time.now().to_sec()
        phase = "ramp"
        p1_tgt, p2_tgt = 0.0, 0.0
        while not rospy.is_shutdown():
            if i >= len(self.schedule):
                self._publish(0.0, 0.0)
                break

            now = rospy.Time.now().to_sec()
            dt = 1.0/self.rate_hz

            if phase == "ramp":
                ps, pd, p1_tgt, p2_tgt = self.schedule[i]
                # 目標を向けながらランプ
                self.prev_p1 = self._slew(self.prev_p1, p1_tgt, dt, slew)
                self.prev_p2 = self._slew(self.prev_p2, p2_tgt, dt, slew)
                self._publish(self.prev_p1, self.prev_p2)
                if now - t_state >= self.ramp_sec:
                    phase = "hold"
                    t_state = now
            else:
                # hold
                self._publish(p1_tgt, p2_tgt)
                if now - t_state >= self.hold_sec:
                    phase = "ramp"
                    t_state = now
                    i += 1

            rate.sleep()

    def _publish(self, p1, p2):
        # クリップ
        p1 = float(np.clip(p1, 0.0, self.p_max))
        p2 = float(np.clip(p2, 0.0, self.p_max))
        msg = Vector3(); msg.x = p1* 4096 / 0.9 ; msg.y = p2* 4096 / 0.9 ; msg.z = 0.0
        self.pub_mpa.publish(msg)
        self.pub_sum.publish(Float32(p1 + p2))
        self.pub_diff.publish(Float32(p1 - p2))

if __name__ == "__main__":
    rospy.init_node("exp_diff_grid_mpa")
    ExpDiffGridMPA().run()
