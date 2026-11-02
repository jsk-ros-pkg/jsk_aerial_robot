#!/usr/bin/env python3
import rospy, numpy as np
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class ExpSingleRandomMPA:
    def __init__(self):
        # 出力レンジ MPa
        self.p1_min = rospy.get_param("~p1_min_MPa", 0.00)
        self.p1_max = rospy.get_param("~p1_max_MPa", 0.70)  # = 7bar
        # タイミング
        self.rate_hz   = rospy.get_param("~rate_hz", 40.0)
        self.hold_sec  = rospy.get_param("~hold_sec", 0.7)  # 1点あたりの静置時間
        self.ramp_sec  = rospy.get_param("~ramp_sec", 0.2)  # ステップ→ランプで安全に
        self.duration  = rospy.get_param("~duration_sec", 120.0)  # 実験全体時間
        self.seed      = rospy.get_param("~seed", 42)
        np.random.seed(int(self.seed))

        # 出力
        self.pub_p12   = rospy.Publisher("/p1p2_cmd", Vector3, queue_size=10)
        self.pub_sum   = rospy.Publisher("~p_sum_MPa", Float32, queue_size=10)
        self.pub_diff  = rospy.Publisher("~p_diff_MPa", Float32, queue_size=10)

        self.prev_p1 = 0.0
        self.t0 = rospy.Time.now().to_sec()
        rospy.loginfo("[exp_single_random_mpa] start: %.1f s", self.duration)

    def _slew(self, prev, cmd, dt, slew=1e9):
        dv = slew * dt
        return float(np.clip(cmd, prev - dv, prev + dv))

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        state = "ramp"
        t_step = rospy.Time.now().to_sec()
        target_p1 = float(np.random.uniform(self.p1_min, self.p1_max))
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            if now - self.t0 > self.duration:
                # 終了時はゼロ出力
                self._publish(0.0, 0.0)
                break

            # ランダムターゲットの更新
            if state == "ramp" and now - t_step >= self.ramp_sec:
                state = "hold"
                t_step = now
            elif state == "hold" and now - t_step >= self.hold_sec:
                state = "ramp"
                t_step = now
                target_p1 = float(np.random.uniform(self.p1_min, self.p1_max))

            # p1 をランプして publish, p2は常に0
            dt = 1.0/self.rate_hz
            slew = (self.p1_max - self.p1_min) / max(self.ramp_sec, 1e-3)
            if state == "ramp":
                p1_cmd = self._slew(self.prev_p1, target_p1, dt, slew)
            else:
                p1_cmd = target_p1

            self._publish(p1_cmd, 0.0)
            self.prev_p1 = p1_cmd
            rate.sleep()

    def _publish(self, p1, p2):
        msg = Vector3()
        msg.x, msg.y, msg.z = float(p1), float(p2), 0.0
        self.pub_p12.publish(msg)
        self.pub_sum.publish(Float32(p1 + p2))
        self.pub_diff.publish(Float32(p1 - p2))

if __name__ == "__main__":
    rospy.init_node("exp_single_random_mpa")
    ExpSingleRandomMPA().run()
