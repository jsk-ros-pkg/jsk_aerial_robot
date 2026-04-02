#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Vector3

DURATION = 10.0  # 目標値までの変化にかける秒数（よりゆっくり）
FREQUENCY = 10   # 更新頻度（Hz）
PAUSE_TIME = 2.0 # 目標値到達後の停止時間（秒）

# p1_value と p2_value の切り替え目標値
TARGET_VALUES = [(0.1, 0.65), (0.65, 0.1)]  # 交互に切り替え

def gradually_update(start, end, duration, frequency):
    """指定の開始と終了値の間を線形に補間してリストを返す"""
    steps = int(duration * frequency)
    return [(start + (end - start) * (i / steps)) for i in range(steps + 1)]

def main():
    rospy.init_node('param_loader', anonymous=True)

    # Publisherの設定
    pub_v1v2 = rospy.Publisher('mpa_cmd', Vector3, queue_size=10)

    rate = rospy.Rate(FREQUENCY)  # 更新頻度で実行

    # 初回の current_p1 と current_p2 を取得
    current_p1, current_p2 = TARGET_VALUES[0]  # 最初の目標値を設定
    target_index = 1  # 次の目標値のインデックス

    try:
        while not rospy.is_shutdown():
            target_p1, target_p2 = TARGET_VALUES[target_index]  # 次の目標値を取得
            rospy.loginfo(f"New target set: p1={target_p1}, p2={target_p2}")

            # 補間値リストを生成
            p1_values = gradually_update(current_p1, target_p1, DURATION, FREQUENCY)
            p2_values = gradually_update(current_p2, target_p2, DURATION, FREQUENCY)

            for p1, p2 in zip(p1_values, p2_values):
                if rospy.is_shutdown():
                    rospy.loginfo("Shutdown detected. Exiting loop.")
                    sys.exit(0)

                v1_value = p1 * 4096 / 0.9
                v2_value = p2 * 4096 / 0.9

                mpa_cmd_msg = Vector3()
                mpa_cmd_msg.x = v1_value
                mpa_cmd_msg.y = v2_value

                # パラメータの更新とメッセージのPublish
                rospy.set_param('/p1_value', p1)
                rospy.set_param('/p2_value', p2)

                pub_v1v2.publish(mpa_cmd_msg)
                rospy.loginfo(f"Published V1={v1_value}, V2={v2_value}")

                rate.sleep()  # 周期的に実行

            # 目標値に到達したら 2 秒停止
            rospy.loginfo("Target values reached, pausing for 2 seconds...")
            rospy.sleep(PAUSE_TIME)

            # 現在値を目標値に更新し、次の目標値に切り替え
            current_p1, current_p2 = target_p1, target_p2
            target_index = (target_index + 1) % len(TARGET_VALUES)  # 交互に切り替え

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt detected. Shutting down.")
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt detected. Shutting down.")
        rospy.signal_shutdown("User requested shutdown")
        sys.exit(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
