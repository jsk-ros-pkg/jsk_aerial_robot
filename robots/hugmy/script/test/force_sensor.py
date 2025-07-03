#!/usr/bin/env python3
"""
RZ-100 Force Gauge ROS Publisher
"""

import rospy
import serial
from std_msgs.msg import Float32

def main():
    rospy.init_node("force_gauge_node")

    # ROSパラメータ
    port = rospy.get_param("~port", "/dev/ttyUSB0")  # 接続ポート
    baud = rospy.get_param("~baud", 19200)  # ボーレート
    timeout = rospy.get_param("~timeout", 10.0)  # タイムアウト
    hz = rospy.get_param("~hz", 10)  # 配信周波数

    # シリアル通信の設定
    try:
        sr = serial.Serial(port, baudrate=baud, timeout=timeout)
    except serial.SerialException as e:
        rospy.logerr(f"Failed to open serial port {port}: {e}")
        return

    pub = rospy.Publisher("force_gauge", Float32, queue_size=10)
    rate = rospy.Rate(hz)
    rospy.loginfo("Force gauge node started")

    while not rospy.is_shutdown():
        try:
            # データ要求コマンド送信 (適切なコマンドに置き換え)
            sr.write(b"D\r")
            # データの受信
            response = sr.readline().decode('utf-8', errors='ignore').strip()
            rospy.loginfo(f"Raw response: '{response}'")

            # データが空の場合の対処
            if not response:
                rospy.logwarn("No data received (empty response).")
                continue

            # データの変換
            value = float(response)
            msg = Float32(data=value)
            pub.publish(msg)
            rospy.loginfo(f"Force: {value} N")
        except ValueError as e:
            rospy.logwarn(f"Invalid data received: '{response}' ({e})")
        except serial.SerialException as e:
            rospy.logerr(f"Serial communication error: {e}")
        rate.sleep()

    sr.close()

if __name__ == "__main__":
    main()
