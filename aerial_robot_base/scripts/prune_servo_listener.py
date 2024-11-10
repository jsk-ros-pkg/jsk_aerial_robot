#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
sys.path.append("/home/leus/kamada_scripts")
import rospy
from std_msgs.msg import Float64  # 角度データを扱うトピックの型
from dx2lib import *  # dx2libをインポート
from setting import *  # サンプル共通のポート・ボーレート・ID等

# グローバル変数の初期化
COMPort = "/dev/ttyACM0"
Baudrate = 57600
TargetID = 1
dev = None

def angle_callback(msg):
    """
    /dynamixel/cmd_angle トピックを監視し、受信した角度にモータを移動させる
    """
    angle = msg.data  # トピックから受け取った角度データ
    rospy.loginfo(f"Received angle: {angle}°")

    # 角度がDynamixelの範囲内かチェック
    if -180.0 <= angle <= 180.0:
        # Dynamixelの目標角度を設定
        DXL_SetGoalAngleAndVelocity(dev, TargetID, angle, 1000.0)
        rospy.loginfo(f"Setting angle to {angle}°")
    else:
        rospy.logwarn(f"Angle out of range: {angle}°. Must be between -180° and 180°.")

def main():
    global dev

    # Dynamixelとの接続を開く
    rospy.loginfo("Opening Dynamixel port...")
    dev = DX2_OpenPort(COMPort, Baudrate)

    if dev is None:
        rospy.logerr("Could not open COM port.")
        return

    # Dynamixelのモデル情報を取得
    if DXL_GetModelInfo(dev, TargetID).contents.devtype == devtNONE:
        rospy.logerr("Device information could not be acquired.")
        DX2_ClosePort(dev)
        return

    rospy.loginfo("Connected to Dynamixel: " +
                  DXL_GetModelInfo(dev, TargetID).contents.name.decode())

    # DynamixelをJointモードに設定
    DXL_SetOperatingMode(dev, TargetID, 3)
    DXL_SetTorqueEnable(dev, TargetID, True)

    # ROSノードの初期化
    rospy.init_node("dynamixel_controller", anonymous=True)
    rospy.Subscriber("/dynamixel/cmd_angle", Float64, angle_callback)
    rospy.loginfo("Subscribed to /dynamixel/cmd_angle")

    # ノードをスピンしてコールバックを有効化
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down...")

    # トルクディスエーブルとポートのクローズ
    DXL_SetTorqueEnable(dev, TargetID, False)
    DX2_ClosePort(dev)
    rospy.loginfo("Dynamixel port closed.")

if __name__ == "__main__":
    main()
