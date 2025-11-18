#!/usr/bin/env python

import rospy
from spinal.msg import ServoControlCmd # メッセージ型のインポート (パスは環境に合わせて確認してください)
import time

def publish_servo_commands():
    # 1. ROSノードの初期化
    # ノード名はユニークである必要があります。
    rospy.init_node('servo_command_publisher', anonymous=True)
    
    # 2. パブリッシャーの作成
    # トピック名とメッセージ型を指定します。
    # queue_size=1 は、メッセージのキューのサイズを制限します。
    pub = rospy.Publisher('/crobat/extra_servo_cmd', ServoControlCmd, queue_size=1)
    
    # 3. ノードがROSマスタとパブリッシャーの準備ができるのを待つ
    # 必須ではありませんが、確実にメッセージを送信するために推奨されます。
    rospy.sleep(1) 

    # 4. 送信するサーボコマンドのリストを定義
    # 各要素は (index, angle) のタプルです。
    commands = [
        (4, 97),
        (5, 95),
        (6, 100),
        (7, 95)
    ]
    
    # 5. コマンドを順番に発行
    for index, angle in commands:
        # ServoControlCmd メッセージオブジェクトを作成
        cmd_msg = ServoControlCmd()
        
        # メッセージフィールドに値を設定
        # index: [4] -> cmd_msg.index = [4]
        cmd_msg.index = [index]
        # angles: [97] -> cmd_msg.angles = [97]
        cmd_msg.angles = [angle]

        # メッセージをトピックにパブリッシュ
        rospy.loginfo("Publishing: Index %d, Angle %d" % (index, angle))
        pub.publish(cmd_msg)
        
        # メッセージの送信が完了するのを少し待つ（Bashの -1 とは異なり、Pythonではループ処理が必要です）
        # rostopic pub -1 は1回だけ送信して終了しますが、Pythonスクリプトでは、
        # 受信側に確実に届けるため、コマンド間に短いディレイを入れると良い場合があります。
        rospy.sleep(4) 
        
    rospy.loginfo("All servo commands published.")

if __name__ == '__main__':
    try:
        publish_servo_commands()
    except rospy.ROSInterruptException:
        pass
