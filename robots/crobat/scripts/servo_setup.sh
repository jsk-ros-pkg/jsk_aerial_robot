#!/bin/bash

# このスクリプトは、ROSトピック '/crobat/extra_servo_cmd' に
# ServoControlCmd メッセージを発行し、サーボモーターの角度を設定します。

# メッセージの内容:
# index: [4]  (サーボモーターのインデックス/ID)
# angles: [95] (設定する角度)

rostopic pub -1 /crobat/extra_servo_cmd spinal/ServoControlCmd "index: [4]
angles: [97]"
rostopic pub -1 /crobat/extra_servo_cmd spinal/ServoControlCmd "index: [5]
angles: [95]"
rostopic pub -1 /crobat/extra_servo_cmd spinal/ServoControlCmd "index: [6] 
angles: [100]"
rostopic pub -1 /crobat/extra_servo_cmd spinal/ServoControlCmd "index: [7] 
angles: [95]"
