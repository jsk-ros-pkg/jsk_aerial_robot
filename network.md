---
layout: default
title: network
---


### 1. Wifi-infra mode
There is a router connect all device (e.g. remote pc, on-board processor) as shown in following figure.
![](images/network2.jpg)
 For `estimate_mode = 0` and `estimate_mode = 1`, which is the mode use on-board IMU for sensor fusion (kalman filter), can use this network. This mode can enable all devices connected to the internet.

   #### The discussion about wifi-infra mode
    問題: estimate_mode:=2 (つまりmocapのみ、kalman filterなし)の場合、上記のネットワークの方法では、飛行制御の不安定性がみられる。一方、PCをホストとするアドホックネットワーク場合(i.e. euclid)

    原因: 
    - 速度項を求める時の微分処理を行うときに時間のズレ＋ノイズによって発生していると考えられる。
    - intel euclidも内部アンテナを使っているため、5Ghz帯のwifiの速度があまり早くないと考えられる。

    対処: 
    - estimate_mode:=2を必要とする場合(e.g. Dragonの場合)は、上記の方法ではなくPCをホストと古いアドホックネットワークの方法を使う。今のDragonはそうしている。その場合、mocapのlaunch fileは手元のリモートPC上で上げる。
    - Jetson TX2で確かめる。
    - estimate_mode:=0と1はsensor fusion をしているので、上記の問題が起きない。今のHydrusはそういう状況。


### 2. Ad-hoc mode (Old)
The remote pc is directly connected with on-board processor (which is the host of this ad-hoc mode). For `estimate_mode = 2`, which is the mode only depends on the motion capture, should only use this network. For instance, the current dragon robot can only use Ad-hoc mode for motion capture.

**note**: related issue: https://github.com/tongtybj/aerial_robot/issues/179