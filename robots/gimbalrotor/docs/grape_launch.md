# Grape と Quad2 の起動・動作確認

ビルド済みの ROS 1 catkin ワークスペースを各端末で読み込む。

```bash
source devel/setup.bash
rospack find gimbalrotor
```

## 機体と構成

| `airframe` | `variant` | 構成 |
| --- | --- | --- |
| `grape` | `default` | アームなし |
| `grape` | `3DoF_arm` | yaw、pitch、グリッパ駆動 |
| `grape` | `5DoF_arm` | 5軸アーム |
| `quad2` | `default` | Quad2 |

`bringup.launch` は `robots/<airframe>/` から実機用または Gazebo 用のモデルを読み込み、`config/<airframe>/<variant>/` からサーボ・制御設定を読む。

Grape と Quad2 のモデルは [urdf/grape/common.xacro](../urdf/grape/common.xacro) の共通マクロを使う。Grape のジンバルと推力リンクの STL は [urdf/grape/meshes/](../urdf/grape/meshes/) にある。Quad2 の機体モデルは [urdf/quad2/](../urdf/quad2/) にある。

Grape のジンバル質量・重心・慣性は、実機用・Gazebo 用とも補強ファンガード付きの値を使う。ジンバルは1基あたり 0.5050447 kg。アーム構成は `variant` で選ぶ。

## Gazebo

```bash
roslaunch gimbalrotor bringup.launch airframe:=grape variant:=3DoF_arm \
  simulation:=true real_machine:=false headless:=false
```

アームなしや5軸アームに切り替える場合は `variant` を変更する。Quad2 は `airframe:=quad2 variant:=default` を指定する。画面表示が不要なら `headless:=true` を指定する。

手動での操縦には別端末で `rosrun aerial_robot_base keyboard_command.py` を実行する。

### 飛行テスト結果

モデル配置を変更した `fd9f0ca5` では、Grape の全3構成と Quad2 の `default` で離陸、3秒間のホバリング、着陸を確認した。ホバリング中の最大水平ドリフトは全構成で 2 mm 以下だった。

## 実機

```bash
roslaunch gimbalrotor bringup.launch airframe:=grape variant:=3DoF_arm \
  real_machine:=true simulation:=false
```

実機の Grape では Livox MID-360 と FAST-LIO が起動する。アーム付きの構成では RealSense と UDTS も起動する。

## 動作確認と記録

アームとグリッパの試験はアーム付き Grape を起動した状態で、1つずつ実行する。試験は既定で URDF 上の可動域を端から端まで動かし、成功時に対象軸をゼロに戻す。小さな動きだけ確認する場合は `_full_range:=false` を指定する。

```bash
rosrun gimbalrotor arm_motion_test.py
rosrun gimbalrotor gripper_motion_test.py _full_range:=false
```

`robot_id` を指定して起動した場合は、アーム・グリッパの試験に `_robot_ns:=/gimbalrotor<robot_id>` を渡す。

ジンバルのコマンドは既定の `/gimbalrotor` 名前空間を使う。動作シーケンスを繰り返す回数は `_test_times` で指定する。

```bash
rosrun gimbalrotor gimbal_motion_test.py _test_times:=1
rosrun gimbalrotor gimbal_return_to_zero.py
rosrun gimbalrotor rosbag_record.sh grape_test
```
