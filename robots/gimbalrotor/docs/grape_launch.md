# Grape アーム付き機体の起動手順

`develop/grape_addfanguard` をビルドした ROS 1 の catkin ワークスペースを使う。各端末で作業環境を読み込む。パスは自分の環境に合わせる。

```bash
source ~/ros/jsk_aerial_robot_ws/devel/setup.bash
rospack find gimbalrotor
```

## キーボード操作

Gazebo での手動操縦と実機の通常操縦は、普段どおり `aerial_robot_base` のキーボード操作を使う。機体の起動後、別端末で実行する。

```bash
rosrun aerial_robot_base keyboard_command.py
```

## シミュレーション

### アーム構成を選ぶ

`airframe:=grape_with_arm` で `with_arm:=true` の場合、既定の `arm_variant:=original` がこれまでの構成を選ぶ。省自由度版を使うときは、実機・Gazebo それぞれの `bringup.launch` に `arm_variant:=compact` を追加する。

省自由度版は機体から **200 mm パイプ → yaw → pitch → ドライブ → グリッパ** の順。`bringup.launch` は `arm_variant:=compact` のとき [Servo_compact_arm.yaml](../config/grape_with_arm/Servo_compact_arm.yaml) を読み込む。`original` は `Servo.yaml`、アームなしは `Servo_no_arm.yaml` を使う。

| 省自由度版の軸 | URDF の関節名 | ServoBridge の `id` |
| --- | --- | --- |
| yaw | `arm_joint3_joint1` | 7 |
| pitch | `arm_joint4_joint1` | 8 |
| グリッパ駆動 | `arm_joint5_joint1` | 9 |

YAML の `id` は Spinal が ROS に出す `/gimbalrotor/servo/states` の `index` に対応する。表の 7、8、9 は省自由度版の実機で確認済み。配線や構成を変更した場合は、起動後に設定値と実機の値を確認する。

```bash
rosparam get /gimbalrotor/servo_controller/joints
rostopic echo -n 1 /gimbalrotor/servo/states
```

先端保持の自動評価は従来構成を使う。`with_arm:=false` ではアームなしになる。

### Gazebo の機体だけを起動する

```bash
roslaunch gimbalrotor bringup.launch \
  airframe:=grape_with_arm with_arm:=true \
  simulation:=true real_machine:=false headless:=false
```

`headless:=true` で GUI を省略できる。

省自由度版を起動する場合は次のコマンドを使う。

```bash
roslaunch gimbalrotor bringup.launch \
  airframe:=grape_with_arm with_arm:=true arm_variant:=compact \
  simulation:=true real_machine:=false headless:=false
```

### 省自由度版のアームを動かす

`arm_variant:=compact` で Gazebo を起動し、非武装のまま別端末で実行する。

```bash
rosrun gimbalrotor arm_set_pre_hover_pose_compact.py
```

指令角の順は yaw、pitch、グリッパ駆動、単位は rad。`[0.3, 0, -3.0]` と `[-0.5, -0.2, -3.0]` を続けて送り、以後は既定で1.5秒間隔で `[0.5, -0.2, -3.0]`、`[0, 0, -3.0]` を送る。待ち時間は `_move_time:=2.0` のように指定できる。

### 先端保持を自動評価する

Gazebo を別に立ち上げていない状態で実行する。この launch が Gazebo の起動、離陸、アーム固定と先端補償ありの比較、着陸まで行う。

```bash
roslaunch gimbalrotor arm_world_eval.launch \
  scale:=0.25 output:=/tmp/arm_world_eval
```

結果は指定したディレクトリの `summary.json`、`samples.csv`、`comparison.png` に出る。抽出済みの波形がブランチに含まれている。GUI を表示する場合は `headless:=false`、旧試験の追加速度制限を使う場合は `joint_speed_cap:=0.6` を付ける。すでに Gazebo を立ち上げている場合は、[設計書の評価ノード単体の実行例](arm_world_hold.md#実行)を使う。

## 実機

### 機体とアームを起動する

```bash
roslaunch gimbalrotor bringup.launch \
  airframe:=grape_with_arm with_arm:=true \
  simulation:=false real_machine:=true
```

省自由度版の実機では `arm_variant:=compact` を追加する。この設定ではフライトコントローラへのシリアル接続（`/dev/flight_controller`）、mocap、Livox MID-360、FAST-LIO を起動する。既定の ROS 名前空間は `/gimbalrotor`。先端保持ノードは従来構成で使う場合に別端末から起動する。

実機側で `rospack find livox_ros_driver2` と `rospack find fast_lio` が通ることを確認する。手元のシミュレーション用ワークスペースでは `fast_lio` が見つからず、実機用 launch の展開はそこまでの確認に留まった。

with_arm:=falseでアームなしのジンバルローターになるのはシミュレーションと同様だが、アームがつながっていないとCANのエラーが多分出るけど、一応大丈夫らしい。
自分はその時は念の為neuronのCANを抜いてエラーが出ないようにしつつ試していた。


### アームの姿勢を送る

以下の2本は `with_arm:=true arm_variant:=original` の機体に関節角を一度だけ送る。各値は第1軸から第5軸（グリッパ）までの順で、単位は rad。姿勢を送る前にアームの周囲に干渉がないことを確認する。

| 用途 | コマンド | 指令角 |
| --- | --- | --- |
| アーム4軸のゼロ姿勢（旧 `arm_init.py`） | `rosrun gimbalrotor arm_set_zero_pose.py` | `[0, 0, 0, 0, 3.0]` |
| ホバリング前の姿勢（旧 `arm_init2.py`） | `rosrun gimbalrotor arm_set_pre_hover_pose.py` | `[0.4, -1.57, -1.15, 0, -3.0]` |

旧 `arm_move.py` は、決め打ちの中間姿勢を既定で1.5秒間隔で送り、最後にグリッパを含む全5軸をゼロにする動作試験用だった。過去の動作シーケンスはコミット `dc13115f` を参照。通常のホバリング前には、表の一回送信用スクリプトを使う。

### 先端保持を試す

このブランチで先端保持を確認したのは Gazebo まで。実機で有効にする前に、アームの干渉と可動域、TCP、観測座標から `fc` への変換を確認する。以下は mocap の観測姿勢が `fc` と一致し、`header.frame_id` が `world` の例。異なる場合は[観測入力と実機への移行](arm_world_hold.md#観測入力と実機への移行)に沿って引数を合わせる。

```bash
rosrun gimbalrotor arm_world_eval.py --control-only \
  --pose-topic /gimbalrotor/mocap/pose --pose-type pose --world-frame world
```

先端保持は無効の状態で起動する。ホバリングが整定してから有効化すると、その時点のTCPを目標にする。

```bash
rosservice call /arm_world_hold/set_enabled 'data: true'
```

先端保持を解除する場合は以下を実行する。

```bash
rosservice call /arm_world_hold/set_enabled 'data: false'
```
