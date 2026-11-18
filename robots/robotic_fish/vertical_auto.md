# Vertical auto

`vertical_auto.py` 是独立的 onboard 控制器，手动部分复制自 `advance_teleop.py`，
保留巡航、位置控制、加减速、快速启动和快速转向按键。旧脚本和 launch 保持原样。

## 启动

在已构建并 source 本 ROS 工作空间的环境中运行。ROS master 独立运行在 onboard，
上位机的 `ROS_MASTER_URI` 指向 onboard；两台机器的 ROS 地址需互相可达。
onboard 本地节点互通不应依赖上位机或水下断开的网络路径。
`vertical_auto.launch` 不再包含 Spinal bridge，也不再接收 `bridge`、`dev` 参数。
master、bridge 和控制器分别启动，使控制器 launch 退出时不结束它们：

```bash
# Onboard，终端 1：独立保持 master 运行。
roscore

# Onboard，终端 2：独立启动 spinal bridge。
roslaunch spinal bridge.launch dev:=vim4

# Onboard，终端 3：只启动控制器。
roslaunch robotic_fish vertical_auto.launch auto_reversals:=6

# 上位机：只启动蓝牙手柄对应的 joy_node。
roslaunch robotic_fish vertical_auto.launch role:=joystick

# 本机 USB 联调：独立 master 保持运行，以下两条分别在不同终端启动。
roslaunch spinal bridge.launch dev:=pc
roslaunch robotic_fish vertical_auto.launch role:=all

# 下潜 prepare 阶段直接给 motor 摆动转速；默认 false，保持两轴位置控制。
roslaunch robotic_fish vertical_auto.launch auto_prepare_fast_start:=true
```

只运行一个电机指令发布程序；不要同时启动 `advance_teleop.launch`。
`role:=joystick` 已设置 50 Hz 手柄重复发布，支持轴方向持续检测和手动输入超时。

## 手动和自动

- `axes[9]` 同一非零方向连续保持 3 秒进入自动：`+1` 从两轴 1024 开始准备，
  `-1` 从两轴 3072 开始准备。回零、换向或输入中断会重置触发计时。
  一次持续输入只触发一次；触发后需要先回零才能再次启动。`buttons[13]` 不再触发自动。
- `axes[10]=+1` 持续 3 秒触发 `vertical_up_auto`：两轴定位 1024 保持 10 秒，
  随后 servo 继续保持 1024，motor 按现有 1024 侧方向转动 20 秒，再停止回中。
  若两个触发轴同时有效，上升模式优先；切换触发模式会重新计时，触发后需两轴均释放
  （`axes[9]=0` 且 `axes[10]!=+1`）才能再次触发。
- 下潜启动需要新鲜的 IMU、servo 和 motor 反馈；默认允许的反馈间隔为 0.5 秒。
  上升是定时动作，只要求 servo、motor 反馈新鲜，不使用 IMU 窗口切换。
- 两种自动模式执行期间都不依赖 `/joy` 或 `/cmd_vel`；收到 `buttons[1]` 可随时中止，
  包括上升准备和 motor 转动阶段。
- 自动结束或中止后回到手动 `stop` 状态，清除旧巡航速度、加速度和转向量。
  servo、motor 各自位置控制回校正坐标 2048；只要执行器反馈有效，回中控制持续运行，
  不依赖手柄重新连接或 IMU。到位后速度归零，仍保持位置控制。
- 手动输入超过 0.5 秒未更新，进入上述回中停止状态；自主任务中手柄断联不终止任务。
- 控制程序退出时尝试发布零速度；这不替代 MCU/驱动侧的命令超时保护。

## 代码结构

控制代码位于 [scripts/vertical_auto.py](scripts/vertical_auto.py)，由 `VerticalAuto` 类统一保存
反馈、状态和待发布指令。启动时注册 ROS 回调及 50 Hz 定时器。

| 部分 | 函数 | 作用 |
|---|---|---|
| 反馈 | `_servoStateCallback()`、`_motorStateCallback()` | 更新两轴角度、motor 偏置换算和反馈时间 |
| 姿态观测 | `_imuCallback()` | 四元数转鼻尖方向、roll 几何，差分及低通处理，更新峰值观测 |
| 手柄 | `_joyCallback()` | 处理按键和摇杆，计算手动指令，记录触发轴方向和保持时间并发布 |
| 外部速度 | `_twistCallback()` | 接受 `/cmd_vel` 的 motor 速度；自主机动期间不接受该输入 |
| 定时调度 | `_controlLoop()` | 检查轴方向保持时间、反馈和输入超时，执行自动动作或回中，再发布指令 |
| 自主任务 | `start_vertical_auto()`、`vertical_control()`、`start_vertical_up_auto()`、`vertical_up_control()`、`finish_vertical_auto()` | 启动、运行和结束下潜/上升状态机 |
| 反摆窗口 | `update_vertical_peak()`、`vertical_reverse_ready()` | 检测当前/预测 roll 峰及姿态许可 |
| 执行器控制 | `pos_control()`、`manual_control()`、`stop_control()` | 根据位置或巡航目标计算速度指令 |
| 指令发布 | `publish_cmd()` | 更新巡航 motor 速度，并包装、发布两轴指令 |

主要调用路径：

```text
/joy       -> _joyCallback()   -> 位置/巡航控制 -> publish_cmd()
/cmd_vel   -> _twistCallback() -> motor 速度    -> publish_cmd()
50 Hz 定时 -> _controlLoop()   -> 自动/回中控制 -> publish_cmd()
/imu、两轴状态 -> 更新观测和反馈 -> 供上述控制函数读取
```

回调和定时器共享 `servo_cmd`，并使用 `control_lock` 串行执行。锁只保证一次执行一个
回调；控制分支仍需明确互斥，避免不同回调先后发布不同目标。

### 状态变量

| 变量 | 含义 |
|---|---|
| `auto_pattern` | `stop / cruise / fast_start / fast_turn / vertical_auto / vertical_up_auto`，当前动作类别 |
| `auto_control_mode` | 定时器是否调用 `auto_control()`；包含旧快速动作和新自主任务 |
| `motor_pos_control` | R1 或 prepare 位置控制标志；下潜 prepare 加速起步时为 False，上升 prepare 时为 True |
| `vertical_auto_mode` | 是否正在执行下潜或上升自主任务，共用手动屏蔽和停止逻辑 |
| `vertical_state` | 新任务内部的 `manual / prepare / reverse / gait / up` |
| `vertical_trigger_mode` | 当前候选触发模式：空值、`vertical_auto` 或 `vertical_up_auto` |
| `vertical_trigger_axis` | 下潜触发方向：+1 准备 1024，-1 准备 3072；上升触发时置 0 |

R1 位置控制不改变 `auto_pattern`，而是保持先前的 `stop` 或 `cruise`，同时将
`motor_pos_control` 设为 True。原快速起步、快速转向与立体机动均由 `auto_control()` 分发。

### 手柄映射

| 输入 | 动作 |
|---|---|
| `buttons[0]` | 开始巡航，初始 motor 速度为 3π/s |
| `buttons[1]` | 停止/中止自动，两轴回中 |
| `buttons[2]` | 初始化 yaw 和 motor 参考；当前要求执行器反馈新鲜、IMU 有效 |
| `buttons[3]` | 原快速起步 |
| `buttons[5] + axes[2]` | R1＋摇杆，两轴使用同一个逻辑位置目标 |
| `buttons[4] + buttons[5] + axes[2]` | 原快速转向 |
| `buttons[6]/[7] + axes[3]/[4]` | 调整巡航速度 |
| `axes[10]` | 保留原来的 π/s 巡航速度步进；+1 持续 3 秒后进入上升模式 |
| `axes[9]` 同一非零方向保持 3 秒 | 启动新自主任务；+1 准备 1024，-1 准备 3072 |

### 指令计算与发布

`servo_cmd[0]` 和 `servo_cmd[1]` 保存的是 servo、motor 的速度指令，不是位置目标。
`pos_control(servo_no, target, rate)` 根据编码器位置误差生成速度，包含环形角度处理、
比例控制和 50 tick 死区。

`publish_cmd()` 在巡航且 `motor_pos_control=False` 时重新计算 motor 速度，叠加
`motor_vari()` 的相位变速；其他状态直接使用已有指令。servo 指令发布到
`/servo/target_states`，motor 指令以 `mode=1`、`cmd=-servo_cmd[1]` 发布到
`/servo/extended_cmds`。位置控制也通过这套速度指令接口执行。

### 定时器顺序与 R1 互斥

`_controlLoop()` 每 20 ms 按以下顺序执行：

1. 检查 `axes[9]` 下潜触发或 `axes[10]=+1` 上升触发是否保持 3 秒，并锁定任务。
2. 执行器反馈超时则结束自动（若正在执行）、输出零速度并返回。
3. 自动模式执行 `auto_control()`。
4. 否则，`auto_pattern == 'stop'` 且不在位置控制时执行回中。
5. 否则，手柄输入超时则进入停止状态。
6. 否则，巡航且不在位置控制时更新 servo 控制。
7. 发布指令。

R1 定位时，手柄回调按 `2048 - axes[2] * 1024` 计算目标（限制到 1024–3072），
对两轴调用 `pos_control()` 并发布。定时器的回中分支必须使用：

```python
elif self.auto_pattern == 'stop' and not self.motor_pos_control:
    self.stop_control()
```

此前该分支只检查 `stop`，会在 R1 定位期间交替发布摇杆目标和回中目标 2048，导致抖动。
本次修正只增加 `not self.motor_pos_control`，保留其他调度、超时和发布逻辑。
当输入及反馈正常时，定时器可重复发布 R1 已算出的指令，但不会重新计算回中目标。

## 下潜状态机

`prepare -> reverse -> gait <-> reverse -> manual stop`

| 状态 | 动作 / 切换 |
|---|---|
| `prepare` | 默认两轴都定位到 1024 或都定位到 3072，保持 4 秒；加速起步开关为 True 时仅 servo 定位，motor 直接按准备侧方向以 `vertical_motor_speed` 转动 |
| 初始 `reverse` | 从准备侧切向另一侧，motor 按新侧以 ±4π/s 转动；至少经过 0.5 秒且 servo 距目标不超过 50 tick 后进入 `gait`，这次准备反摆不计数 |
| `gait` | 保持本侧 servo 目标和固定 motor 转速；四元数观测满足 roll 窗口时切到另一侧 |
| 后续 `reverse` | 同一控制周期翻转 servo 目标和 motor 方向；至少经过 0.5 秒且 servo 到位后计为一次完成的反摆 |
| 完成 | 达到 `auto_reversals`（默认 6）后回手动 `stop`，servo、motor 持续位置控制回各自的 2048 |

准备总时长由 `auto_prepare_seconds` 单独指定，默认 4 秒；不再叠加 `auto_position_seconds`。
`auto_prepare_fast_start` 默认 False。False 时，准备结束要求两轴均距位置目标不超过
50 tick；True 时 motor 已连续转动，只检查 servo 到位，不等待 motor 的位置目标。
加速起步的 motor 内部速度为 `side * auto_motor_direction * vertical_motor_speed`。
后续反摆完成条件只检查最短持续时间和 servo 到位，不检查 motor 累计转角。

| 触发方向 | 准备侧 | servo / motor 逻辑目标 | 初始反摆后的 servo 目标 / motor 内部速度 |
|---|---|---|---|
| `axes[9]=+1` | `side=-1` | 1024 / 1024 | 3072 / +4π/s |
| `axes[9]=-1` | `side=+1` | 3072 / 3072 | 1024 / -4π/s |

上表 motor 速度按默认 `auto_motor_direction=1`；实际发布仍统一取负。
进入任务后，准备侧已锁定，后续摇杆变化不会改写任务侧别。

单侧等待超过 20 秒或反摆超过 2.5 秒未到位，也终止自动并回手动。
IMU 无效、IMU 超时或执行器反馈超时会终止自动。
准备结束后再过 2.5 秒仍未定位，也终止自动。执行器反馈超时期间输出零速度，
反馈恢复后继续 `stop` 回中。
下潜任务结束按反摆计数判断，上升任务按持续时间结束，均没有水面检测判据。

`side=+1` 对应 servo=3072、控制器内 motor=+4π/s；`side=-1` 对应
servo=1024、motor=-4π/s。最终发布保留旧程序的 `motor.cmd = -self.servo_cmd[1]`。
如需交换电机物理方向，设置 `auto_motor_direction:=-1`，servo 目标不变。
`reverse` / `gait` 使用恒定速度，不叠加旧巡航的相位变速。
默认 `prepare` 和停止时使用 R1＋摇杆同一个 `pos_control()`，以反馈位置误差生成速度，
到位后速度归零。位置目标不受 `auto_motor_direction` 影响；开启准备加速起步后，
motor 转速方向使用该参数。

motor 的位置目标使用原程序的校正坐标：

```text
motor_equ_angles = (motor_angle_raw - motor_angle_ref + 2048) % 4096
motor_angle_raw_target = (logical_target + motor_angle_ref - 2048) % 4096
```

`motor_angle_ref` 默认 720：准备目标 1024 对应原始编码器 3792，
准备目标 3072 对应原始编码器 1744，停止目标 2048 对应原始编码器 720。
偏置已在反馈换算中处理，位置目标不重复加减参考值，也不额外取负；方向沿用原程序发布
速度时的负号。原 R1＋摇杆和自动准备均给两轴同一个逻辑位置目标。
可通过 `motor_angle_ref:=720` 修改默认参考；原初始化按键 `buttons[2]` 仍将当前 motor
原始角度设为新的参考零位。

servo 编码器方向和实际弯曲方向仍需真机核对；这里约定正侧对应身体向左弯。
准备阶段为 4 秒，旧的 `auto_position_seconds` 参数已移除。执行动作使用真机的位置控制
和 motor 速度控制，
不是仿真关节轨迹的精确复现。

## 螺旋上升状态机

`vertical_up_auto: prepare (10 s) -> up (20 s) -> manual stop`

| 阶段 | servo | motor | 切换 |
|---|---|---|---|
| `prepare` | 位置目标 1024 | 校正位置目标 1024 | 进入模式 10 秒后切换 |
| `up` | 继续位置控制在 1024 | 沿用 1024 侧方向：内部速度为 `-auto_motor_direction * vertical_motor_speed`，默认 −4π/s | 转动满 20 秒后停止 |
| 停止 | 回到 2048 | 校正坐标回到 2048 | 也可通过 `buttons[1]` 随时进入 |

上升的准备时间和转动时间分别由 `auto_up_prepare_seconds=10`、`auto_up_seconds=20`
控制，不受下潜 prepare 的 4 秒设置或 `auto_prepare_fast_start` 开关影响。
上升不反摆、不统计反摆次数，也不根据 IMU 判断已经到达水面；servo 始终保持同侧目标，
直到停止时回中。最终 motor 发布值仍统一取负，默认上升期间发布 +4π/s。

## 四元数判断

输入 `/imu` 为 `spinal/Imu`，四元数顺序 `[x, y, z, w]`，按身体到世界的旋转解释，
身体 +X 向前、+Y 向左、+Z 向上。使用完整四元数，不使用旧程序的 yaw 参考角来判断反摆。
角度变化率由相邻四元数观测差分获得，不使用 `gyro`。

反摆窗口沿用仿真 `roll_sync` 的姿态部分：鼻尖角许可锁存（≤88°）、最短本侧摆动
1.5 秒、有效 roll 几何、至少 25° 有利 roll、向下投影 ≥0.2、鼻尖角 ≥70°，
以及当前或预测 roll 峰窗口连续确认 0.04 秒。鼻尖角以世界向下为 0°、水平为 90°。
保留原低通时间常数和峰值窗口阈值；移除仿真的 J1 偏转角条件和关节 PD。
roll 几何在鼻尖接近竖直时无效，不用于触发新反摆。

这版是待验证的真机适配，没有宣称能够可靠下潜或上浮。IMU 方向、左右弯曲对应关系、
实际反摆延迟，以及差分噪声下的峰值窗口，都需要通过真机记录确认。

## 离线检查

```bash
# 仓库根目录执行，不连接或驱动硬件。
python3 robots/robotic_fish/test/test_vertical_auto.py
```
