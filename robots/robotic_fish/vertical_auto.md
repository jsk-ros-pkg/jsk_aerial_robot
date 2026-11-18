# Vertical auto

`vertical_auto.py` 是独立的 onboard 控制器，手动部分复制自 `advance_teleop.py`，
保留巡航、位置控制、加减速、快速启动和快速转向按键。旧脚本和 launch 保持原样。

## 启动

在已构建并 source 本 ROS 工作空间的环境中运行。ROS master 放在 onboard，
上位机的 `ROS_MASTER_URI` 指向 onboard；两台机器的 ROS 地址需互相可达。
onboard 本地节点互通不应依赖上位机或水下断开的网络路径。

```bash
# Onboard：启动控制器和 spinal bridge，默认 dev:=vim4。
roslaunch robotic_fish vertical_auto.launch auto_reversals:=2

# 上位机：只启动蓝牙手柄对应的 joy_node。
roslaunch robotic_fish vertical_auto.launch role:=joystick

# 如果 onboard 已经单独运行 spinal bridge：
roslaunch robotic_fish vertical_auto.launch bridge:=false auto_reversals:=2

# 本机 USB 连接 Spinal 和手柄时：
roslaunch robotic_fish vertical_auto.launch role:=all dev:=pc
```

只运行一个电机指令发布程序；不要同时启动 `advance_teleop.launch`。
`role:=joystick` 已设置 50 Hz 手柄重复发布，支持持续长按检测和手动输入超时。

## 手动和自动

- `buttons[13]` 连续长按 3 秒进入自动；松开、输入中断会重置长按计时。
  一次长按只触发一次，再次启动需要先松开。
- 自动启动需要新鲜的 IMU、servo 和 motor 反馈；默认允许的反馈间隔为 0.5 秒。
- 自动执行期间不依赖 `/joy` 或 `/cmd_vel`；收到 `buttons[1]` 仍可中止。
- 自动结束或中止后回到手动 `stop` 状态，清除旧巡航速度、加速度和转向量。
  servo、motor 各自位置控制回校正坐标 2048；只要执行器反馈有效，回中控制持续运行，
  不依赖手柄重新连接或 IMU。到位后速度归零，仍保持位置控制。
- 手动输入超过 0.5 秒未更新，进入上述回中停止状态；自主任务中手柄断联不终止任务。
- 控制程序退出时尝试发布零速度；这不替代 MCU/驱动侧的命令超时保护。

## 自动状态机

`prepare -> reverse -> gait <-> reverse -> manual stop`

| 状态 | 动作 / 切换 |
|---|---|
| `prepare` | servo 定位 1024、motor 定位校正坐标 3072，并持续保持；默认准备时间为 2 秒定位窗口加 10 秒保持，到时还要求两者距目标均不超过 50 tick |
| 初始 `reverse` | servo=3072、motor=+4π/s；至少经过 0.5 秒且 servo 距目标不超过 50 tick 后进入 `gait`，这次准备反摆不计数 |
| `gait` | 保持本侧 servo 目标和固定 motor 转速；四元数观测满足 roll 窗口时切到另一侧 |
| 后续 `reverse` | 同一控制周期翻转 servo 目标和 motor 方向；至少经过 0.5 秒且 servo 到位后计为一次完成的反摆 |
| 完成 | 达到 `auto_reversals`（默认 2）后回手动 `stop`，servo、motor 持续位置控制回各自的 2048 |

单侧等待超过 20 秒或反摆超过 2.5 秒未到位，也终止自动并回手动。
IMU 无效、IMU 超时或执行器反馈超时会终止自动。
准备结束后再过 2.5 秒仍未定位，也终止自动。执行器反馈超时期间输出零速度，
反馈恢复后继续 `stop` 回中。
任务结束按反摆计数判断，没有水面检测或自动上浮判据。

`side=+1` 对应 servo=3072、控制器内 motor=+4π/s；`side=-1` 对应
servo=1024、motor=-4π/s。最终发布保留旧程序的 `motor.cmd = -self.servo_cmd[1]`。
如需交换电机物理方向，设置 `auto_motor_direction:=-1`，servo 目标不变。
`reverse` / `gait` 使用恒定速度，不叠加旧巡航的相位变速。
`prepare` 和停止时使用 R1＋摇杆同一个 `pos_control()`，以反馈位置误差生成速度，
到位后速度归零。准备阶段 motor 的位置目标不受 `auto_motor_direction` 影响。

motor 的位置目标使用原程序的校正坐标：

```text
motor_equ_angles = (motor_angle_raw - motor_angle_ref + 2048) % 4096
motor_angle_raw_target = (logical_target + motor_angle_ref - 2048) % 4096
```

`motor_angle_ref` 默认 `2048+720=2768`，因此准备目标 3072 对应原始编码器 3792，
停止目标 2048 对应原始编码器 2768。偏置已在反馈换算中处理，位置目标不重复加 720，
也不额外取负；方向沿用原程序发布速度时的负号。原 R1＋摇杆给两轴同一个逻辑目标，
自动准备按本次要求给 servo=1024、motor=3072 两个独立目标。
可通过 `motor_angle_ref:=2768` 修改默认参考；原初始化按键 `buttons[2]` 仍将当前 motor
原始角度设为新的参考零位。

servo 编码器方向和实际弯曲方向仍需真机核对；这里约定正侧对应身体向左弯。
准备阶段保持原来的 12 秒总时序；初始定位时间参数改名为 `auto_position_seconds`，
替代 `auto_straight_seconds`。执行动作已改成真机的双轴位置控制和后续 motor 速度控制，
不是仿真关节轨迹的精确复现。

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
