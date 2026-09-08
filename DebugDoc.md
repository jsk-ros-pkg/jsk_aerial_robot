# Spinal / Neuron Servo 调试记录

## 修复基线与原则

- 可工作的固件运行时基线：`9cc1dd7f028274a90faccf5d9ba1a1fa77887851`。
- `debug` 分支 `c9e2734347cd7492fc2ff51fa6158682877e5fbb` 相对该基线只修改两个 STM32CubeIDE `.cproject`，没有固件运行时代码变化。
- 本次修复以该基线的启动顺序、RTOS 边界和 Classic CAN 短帧行为为基础，再移植 DirectServo 的配置事务与 CAN FD servo 功能；不是在故障路径上继续叠加临时补丁。
- 保持最小改动：没有新增 RTOS task、queue、mutex、semaphore 或修改任务优先级；复用已有 `ServoTask` 和 CAN Mail Queue。
- 修复顺序：Neuron Dynamixel -> CAN 传输 -> 枚举/重连 -> 统一接口 -> 自动配置 -> RQT/上位机。
- 每层均先增加可复现失败的检查，再修复并单独验证。

## 问题统计

共记录 12 组问题：代码级已解决 7 项、构建环境已解决 1 项、部分解决且待硬件确认 3 项、已排除 1 项。

| 编号 | 层级 | 问题与根因 | 修复结果 | 状态 |
|---|---|---|---|---|
| D01 | RTOS/CAN | `Spine::useRTOS()` 曾在 CAN Mail Queue 创建前使用空句柄，导致无法枚举未修改 Neuron | 恢复 debug 启动边界：非 RTOS 初始化和枚举先完成，RTOS 对象创建后才调用 `useRTOS()` | 已解决 |
| D02 | ROS/UI | Neuron servo 配置字段没有完整进入统一 `BoardInfo`，RQT 只能看到直连 servo 的配置 | Spinal 统一生成 Board 0 和全部 Neuron 的完整 `BoardInfo`；RQT 展示相同字段 | 已解决 |
| D03 | rosserial | checksum/topicId 异常来自消息定义、MCU `ros_lib` 和服务注册不同步，以及服务端点未完全建立时过早发布 | 已重生成三块 H7/F7 `ros_lib`，请求/响应 MD5 一致；增加 topicId 范围和服务端点就绪检查 | 部分解决：编译和静态验证通过，仍需板上长时间串口验证 |
| D04 | CAN | 当前固件与 debug 短帧格式不兼容，且共享 TX header/buffer 会跨任务竞争 | `DLC <= 8` 恢复 debug 的 Classic CAN/BRS-off；长帧使用 FD+BRS；接收端兼容这两类合法格式；TX 数据改为调用局部变量 | 部分解决：枚举短帧可滚动兼容；64-byte servo 协议仍要求 Spinal/Neuron 成套升级 |
| D05 | 自动配置 | 全量关闭 torque、不恢复；失败后循环重试并永久阻止 target/current/enable | 删除上层全量关断、torque 缓存恢复、无限重试和控制门禁；自动配置只执行一次，失败后控制仍可用 | 已解决 |
| D06 | Neuron 配置 | 只有先写 `bus_type` 才创建 Flash 记录，导致 servo index 1/2/3 的其他配置返回 INVALID | 任意单 servo 配置都可先创建暂存记录；只有 enable/目标控制要求 bus+mode 完整 | 已解决 |
| D07 | 反馈频率 | Neuron 只允许 200 Hz，而且 `servo_feedback_frequency` 不控制 ROS 输出 | 接受 `1..200 Hz`，Spinal 按每个 board 的频率独立决定其 servo 状态进入 `/servo/states` 的周期 | 已解决 |
| D08 | Torque/目标 | 目标帧先检查旧 torque、后处理 enable，首帧目标被丢；移植代码还错误地让 Neuron servo 收到 P/V/C 后自动 enable | 保持 debug 语义：Neuron servo 的 command/feedback 分离，P/V/C 只更新目标，只有 `/servo/torque_enable` 可主动 enable；DirectServo 保留目标自动 enable；Neuron 在同一 CAN FD 帧中先应用 torque、再接受目标 | 已解决 |
| D09 | Board 0 | 直连舵机枚举失败后 `DirectServo*` 置空，同时丢失已读取的板级配置，导致 Board 0 错报 `0/0/false`；空板配置还只修改临时缓存并伪成功 | 将只读配置快照与 UART 控制指针分离；空板报告启动阶段的默认值/Flash 值、拒绝配置写入，主机自动配置跳过无舵机 board | 已解决 |
| D10 | 扩展性 | 多 servo 配置和 Dynamixel 事务可能阻塞 CAN/上位机 | CAN RX 只入队，UART/Flash 在已有 ServoTask 执行；IMU/Motor CAN TX 不再等待 DXL；上层服务按间隔串行调用 | 部分解决：协议上限 5 boards × 4 servos 已覆盖，仍需满载硬件压力测试 |
| D11 | 配置可靠性 | 多个 setter 无返回值，UART 写失败后缓存仍可能与请求一致并返回成功 | homing/PID/profile/current/mode 均验证写入结果；失败返回错误并回滚缓存 | 已解决 |
| D12 | Docker 构建 | 使用 Docker 默认 root 用户生成三个板型的 `ros_lib`，使目录和 `State.h` 成为 `root:root`，普通用户执行 `catkin build` 时无法删除旧目录 | 只恢复 STM32F7、STM32H7、STM32H7_v2 三个 `ros_lib` 目录为 `wentao:wentao`；随后以 UID 1000 重跑最小和完整构建 | 已解决；后续 Docker 构建必须使用普通用户 |

## 最终结构

### 1. Neuron Dynamixel 与 RTOS

- CAN RX callback 只解析并投递一个配置事务，忙时明确返回 `BUSY`。
- 已有 1 ms `ServoTask` 执行 UART 和 Flash；没有创建新的同步资源。
- Dynamixel 包长度、PING 最大数量和重复 ID 均有边界检查。
- 一只未配置 servo 不再阻止同板其他已配置 servo 控制。
- 需要 torque-off 的寄存器写仍由 MCU 返回 `BUSY`，安全约束没有放宽。

### 2. CAN 传输与枚举

- 短帧沿用 debug Classic CAN，CAN FD 大数据使用 FD+BRS。
- TX header 和 64-byte payload 为调用局部对象；仅在 HAL 入队的短区间关中断。
- TX FIFO、RX 格式、HAL 读取和 Mail Queue 丢帧均有计数。
- Spinal CAN Mail Queue 从 10 增至 20，可容纳一个 4-servo 完整配置快照（17 帧）。
- 初始化配置先写入固定大小 staging；只有版本、DLC、index 和 bitmap 全部完整后才提交。
- 初始枚举和运行时重连均有限次重试；失败快照不会污染 `BoardInfo`。

### 3. Board 与 servo 索引

- `/get_board_info` 中 Board 0 永远表示 Spinal；Board 1..14 对应 Neuron CAN ID。
- `ServoControlCmd.index` 仍是全局运行时索引：直连 servo 在前，其后按
  `/get_board_info` 的 board 返回顺序和板内 slot 排列。
- DirectServo 收到目标后保持 debug 的自动 enable 行为；Neuron servo 必须通过 `/servo/torque_enable` 显式 enable，普通 CAN 反馈不能覆盖 torque command。
- YAML、反馈和控制统一使用全局 servo `id`；ServoBridge 按 `/get_board_info`
  返回的板和板内 servo 顺序解析为内部 `board_id + servo_index` 地址。
- `servo_index` 只作为板内配置服务的内部地址；`send_data_flag=false` 的 servo
  仍保留在 `BoardInfo.servos[]` 和全局索引中。
- 物理 Dynamixel ID 仅用于 MCU 内部查表和 Flash key，不再由 YAML、RQT 或服务 payload 传递。

### 4. 自动配置

- YAML 每个 servo 只给全局 `id`，不暴露 `board_id`、板内 `servo_index` 或物理 Dynamixel ID。
- 先读取全部 boards 并解析全局索引，再依次配置 board 参数、bus、mode 和可选参数；服务调用间隔 20 ms、启动延迟 1 s 固定在 ServoBridge 内部。
- Board 0 没有 servo 时仍报告启动阶段读取到的 board 配置，但不会接收配置写入；ServoBridge 跳过所有无 servo board 的板级自动配置。
- 不要求 YAML servo 数量等于硬件总数，因此可以只管理其中一部分 servo。
- 自动配置使用 one-shot timer；失败会报错并结束，不会持续重试或永久阻止控制命令。

## 分层验证记录

| 层级 | 验证内容 | 结果 |
|---|---|---|
| V1 Neuron Dynamixel | 配置入队、BUSY、setter 回滚、无 bus-first 硬依赖、单 servo 隔离、包边界、首帧 torque 顺序 | 静态回归通过 |
| V2 CAN | 局部 TX 对象、发送失败返回、TX/RX drop 计数、Classic/FD 格式、20-slot queue、完整快照提交与重试 | 静态回归通过 |
| V3 索引/API | YAML 全局 `id` 自动解析为板内地址、MCU 内部解析物理 ID、`send_data_flag=false` 索引稳定 | 静态回归通过 |
| V4 自动配置 | one-shot、固定服务间隔、无全量 torque 关断、失败不锁控制、YAML 全局 ID 寻址 | 静态回归通过 |
| V5 RQT | Neuron 配置展示、12 类配置 payload、homing calibration 统一服务 | ROS 消息类型测试通过 |
| V6 ROS 构建 | 普通用户执行最小 `spinal` 构建及完整 `catkin build` | `spinal` 通过；完整工作空间 51/51 packages 成功，只有两个无关包的既有 warning |
| V7 MCU 构建 | Neuron G4、Spinal H7、Spinal H7_v2 | 3/3 成功；只有仓库既有 warning |
| V8 板上运行 | 上电枚举、自动配置回读、首目标 enable、50/200 Hz、满载、rosserial checksum | 部分通过；H7 Spinal 已烧录并校验，复位后 Board 0 在无直连 servo 时正确报告 `50 Hz / 0.1 s / mixed=true`，Neuron Board 1 和四个 servo 正常枚举；其余压力测试待完成 |

当前完整回归命令在 ROS Noetic 容器中运行，覆盖 55 项检查：

```bash
source /opt/ros/noetic/setup.bash
source /home/wentao/aerial_ws/devel/setup.bash
cd /home/wentao/aerial_ws/src/aerial_robot
python3 -m unittest discover -s aerial_robot_nerve/spinal/test -p 'test_*.py'
```

Docker 中不要以默认 root 身份生成源码树内的 `ros_lib`。从宿主机执行时使用：

```bash
docker exec -u 1000:1000 -it noetic bash
```

## 下一轮硬件验证顺序

1. 只连接 Spinal + 一个未修改 Neuron，上电确认枚举和 LED 通信状态。
2. 调用 `/get_board_info`，确认 Board 0 和 Neuron board 均存在、servo slot 和物理 ID 对应正确。
3. 关闭自动配置启动 bridge，确认 CAN 和 rosserial 长时间稳定。
4. 打开自动配置，确认服务只执行一轮，逐项 readback 正确，CAN 不锁死。
5. 分别写 servo index 0/1/2/3 的 bus、mode、PID、profile、current、send flag。
6. Neuron servo torque-off 后持续发送目标，确认仍保持关闭；显式发布 `/servo/torque_enable` 后确认 enable 且首帧目标未丢。
7. 分别设置 50 Hz 和 200 Hz，测量对应 board 的 `/servo/states` 输出频率。
8. 接入最大计划 servo 数量，持续观察 CAN TX/RX drop、rosserial checksum 和控制周期。
