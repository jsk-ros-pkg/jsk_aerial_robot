# Servo UART 最小修改方案

## 1. 目标

在不降低现有控制、Servo feedback 和 ROS 发布频率，也不重构底层任务框架的前提下，解决以下问题：

- Servo 配置 service 偶发没有响应。
- 周期性 Fast Sync Read 与 service UART 操作可能互相打断。
- UART command 与 response 不能始终保持一一对应。
- Service 可能因为无限等待而阻塞 `rosSpinTask`。

本方案继续使用现有 `instruction_buffer_`、`read_status_packet_flag_` 和 `instruction_last_`，不引入完整的通用 `pending_transaction_` 框架。

## 2. 核心原则

修改后必须满足以下约束：

1. 初始化完成后，只有 `DynamixelSerial::update()` 可以访问 Servo UART3。
2. 所有运行期 Servo UART 操作，包括 service，都必须进入 `instruction_buffer_`。
3. UART 同一时间最多存在一个等待 response 的 command。
4. 当前 response 完成或超时前，不允许发送下一条 UART command。
5. Service 只能有限等待，不能使用 `osWaitForever`。
6. 队列 mutex 只保护队列和 mailbox，不能覆盖 UART 收发或 service 等待过程。
7. 保持现有 200 Hz 控制、feedback 和 ROS 发布频率不变。

## 3. 保留现有 pending 机制

现有字段已经表达了一个简化的 pending command：

```cpp
bool read_status_packet_flag_;
std::pair<uint8_t, uint8_t> instruction_last_;
```

其含义为：

```text
read_status_packet_flag_ == true
    当前存在一条已经发送、但尚未完成 response 解析的 command。
```

因此不需要再增加一个通用的 `pending_transaction_` 对象，只需要补全现有机制的执行约束。

## 4. UART command 处理流程

```text
ROS callback / service
          │
          ▼
  instruction_buffer_
          │
          ▼
DynamixelSerial::update()
          │
          ├── 无 response：发送完成后可以继续处理
          │
          └── 有 response：记录 instruction_last_
                           设置 read_status_packet_flag_
                           立即停止发送后续 command
                                      │
                                      ▼
                              下一次 update() 解析 response
```

## 5. 修改 `update()`

每次进入 `update()` 时，必须首先处理上一条 pending response：

```cpp
void DynamixelSerial::update()
{
  if (read_status_packet_flag_) {
    const int result = readExpectedResponse(instruction_last_);

    read_status_packet_flag_ = false;

    if (result != SERVO_RESULT_OK) {
      discardUnreadRxData();
    }

    completeServiceIfNeeded(instruction_last_, result);
  }

  schedulePeriodicCommands();
  processInstructionQueue();
}
```

发送需要 response 的 command 后必须立即停止处理队列：

```cpp
void DynamixelSerial::processInstructionQueue()
{
  Instruction instruction;

  while (popInstruction(instruction)) {
    sendInstruction(instruction);

    if (commandNeedsResponse(instruction.command)) {
      instruction_last_ = std::make_pair(
        instruction.command,
        instruction.servo_index
      );
      read_status_packet_flag_ = true;

      // 关键约束：等待当前 response，不能继续发送下一条 command。
      break;
    }

    completeServiceIfNeeded(instruction, SERVO_RESULT_OK);
  }
}
```

## 6. Service 使用单槽 mailbox

由于 rosserial 的 service callback 在 `rosSpinTask` 中串行执行，最小方案只需要一个固定 service mailbox，不需要为每条命令建立动态 transaction 对象。

```cpp
struct ServoServiceRequest
{
  uint8_t command;
  uint8_t servo_index;
  int32_t data[4];
  uint8_t data_length;

  volatile int8_t result;
  volatile bool pending;
};

ServoServiceRequest service_request_;
osSemaphoreId service_completion_;
```

建议增加一个特殊 instruction：

```cpp
#define INST_SERVICE_REQUEST 18
```

Service callback 只负责填写 mailbox、压入队列并等待结果，不能直接访问 UART：

```cpp
void DirectServo::servoConfigCallback(
  const spinal::SetDirectServoConfig::Request& req,
  spinal::SetDirectServoConfig::Response& res)
{
  if (!servo_handler_.enqueueServiceRequest(req)) {
    res.success = false;
    return;
  }

  const osStatus status = osSemaphoreWait(
    servo_handler_.getServiceCompletionSemaphore(),
    SERVICE_TIMEOUT_MS
  );

  res.success =
    status == osOK &&
    servo_handler_.getServiceResult() == SERVO_RESULT_OK;
}
```

如果 mailbox 已被占用或 instruction queue 已满，`enqueueServiceRequest()` 必须立即返回 `false`。

## 7. 在 `update()` 中执行 service

`INST_SERVICE_REQUEST` 由 `DynamixelSerial::update()` 处理：

```cpp
case INST_SERVICE_REQUEST:
{
  const int result = executeServiceRequest(service_request_);

  service_request_.result = result;
  service_request_.pending = false;
  osSemaphoreRelease(service_completion_);
  break;
}
```

`executeServiceRequest()` 可以继续使用现有配置逻辑，但其内部 UART 操作只能在当前 `update()` 上下文中执行。

例如 operating-mode 配置可以作为一次复合 service 操作：

```text
READ_OPERATING_MODE
        │
        ├── 已经是目标 mode ──► SUCCESS
        │
        └── mode 不同
                │
                ▼
         WRITE_OPERATING_MODE
                │
                ▼
          READ_MODE_VERIFY
                │
                ├── 匹配 ──► SUCCESS
                └── 不匹配/超时 ──► FAILURE
```

这类 service 通常发生在 torque-off 配置阶段，可以在一次 `INST_SERVICE_REQUEST` 中同步完成，不需要建立通用异步状态机。

## 8. 队列并发保护

ROS callback 和 Servo timer 都可能访问 `instruction_buffer_`，因此 push/pop 必须进行短时间 mutex 保护：

```cpp
bool DynamixelSerial::enqueueInstruction(const Instruction& instruction)
{
  if (osMutexWait(*mutex_, QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return false;
  }

  const bool result = instruction_buffer_.push(instruction);
  osMutexRelease(*mutex_);
  return result;
}
```

处理队列时，应先在 mutex 内 pop，再释放 mutex，最后执行 UART 操作：

```cpp
bool DynamixelSerial::popInstruction(Instruction& instruction)
{
  if (osMutexWait(*mutex_, QUEUE_LOCK_TIMEOUT_MS) != osOK) {
    return false;
  }

  const bool result = instruction_buffer_.pop(instruction);
  osMutexRelease(*mutex_);
  return result;
}
```

严禁以下操作：

```text
持有 queue mutex 时进行 UART 收发
持有 queue mutex 时等待 service completion
使用 osWaitForever 获取 queue mutex
```

## 9. Response 超时恢复

Response 解析失败或超时后，应丢弃 UART DMA 中尚未处理的残留数据，防止污染下一条 command：

```cpp
void DynamixelSerial::discardUnreadRxData()
{
  rd_ptr_ =
    (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx))
    % RX_BUFFER_SIZE;

  __HAL_UART_CLEAR_FLAG(huart_, UART_FLAG_RXNE);
  __HAL_UART_CLEAR_PEFLAG(huart_);
  __HAL_UART_CLEAR_OREFLAG(huart_);
  __HAL_UART_CLEAR_FEFLAG(huart_);
}
```

超时后还必须完成对应 service：

```cpp
service_request_.result = SERVO_RESULT_TIMEOUT;
service_request_.pending = false;
osSemaphoreRelease(service_completion_);
```

否则 service callback 仍可能一直等待。

## 10. 建议的结果码

```cpp
enum ServoResult : int8_t
{
  SERVO_RESULT_OK = 0,
  SERVO_RESULT_BUSY = -1,
  SERVO_RESULT_QUEUE_FULL = -2,
  SERVO_RESULT_TIMEOUT = -3,
  SERVO_RESULT_INVALID_RESPONSE = -4,
  SERVO_RESULT_INVALID_ARGUMENT = -5,
};
```

现有 ROS service response 只有 `bool success` 时，对外可以保持兼容：

```cpp
res.success = result == SERVO_RESULT_OK;
```

具体错误码可以先保存在 MCU 内部，供调试器或后续 diagnostic message 使用。

## 11. 最小修改范围

主要修改以下文件：

```text
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/
├── servo.cpp
├── servo.h
└── Dynamixel/
    ├── dynamixel_serial.cpp
    └── dynamixel_serial.h
```

不需要修改：

- Servo 控制频率。
- Fast Sync feedback 频率。
- ROS Servo 状态发布频率。
- Servo ROS message/service 定义。
- FreeRTOS 任务划分。
- USART1/USART3 波特率。

## 12. 实施顺序

1. 为队列增加线程安全的 `enqueueInstruction()` 和 `popInstruction()`。
2. 所有 service UART 操作改为 `INST_SERVICE_REQUEST`。
3. 发出需要 response 的 command 后立即 `break`。
4. `update()` 开头优先完成 `instruction_last_` 对应的 response。
5. 增加 service completion semaphore 和有限等待。
6. 增加 response 超时后的 RX 清理。
7. 确认运行期不存在 `update()` 之外的直接 UART3 访问。

## 13. 验收条件

- 任意时刻最多存在一个等待 response 的 UART command。
- Fast Sync 与 service command 不会交叉消费 response。
- Service 超时后能够返回失败，不会永久阻塞 `rosSpinTask`。
- Queue 满或 mailbox busy 时能够明确返回失败。
- Response 超时不会影响下一条 command 的解析。
- 现有 200 Hz 控制、feedback 和 ROS 发布频率保持不变。
