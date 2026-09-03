# Spinal Servo and MIT Control Upgrade Plan

## 1. Purpose and Original Development Goals

This document defines the step-by-step migration and validation plan for upgrading the Spinal servo stack in the current workspace:

```text
/home/wentao/aerial_ws/src/aerial_robot
```

The plan is based on:

- the working servo mode, MIT control, and frequency implementation in
  `/home/wentao/mpc_aerial/src/jsk_aerial_robot_dev`;
- the unresolved UART issues recorded in
  `aerial_robot_nerve/spinal/mcu_project/boards/stm32H7/SERVO_UART_MINIMAL_MODIFICATION.md`;
- Stage 1 communication references `fe95f2df` (Fast Sync Read) and `89f6ee00`
  (high-rate feedback experiment);
- Stage 2 mode/MIT references `70cd740f` and `d24a82cf`; `f5e0a2db` is retained only as
  evidence of the known hybrid-mode failure and is not a mixed-bus implementation template;
- the current root implementation, which already has a 1 kHz servo task, DMA RX, velocity feedback, and Fast Sync Read support.

The original development goals are:

1. Port the proven servo mode and low-level MIT implementation into the current root without blindly cherry-picking old code.
2. Resolve the UART concurrency, pending-response, timeout, and recovery problems before relying on higher rates.
3. Replace mixed-mode per-servo reads with at most one RS485-group read and one TTL-group read.
4. Raise the local servo write/feedback rate toward 200 Hz while keeping PC communication at a lower rate.
5. Preserve explicit low-rate configuration, torque safety, current limits, and emergency behavior.
6. Make every implementation step small enough for human code review and functional validation.
7. Validate without an oscilloscope or logic analyzer by using firmware counters, ROS timestamps, fault injection, and stress tests.

### 1.1 System scope

The first hardware validation path is the STM32H7/H7_v2 DirectServo path because that is where the reference implementation was exercised. This is an implementation order, not a reduction of the original system goal.

The complete architecture remains:

```text
VIM4 / ROS <-> Spinal <-> CAN <-> Neuron <-> Dynamixel
```

After the DirectServo communication and mode contract is validated, the same semantics must be propagated to the Spine/CAN/Neuron path when that path is used by the target Dragon hardware. This propagation is a separate review gate in Stage 3 so that DirectServo and CAN/Neuron are not changed simultaneously.

Kondo, STM32F7, and unrelated flight-control behavior are outside the feature scope, but they remain compile-regression targets.

## 2. Target Architecture and Frequency Domains

```text
PC / ROS controller
  |
  | target updates: normally 50-100 Hz
  v
servo_bridge: SI units <-> Dynamixel raw units
  |
  v
Spinal command cache
  |
  +-- DirectServo path --------------------------+
  |                                              |
  +-- Spine -> CAN -> Neuron path                 |
                                                 v
                              MCU nearest to the servo UART
                                                 |
                       1 kHz scheduler / sole UART owner
                                                 |
                       +-- goal update: up to 200 Hz
                       +-- feedback: up to 200 Hz
                       +-- ROS state publication: 50 Hz
                       +-- temperature/error: 5 Hz
                       +-- communication diagnostics: 1 Hz
```

Four frequency domains must remain independent:

1. Servo task scheduling: 1 kHz.
2. Dynamixel goal writes: target 200 Hz.
3. Current/velocity/position feedback: target 200 Hz.
4. MCU-to-PC servo state publication: default 50 Hz.

The intended final relationship is:

```text
SERVO_CONTROL_HZ = 200
SERVO_FEEDBACK_HZ = 200
SERVO_STATE_PUB_HZ = 50
SERVO_STATE_PUB_DIVIDER = 4
SERVO_DIAGNOSTIC_PUB_HZ = 1
```

The reference tree contains `SERVO_STATE_PUB_DIVIDER`, but it is currently set to 1. The current root does not contain that divider, and `sendData(true)` publishes after each completed measurement. The migration must therefore restore real decimation rather than merely copying the macro.

The PC command rate does not determine the local MIT rate. The MCU stores the latest timestamped target and runs its local loop at the configured servo-control rate. A target watchdog prevents indefinite use of stale PC commands.

## 3. Fixed Design Decisions

### 3.1 Semantic port instead of commit-level port

Reference code is used as behavioral evidence. Each change must be adapted to the current class layout, 1 kHz task, DMA buffer, existing Fast Sync parser, and current ROS interfaces. Old timer/task code must not replace the current dedicated servo task.

### 3.2 Fast grouped reads are implemented before MIT

Fast Sync Read and the frequency foundation are Stage 1 work. MIT depends on fresh position and velocity, so the communication layer must be measured and stable before adding the MIT calculation.

Frequency constants are not raised first. The dependency order is:

```text
transaction safety at 50 Hz
-> homogeneous Fast Sync baseline: send on tick N, parse on tick N+1
-> servo-ID-keyed R/T configuration
-> serialized mixed R/T Fast Sync at 50 Hz
-> remaining periodic low-rate read grouping
-> publication/control-loop decoupling
-> measured 50 -> 100 -> 200 Hz ramp
-> MIT migration
```

The existing root already contains a homogeneous Fast Sync path. Stage 1B validates and
refactors that primitive around an explicit ordered ID snapshot before mixed-bus grouping is
added. The 500 Hz reference result is implementation evidence, not a release target or default.

### 3.3 Per-servo bus type uses one-byte `R/T`

The runtime bus type is one byte:

```cpp
enum ServoBusType : uint8_t {
  SERVO_BUS_UNKNOWN = 0xFF,
  SERVO_BUS_RS485 = 'R',
  SERVO_BUS_TTL = 'T'
};
```

The Flash association must be keyed by Dynamixel ID, not by discovery index:

```cpp
struct ServoBusRecord {
  uint8_t id;
  uint8_t type;  // 'R' or 'T'
};

ServoBusRecord servo_bus_map[MAX_SERVO_NUM];  // 20 bytes for 10 servos
```

A direct table indexed by every possible Dynamixel ID would use about 253 bytes, so the 20-byte record array is the smaller stable mapping.

Flash rules:

- Append the bus records after the existing `ttl_rs485_mixed_` field.
- Do not insert fields into the existing per-servo Flash registration sequence.
- Use `0xFF, 0xFF` for unused records.
- Accept only `R` and `T`; every other value is unconfigured or invalid.
- Keep `ttl_rs485_mixed_` only for layout/tool compatibility.
- Derive homogeneous or mixed operation from the valid per-ID records.
- Do not write Flash when the requested map is unchanged.

The configuration command submits the whole map once:

```text
SET_SERVO_BUS_MAP
payload: [id0, 'R', id1, 'T', ...]
```

A valid map must contain every detected ID exactly once. IDs must also be unique across the RS485 and TTL branches; the same Dynamixel ID cannot safely exist on both branches because the transmitted instruction reaches both sides.

The actual Dynamixel ID is the only persistent configuration key. Discovery index and
`servo_serial_index` are runtime/UI ordering only. Map lookup uses `findServoById(id)`, and the
derived R and T request groups have deterministic ordering, preferably ascending ID. Later
persistent properties such as control mode and gains follow the same ID-keyed rule, although
their behavior is implemented in Stage 2.

The existing `direct_ttl_mode_` has a different meaning: it selects the MCU UART electrical mode for a direct half-duplex connection. It must not be derived from the downstream per-servo `R/T` map.

### 3.4 Conflict handling inhibits control and reports status

Configuration conflicts include:

- missing, duplicate, or unknown IDs;
- an invalid type other than `R` or `T`;
- a detected ID set that differs from the Flash map.

Runtime response conflicts include:

- an ID outside the current R/T request group;
- duplicate or out-of-order IDs in a Fast Sync aggregate response;
- a response length inconsistent with the expected group;
- repeated communication failures that prevent a complete feedback cycle.

On a definite configuration or response conflict:

1. Set `control_inhibited = true`.
2. Reject new torque-on requests.
3. Stop normal position, velocity, current, and MIT goal writes.
4. If torque is already enabled, allow one safety torque-off operation.
5. Keep configuration and diagnostic interfaces available.
6. Publish the fault immediately and then at the normal diagnostic rate.
7. Never re-enable torque automatically after recovery.

A single CRC error or timeout increments a counter and triggers RX recovery; it does not immediately become a bus conflict. The initial consecutive-failure threshold is three complete feedback cycles and must be confirmed by stress testing.

Do not reuse the existing Dynamixel hardware bits in `ServoState.error`. Add a low-rate `ServoCommStatus` message containing at least:

```text
bus_status
control_inhibited
feedback_success_count
feedback_failed_count
read_command_count
crc_error_count
timeout_count
invalid_packet_count
unexpected_id_count
rx_recovery_count
queue_overflow_count
deadline_miss_count
last_feedback_duration_us
max_feedback_duration_us
```

`ServoInfo` and `BoardInfo` report the configured `R/T` value for each ID. `ServoCommStatus` reports runtime health and test counters at 1 Hz and immediately on a state transition.

### 3.5 UART transaction ownership

Only the 1 kHz servo task may transmit to or consume responses from the Dynamixel UART. ROS subscribers and services only update a target cache or submit a request.

Every response-producing command owns a transaction record containing:

```text
instruction
bus group
ordered expected ID list
expected response length
received/success mask
absolute deadline
feedback-cycle sequence
```

The ordered expected-ID snapshot is passed unchanged to the command builder, pending
transaction, and parser. While a transaction is pending, no unrelated command is sent. Timeout
recovery clears the pending state, advances the DMA read pointer to a known boundary, clears
relevant HAL UART error flags, and permits a later cycle to retry.

The high-rate path is deliberately pipelined across servo-task ticks:

```text
tick N:   send one response-producing command and return
tick N+1: parse/finish that transaction, then send the next due command if allowed
```

It must not send a read command and synchronously wait for its response in the same 1 ms tick.
At 50 Hz this is typically send at `t0`, parse at `t1`, and send the next feedback request at
`t20`; at 200 Hz it is `t0`, `t1`, and `t5`.

R and T transactions are serialized:

```text
send R Fast Sync Read
-> on the next task tick, receive and parse the complete R aggregate response
-> send T Fast Sync Read and return
-> on the following task tick, receive and parse the complete T aggregate response
-> commit one complete feedback cycle
```

Sending both commands before parsing the first response is prohibited.

### 3.6 Control modes and MIT behavior

Keep the reference protocol values:

```text
MODE_DP  = 0  Direct Position
MODE_DV  = 1  Direct Velocity
MODE_DC  = 2  Direct Current
MODE_MIT = 3  MIT Current Control
MODE_EM  = 4  Emergency
MODE_DEP = 5  Direct Extended Position
```

Dynamixel operating-mode mapping:

```text
DP  -> Position Control Mode (3)
DEP -> Extended Position Control Mode (4)
DV  -> Velocity Control Mode (1)
DC  -> Current Control Mode (0)
MIT -> Current Control Mode (0)
EM  -> torque off; EM is not persisted as a startup mode
```

MIT uses raw MCU-side values:

```text
goal_current = current_ff
             + kp * (target_position - present_position)
             + kd * (target_velocity - present_velocity)
```

Implementation requirements:

- Use `int64_t` intermediate values.
- Preserve symmetric rounding for positive and negative current.
- Clamp against both configured current limit and the Dynamixel goal-current range.
- Output zero before the first complete position/velocity feedback.
- Update the MIT feedback snapshot only after all non-empty R/T groups succeed.
- Output zero for an incomplete MIT feedback cycle; inhibit control after the failure threshold.
- Apply a timestamp-based target watchdog to MIT and DC.
- Change mode, `kp`, and `kd` only with torque disabled.
- Read configuration back before allowing torque-on.

MIT gain encoding remains:

```text
GAIN_DECIMAL_SCALE = 10000
gain = integer + decimal / 10000
```

### 3.7 Units, messages, and topic compatibility

```text
PC/controller: rad, rad/s, Nm
servo_bridge: SI <-> raw conversion
Spinal/Neuron: raw position, velocity, and current
Dynamixel: register access and internal motor control
```

The unified target is:

```text
ServoTargetStates
  uint8[] index
  int16[] position
  int16[] velocity
  int16[] current
```

Torque enable remains a separate safety command.

DirectServo and Spine currently use `ServoControlCmd` on the same `servo/target_states` topic. DirectServo must not change that topic to `ServoTargetStates` while Spine still expects the old type, because this creates a ROS MD5/type conflict. Use a distinct DirectServo target topic during the staged migration, or switch every consumer atomically after the CAN/Neuron path is ready.

## 4. Development Stages and Review Gates

There are four major stages. Stage 1 is intentionally divided into smaller code changes because it carries most of the communication risk. Each Stage 1 substep is a separate commit and human review gate.

## Stage 1: UART, Grouped Reading, and Frequency Foundation

### Stage 1A: UART ownership and transaction recovery at 50 Hz

Scope:

- Keep the current control mode and current read strategy.
- Make the servo task the sole UART owner.
- Convert direct service UART calls into queued or mailbox requests.
- Represent urgent torque-off as a task-consumed bitmask, not as a queue entry; it preempts normal dispatch without enqueue/pop replacement.
- Use `cmdWriteTorqueEnableWithLED` only where synchronized torque/LED behavior is required and preserve the legacy torque-write helper.
- Add a pending transaction object and absolute deadline.
- Stop draining the command queue immediately after sending a response-producing instruction.
- Dispatch at most one new UART request per 1 ms task tick.
- Add deterministic timeout/CRC recovery.
- Add minimal counters for success, timeout, CRC, invalid packet, recovery, deadline miss, and queue overflow.

Human review checklist:

- No callback directly calls `cmdRead`, `readStatusPacket`, mode writes, reboot, or other UART operations.
- Exactly one transaction can be pending.
- Torque-off mask handling is checked before ordinary queued work and does not reorder the queue.
- Torque-on lights the servo LED and torque-off clears it only through the explicit combined helper.
- A timed-out service request cannot consume a stale completion from a later request.
- No `osWaitForever` remains in a callback-to-UART path.
- RX recovery cannot leave the driver permanently busy.
- Existing 50 Hz DP/DEP behavior is otherwise unchanged.

Functional gate:

- Existing homogeneous and mixed configurations still operate at 50 Hz.
- Repeated board-info/config service calls do not freeze periodic feedback.
- Torque-off is serviced ahead of queued normal work and the corresponding servo LED turns off.
- Disconnecting and reconnecting a servo causes timeout/recovery counters to change without permanently stopping the task.
- Queue overflow and unexpected transaction completion remain zero.

### Stage 1B: Reference Fast Sync baseline at 50 Hz

Scope:

- Reuse the existing/root Fast Sync support and the `fe95f2df` reference behavior for one homogeneous detected-ID set.
- Read Present Current, Present Velocity, and Present Position as one contiguous 10-byte item per servo with one command.
- Replace `send_all`-style selection with an explicit ordered `ids[]` plus `count` interface.
- Snapshot the same ordered IDs into the command builder, `PendingTransaction`, and aggregate parser.
- Send at task tick N, return immediately, and parse/complete at tick N+1; do not synchronously wait inside tick N.
- Store expected response size, deadline, and feedback-cycle sequence in the pending transaction.
- Validate outer length, exact ID order, duplicate/unknown IDs, CRC, and buffer bounds before mutating `ServoData`.
- Keep feedback, goal writes, and publication at the existing 50 Hz.

Human review checklist:

- One scheduled feedback cycle sends exactly one response-producing read command regardless of servo count.
- The parser is driven by the pending transaction's ID snapshot, not mutable discovery-array order.
- A partial or invalid aggregate packet cannot update only some public servo states.
- `read_command_count` increments per command, not per returned servo.
- `STATUS_PACKET_SIZE` covers the maximum supported aggregate response.
- The existing per-servo mixed fallback is not used as evidence that this baseline passed.

Functional gate:

- Test 1, 2, 4, and the maximum supported servo count.
- Test homogeneous all-RS485 and all-TTL electrical configurations without requiring the new per-ID map.
- Compare grouped raw current/velocity/position with the previous unicast values.
- Confirm one read command per feedback cycle and a tick-N/tick-N+1 completion sequence.
- Disconnect and reconnect one servo; recovery is bounded and no stale aggregate is accepted.

### Stage 1C: Servo-ID-keyed `R/T` storage, configuration, and control gate

Scope:

- Add one-byte runtime `ServoBusType` to `ServoData`.
- Append the 20-byte `{id, type}` record array to Flash registration without moving existing offsets.
- Add `SET_SERVO_BUS_MAP` and ID-keyed map readback.
- Treat the actual Dynamixel ID as the only persistent key; discovery index is presentation order only.
- Validate that every detected ID appears exactly once and is assigned only `R` or `T`.
- Derive deterministic R and T ID groups, preferably sorted by ID.
- Add `map_valid`, configuration conflict codes, torque-on rejection, and bus-map fields in `ServoCommStatus`.
- Keep the Stage 1B Fast Sync primitive unchanged; mixed grouping is deliberately deferred to Stage 1D.
- Require later persistent properties, including mode and gains, to follow the same ID-keyed model; their behavior remains Stage 2 work.

Human review checklist:

- Existing Flash field offsets do not move and unused records remain `0xFF, 0xFF`.
- `findServoById(id)` is used instead of assuming ID equals index or array position.
- `direct_ttl_mode_` remains independent from the downstream per-servo R/T map.
- The map is applied only while torque is off and the UART is idle.
- Invalid or unconfigured maps reject torque-on; a safety torque-off remains available.
- An unchanged map does not trigger another Flash write.

Functional gate:

- Configure all-R, all-T, and mixed maps; reboot and read them back by servo ID.
- Change discovery order and confirm each ID retains its own R/T value.
- Try missing, duplicate, unknown-ID, duplicate-across-groups, and invalid-type records.
- Confirm every invalid map inhibits normal control and publishes a distinct configuration status.

### Stage 1D: Serialized mixed R/T Fast Sync feedback at 50 Hz

Scope:

- Reuse the Stage 1B command builder and aggregate parser for each non-empty ID group.
- Execute `R send -> next-tick R parse -> T send -> following-tick T parse` under the single pending-transaction owner.
- Use one read command for homogeneous configurations and at most two for mixed configurations.
- Stage R/T results in a feedback-cycle snapshot and atomically commit only after all non-empty groups succeed.
- If the R transaction fails, abort the cycle without sending T; never publish or expose the partial R result.
- Set `setROSCommFlag(true)` only after a complete feedback-cycle commit.
- Add runtime response-conflict codes and apply the control-inhibit policy after definite/repeated failures.
- Remove periodic N-servo unicast fallback from the high-rate current/velocity/position path.

Human review checklist:

- Empty groups send no command and each command uses its transaction's ordered expected IDs.
- Response length, order, duplicates, unknown IDs, CRC, and bounds are validated before staging data.
- The second group is never transmitted while the first group is still pending.
- A failed group discards the complete cycle snapshot and cannot mix old/new group samples.
- `read_command_count` increases by the number of non-empty groups.
- Homogeneous operation still follows exactly the Stage 1B primitive.

Functional gate:

- Test all-R, all-T, 1R+1T, 1R+N-T, N-R+1T, and maximum supported mixed layouts.
- Confirm counters show one command for homogeneous and at most two for mixed feedback cycles.
- Disconnect a servo in each group and confirm no partial sample is committed or published.
- Restore communication and confirm bounded recovery without automatic torque re-enable.

### Stage 1E: Group remaining periodic low-rate reads at 50 Hz

Scope:

- Apply the explicit ordered R/T groups to periodic temperature, moving, and hardware-error reads.
- Use Fast Sync Read or grouped normal Sync Read according to register/model support.
- Schedule low-rate work only when no high-rate feedback transaction is pending or due.
- Keep bounded unicast for an explicitly addressed single-servo service or rare startup/configuration readback.
- Do not require every startup/configuration read to be grouped before the high-rate feedback milestone can pass.

Human review checklist:

- Register address, byte length, and parser layout match for every grouped instruction.
- Optional low-rate work never delays a due current/velocity/position cycle.
- Startup and service reads have finite deadlines and cannot block the UART owner.
- Periodic all-servo reads never silently fall back to N unicasts.
- Every response-producing fallback still uses the same pending-transaction ownership.

Functional gate:

- Verify temperature, moving, and hardware-error values for R, T, and mixed configurations.
- Confirm each periodic all-servo register read uses no more than one command per non-empty group.
- Exercise single-servo services during feedback and confirm bounded latency without UART overlap.
- Reboot repeatedly and confirm startup either completes or reports a bounded failure.

### Stage 1F: Feedback, control, publication, and diagnostic decoupling

Scope:

- Introduce independent `SERVO_CONTROL_HZ`, `SERVO_FEEDBACK_HZ`, `SERVO_STATE_PUB_HZ`, and `SERVO_DIAGNOSTIC_PUB_HZ` settings.
- Keep the Stage 1F baseline at 50 Hz control, 50 Hz feedback, 50 Hz state publication, and 1 Hz diagnostics.
- Implement a real cycle counter/divider and publish only atomically committed feedback cycles.
- Keep temperature/error near 5 Hz and communication diagnostics/torque state near 1 Hz.
- Ensure ROS serialization/publication never owns or waits for the Dynamixel UART.

Human review checklist:

- `sendData(true)` cannot bypass the configured state-publication divider.
- PC publication does not trigger, drive, or block local feedback/control scheduling.
- Incomplete mixed feedback never advances the publication divider.
- Rate configuration uses named constants and checks for a valid integral divider where required.
- Diagnostic publication does not add high-rate queue traffic.

Functional gate:

- At the unchanged 50 Hz baseline, `servo/states` remains approximately 50 Hz.
- Diagnostics remain approximately 1 Hz and also publish on a fault-state transition.
- Added ROS traffic and repeated read-only queries do not change UART transaction ownership.

### Stage 1G: Measured 50/100/200 Hz rate ramp

Scope:

- Measure and pass 50 Hz before testing 100 Hz, then 200 Hz, without changing the transaction logic.
- Assign named 1 ms offsets so goal writes, high-rate reads, and low-rate work do not create a scheduled burst.
- Preserve the rule that only one transaction is pending and at most one new UART request is dispatched per task tick.
- Set the state-publication divider to 1, 2, and 4 for 50, 100, and 200 Hz feedback respectively.
- Record feedback-duration P50/P99/maximum, command counts, timeouts, CRC/invalid packets, recoveries, deadline misses, and queue high-water mark.
- Treat the reference 500 Hz result as evidence only; it is not a target or release setting.

Human review checklist:

- Offset selection leaves enough ticks for the worst-case mixed R/T completion before the next due cycle.
- A rate change is made only through named settings, not scattered literals.
- PC publication remains near 50 Hz at every local feedback rate.
- Optional work is deferred rather than queued into an unbounded backlog.
- Measured command counts remain one per homogeneous cycle and at most two per mixed cycle.

Functional gate:

- Pass the defined stress procedure at 50 Hz and 100 Hz before enabling 200 Hz.
- At 200 Hz, complete mixed feedback P99 is below 4 ms; otherwise reject 200 Hz and release the highest stable lower rate.
- No permanent UART lock, queue overflow, response conflict, or sustained deadline miss occurs.
- Record the selected stable rate and evidence before Stage 2 starts.

## Stage 2: Control Modes and Low-Level MIT Port

Scope:

- Port mode, operating-mode readback, target position/velocity/current, and MIT gains into the current data model.
- Port DP, DEP, DV, DC, MIT, and EM goal generation.
- Port the `int64_t` MIT calculation, symmetric rounding, and current clamping.
- Add mode-change, target-watchdog, feedback-failure, and emergency state transitions.
- Use Bulk Write for a mixed set of per-servo goal registers.
- Keep DP or DEP as the compatible startup default; never make MIT the unreviewed default.

A critical buffer issue must be resolved here: `INSTRUCTION_PACKET_SIZE` is currently 64 bytes, while a 10-servo Bulk Write can require roughly 70 bytes for current and 90 bytes for position/velocity parameters. The implementation must either enlarge and bounds-check the packet buffer or split the Bulk Write into bounded packets.

Human review checklist:

- Logical and Dynamixel operating modes match the fixed mapping.
- Every multiplication is promoted before it can overflow.
- Positive and negative current rounding is symmetric.
- All DC/MIT paths enforce current limits.
- Mode changes require torque-off and readback.
- EM is runtime-only and produces no normal goal write.
- Incomplete feedback, stale target, conflict, and emergency states cannot continue MIT current output.
- Bulk Write cannot exceed any instruction buffer.

Functional gate:

- Configure and read back every mode with torque disabled.
- Test DP/DEP/DV/DC direction, target field, and limit behavior.
- Verify `kp=kd=0` makes MIT equal current feedforward.
- Test low MIT gains and low current limit with propellers removed.
- Verify emergency, target timeout, and communication failure stop normal control and report status.

## Stage 3: ROS, servo_bridge, and End-to-End System Integration

### Stage 3A: DirectServo host integration

Scope:

- Add or enable `ServoTargetStates`.
- Use a non-conflicting DirectServo target topic during migration.
- Update `ServoInfo`, `BoardInfo`, `SetDirectServoConfig`, `ServoCommStatus`, and rosserial headers.
- Implement mode-aware SI/raw conversion in `servo_bridge`.
- Add per-servo mode, MIT gains, current limit, and `R/T` to YAML.
- On startup, discover IDs, compare BoardInfo with YAML, apply configuration while torque is off, read it back, and only then permit torque-on.
- Keep the old DP/DEP controller behavior as the compatibility default.

### Stage 3B: Spine/CAN/Neuron propagation

Scope:

- Apply the verified mode and target semantics to the production Spine/CAN/Neuron path.
- Respect classic CAN payload limits; do not assume DirectServo service payloads fit in one CAN frame.
- Keep high-rate targets separate from low-rate mode/gain/bus configuration.
- Run the high-rate MIT loop on the MCU nearest the Dynamixel UART, normally Neuron for this path.
- Apply the same local-control versus PC-publication decimation principle.
- Avoid simultaneous old/new topic types on the same ROS topic.

Stage 3B may be marked not applicable only through an explicit human scope decision that the target hardware uses DirectServo exclusively. It must not be silently omitted, because the original Dragon architecture includes the CAN/Neuron path.

Human review checklist:

- Position, velocity, and current signs, scales, and zero offsets are consistent.
- MIT gains are converted exactly once.
- Simulation uses ROS effort control while real hardware closes MIT on the embedded MCU.
- ROS topics do not contain mixed message types.
- A service transport success is not treated as register/Flash readback success.
- Torque-on requires a valid bus map, valid mode/configuration, and no control inhibit.
- CAN payload packing has explicit size and acknowledgment behavior.

Functional gate:

- ROS messages and rosserial headers build successfully.
- Existing DP/DEP controllers still work with the compatibility default.
- YAML -> service/CAN -> Flash -> BoardInfo readback is consistent.
- PC command rate and local servo-loop rate are independent.
- Servo state remains near 50 Hz and communication diagnostics near 1 Hz.
- The actual target hardware path, DirectServo or CAN/Neuron, passes the same mode and safety tests.

## Stage 4: Functional Regression, Stress Testing, and Release

Scope:

- Finalize firmware counters and a PC-side logging/analysis script.
- Run bus-layout, mode, rate, publication-load, and fault-injection tests.
- Select 100 Hz or 200 Hz from measured results.
- Remove temporary debugging switches and obsolete compatibility paths.
- Record final parameters, limitations, and rollback instructions.

Review evidence:

- Final code diff.
- H7 and H7_v2 build results.
- STM32F7, Kondo, and CAN/Neuron compile-regression results.
- Test configuration and duration for each run.
- Start/end counter values.
- `rostopic hz`, timestamp statistics, and recovery results.
- Written justification for the released local-control frequency.

## 5. Validation Without External Measurement Equipment

### 5.1 Functional matrix

| Scenario | Required result |
|---|---|
| All R | One grouped read transaction and continuous state updates |
| All T | One grouped read transaction and continuous state updates |
| Mixed R/T | At most two grouped transactions and all IDs updated |
| Unbalanced groups | 1R+N-T and N-R+1T both work |
| Flash unconfigured | Control inhibited; ID discovery and configuration remain available |
| Duplicate/missing/invalid map | No normal control; distinct status reported |
| Duplicate physical ID across buses | Response error/conflict and control inhibit |
| One servo disconnected | Timeout increases; bounded recovery or safe inhibit; no permanent lock |
| Servo reconnected | Communication may recover; torque never re-enables automatically |
| DP/DEP/DV/DC/MIT | Correct readback, sign, target field, and limits |
| EM or target timeout | Current goes to zero or torque-off; fault state is reported |

### 5.2 Frequency ladder

| Item | 50 Hz gate | 100 Hz gate | 200 Hz gate |
|---|---:|---:|---:|
| Goal period | 20 ms | 10 ms | 5 ms |
| Feedback period | 20 ms | 10 ms | 5 ms |
| PC state publication | 50 Hz | 50 Hz | 50 Hz |
| State divider | 1 | 2 | 4 |
| Temperature/error | 5 Hz | 5 Hz | 5 Hz |
| Communication diagnostics | 1 Hz | 1 Hz | 1 Hz |

Each gate must pass before moving to the next rate. Goal write and feedback read must not be scheduled on the same 1 ms tick, although the transaction state machine remains the final serialization authority.

### 5.3 Stress procedure

With propellers removed and conservative current limits:

1. Run 50 Hz for 30 minutes to establish a baseline.
2. Run 100 Hz for 30 minutes.
3. Run 200 Hz for 10 minutes, then for 2 hours if the short test passes.
4. Test both a stationary servo and a slow repeatable trajectory.
5. Disconnect and reconnect one servo while running.
6. Run normal ROS traffic and repeated read-only configuration queries to add PC communication load.
7. Repeat the relevant run for all-R, all-T, and mixed R/T layouts.

Firmware statistics:

- complete feedback success/failure;
- read commands per feedback cycle;
- CRC, timeout, invalid length, and unexpected ID;
- RX recovery count;
- queue high-water mark and overflow;
- control/feedback deadline misses;
- last/max complete feedback duration;
- goal write count and ROS publish count.

PC-side statistics:

- mean `rostopic hz` and window variation;
- mean, standard deviation, P95, P99, and maximum `ServoStates.stamp` interval;
- per-servo maximum update gap;
- stationary raw-value variation;
- unexpected jumps or sign errors during slow motion;
- diagnostic-counter growth per minute.

### 5.4 Initial acceptance criteria

- Valid maps result in `control_inhibited=false`; every invalid map results in true.
- A mixed high-rate feedback cycle sends no more than two read commands.
- `unexpected_id_count`, definite response conflict, and queue overflow remain zero in normal operation.
- Injected failures never cause a permanent UART lock.
- At 200 Hz, mixed complete-feedback P99 is below 4 ms; otherwise 200 Hz is rejected.
- PC state publication stays near 50 Hz instead of following the 200 Hz local loop.
- CRC+timeout rate at 100/200 Hz does not materially exceed the 50 Hz baseline; the initial candidate limit is 0.01% of feedback cycles and must be confirmed from test data.
- DP/DEP behavior remains compatible with the pre-migration baseline.
- DC/MIT never exceeds the configured current limit.
- Conflict, repeated communication failure, target timeout, and EM never continue normal control.

A stable 100 Hz release is preferred over an unreliable nominal 200 Hz release.

## 6. Main Modification Boundaries

Embedded DirectServo and communication files:

```text
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/servo_base_driver.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.cpp
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/servo.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/servo.cpp
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flashmemory/flashmemory.*
aerial_robot_nerve/spinal/mcu_project/boards/stm32H7/Src/main.c
aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/Src/main.c
```

Messages and host files:

```text
aerial_robot_nerve/spinal/msg/ServoInfo.msg
aerial_robot_nerve/spinal/msg/ServoState.msg
aerial_robot_nerve/spinal/msg/ServoStates.msg
aerial_robot_nerve/spinal/msg/ServoCommStatus.msg
aerial_robot_nerve/spinal/msg/ServoTargetStates.msg
aerial_robot_nerve/spinal/srv/SetDirectServoConfig.srv
aerial_robot_model/include/aerial_robot_model/servo_bridge.h
aerial_robot_model/src/servo_bridge/servo_bridge.cpp
robots/dragon/config/quad/v1_5_202601/Servo.yaml
```

CAN/Neuron propagation files are added only in Stage 3B after the DirectServo contract is reviewed. Avoid editing CubeMX-generated regions or unrelated flight-control code unless a stage review identifies a concrete requirement.

## 7. Commit, Review, and Rollback Rules

Each Stage 1 substep and each later major stage delivers:

1. One focused Git commit.
2. A changed-file list and behavior summary.
3. Build commands and results.
4. Functional test procedure and actual result.
5. Diagnostic counter start/end values.
6. Known issues and prerequisites for the next step.
7. A recorded human review decision.

Rollback points:

- Stage 1A: original 50 Hz communication behavior.
- Stage 1B: keep the previous homogeneous 50 Hz feedback implementation.
- Stage 1C: disable the new per-ID map and return to the reviewed Stage 1B homogeneous baseline.
- Stage 1D: retain the previous per-servo mixed read path temporarily behind a review-only fallback.
- Stage 1E: retain bounded legacy low-rate reads while keeping Stage 1D high-rate feedback.
- Stage 1F: retain 50 Hz publication with divider 1.
- Stage 1G: keep 50 Hz and 100 Hz configurations selectable.
- Stage 2: DP/DEP remains the default mode.
- Stage 3: the old bridge/topic remains available until end-to-end validation is complete.

Temporary fallbacks must be removed or explicitly documented before release.

## 8. Goal-Alignment Audit

| Original goal | Plan coverage | Review point |
|---|---|---|
| Port proven reference behavior | Semantic port from the Stage 1 communication and Stage 2 mode/MIT references | `f5e0a2db` is treated as failure evidence, not copied as a mixed-mode solution |
| Resolve documented UART problems | Stage 1A owns UART, pending response, deadline, callback isolation, and recovery | Stage 1A cannot pass while callbacks still touch UART |
| Read mixed R/T servos efficiently | Stage 1B proves the primitive, Stage 1C builds ID-keyed groups, and Stage 1D serializes them | Firmware counters verify one command for homogeneous and at most two for mixed |
| Prioritize high frequency | Fast Sync is the first Stage 1 feature after UART safety | Stage 1G selects the stable rate from measured P99/error data |
| Keep PC load lower than the local loop | Stage 1F decouples publication; Stage 1G uses divider 1/2/4 | `rostopic hz` and publish counters verify approximately 50 Hz |
| Preserve low-level MIT control | Stage 2 ports the raw-value embedded MIT loop and safety limits | Low-gain/current-limited functional tests |
| Human review after each migration step | Stage 1A-1G and Stages 2-4 have code and functional gates | No next step before recorded approval |
| Validate without external instruments | Built-in counters, timestamps, disconnect tests, and stress runs | Stage 4 evidence package |
| Preserve the original Dragon chain | Stage 3B propagates the verified contract to Spine/CAN/Neuron | May be skipped only by an explicit scope decision |
| Fail safely on bus conflict | Control inhibit, torque-on rejection, one safety torque-off, status publication | Stage 1C map-conflict and Stage 1D response-conflict tests |

### Audit conclusion

The revised plan matches the initial development objective with the following explicit interpretation:

- DirectServo is the first validation platform, not the final replacement for the CAN/Neuron architecture.
- The homogeneous Fast Sync primitive is validated before R/T persistence and mixed grouping are introduced.
- Fast grouped feedback and frequency validation precede MIT migration.
- Local closed-loop and UART feedback rates are independent from PC state-publication rate.
- R/T configuration remains a compact one-byte value per servo, with a two-byte `{ID, type}` Flash record for stable association.
- Testing is based on observable software behavior and stress statistics, not unavailable measurement equipment.
- A nominal 200 Hz target is conditional on measured timing and error-rate acceptance.

Parameters still requiring human confirmation during their owning stage are the exact DirectServo topic name, `TARGET_TIMEOUT_MS`, the consecutive-failure threshold, and whether maximum-size Bulk Write uses a larger buffer or bounded packet splitting. These choices do not block Stage 1A.
