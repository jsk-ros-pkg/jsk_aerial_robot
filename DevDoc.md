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
- the current root implementation, which already has a 1 kHz servo task, one mask-based command queue, single pending-transaction ownership, 200 Hz goal/feedback scheduling, Fast Sync Read, and per-servo communication counters.

The original development goals are:

1. Port the proven servo mode and low-level MIT implementation into the current root without blindly cherry-picking old code.
2. Resolve the UART concurrency, pending-response, timeout, and recovery problems before relying on higher rates.
3. Replace mixed-mode per-servo reads with at most one RS485-group read and one TTL-group read.
4. Keep the local servo goal-write and feedback rates fixed at 200 Hz for development and full-function validation while keeping PC communication independently configurable at a lower rate.
5. Preserve explicit low-rate configuration, torque safety, current limits, and emergency behavior.
6. Make every implementation step small enough for human code review and functional validation.
7. Validate without an oscilloscope or logic analyzer by using firmware counters, ROS timestamps, fault injection, and stress tests.

### 1.1 System scope

The first hardware validation path is the STM32H7/H7_v2 DirectServo path because that is where the reference implementation was exercised. This is an implementation order, not a reduction of the original system goal.

The complete architecture remains:

```text
VIM4 / ROS <-> Spinal <-> CAN <-> Neuron <-> Dynamixel
```

Stage 1 finishes and validates the DirectServo mixed-bus transport. Stage 2 completes the DirectServo configuration, mode/MIT, and `servo_bridge` system contract. Stage 3 then separately designs and validates how that proven DirectServo behavior is integrated into the Spine/CAN/Neuron path.

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
                       +-- goal update: 200 Hz
                       +-- feedback: 200 Hz
                       +-- ROS state publication: 50 Hz
                       +-- temperature/error: 5 Hz
                       +-- communication diagnostics: 1 Hz
```

Four frequency domains must remain independent:

1. Servo task scheduling: 1 kHz.
2. Dynamixel goal writes: fixed 200 Hz development and validation rate.
3. Current/velocity/position feedback: fixed 200 Hz development and validation rate.
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

### 3.2 Finish mixed grouped transport before configuration and MIT

The current code already provides the Stage 1 communication foundation: sole task ownership of the UART, one mask-based queue, one pending response transaction, homogeneous Fast Sync feedback, queued single-servo services, 200 Hz scheduling, and per-servo counters.

Only one Stage 1 implementation step remains. The dependency order is:

```text
completed Stage 1 communication baseline at 200 Hz
-> finish ttl_rs485_mixed_-controlled R/T grouped Fast Sync Read and Sync Write
-> Stage 2: persistent ID-to-R/T assignment, mode/MIT, and servo_bridge system configuration
-> Stage 3: integrate the verified DirectServo contract into Spine/CAN/Neuron
-> Stage 4: complete regression and release evidence
```

`ttl_rs485_mixed_` is an enable flag, not a servo mask. When it is zero, an operation keeps the existing single selected-servo mask. When it is nonzero, the same selected mask is split into non-overlapping R and T masks and each non-empty group is queued on the same command queue.

Stage 1 uses temporary physical-ID lists in `dynamixel_serial.cpp` only to create servo-array-index masks and validate transport behavior. How users assign R/T to physical IDs, how that assignment is stored, and how it reaches firmware all belong to Stage 2.

The 500 Hz reference result and optional 50/100 Hz runs are diagnostic evidence only. The development and acceptance rate remains fixed at 200 Hz.

### 3.3 Persistent ID-to-R/T configuration belongs to Stage 2

The runtime bus type is one byte:

```cpp
enum ServoBusType : uint8_t {
  SERVO_BUS_UNKNOWN = 0xFF,
  SERVO_BUS_RS485 = 'R',
  SERVO_BUS_TTL = 'T'
};
```

Any persistent association must be keyed by physical Dynamixel ID, not discovery index. Each slot keeps the existing 20-byte footprint:

```text
offset  0: p_gain              uint16
offset  2: i_gain              uint16
offset  4: d_gain              uint16
offset  6: profile_velocity    uint16
offset  8: physical ID         uint8
offset  9: bus type            uint8 ('R' or 'T')
offset 10: mode                uint8 (DP/DEP/DV/DC/MIT)
offset 11: flags               uint8 (bit 0 send, bit 1 external encoder)
offset 12: joint_resolution    uint16
offset 14: servo_resolution    uint16
offset 16: joint_offset        int32
```

A direct table indexed by every possible Dynamixel ID would use about 253 bytes, so the 20-byte record array is the smaller stable mapping.

Stage 2 persistence rules:

- Stage 2 owns the fixed 20-byte ID-keyed record shown above.
- The new ID/bus/mode/flags bytes reuse the old two 16-bit flag offsets, preserving all later offsets and the total Flash region size.
- An all-`0xFF` slot is unused; legacy flag-only data is invalid because it has no valid bus.
- Accept physical IDs 0..252, bus `R`/`T`, known flag bits, and either a persistent
  DP/DEP/DV/DC/MIT mode or `Unknown` while a newly allocated record is incomplete.
- After scanning, apply only unique matching records; extra detected IDs remain visible but unconfigured.
- Read the Dynamixel EEPROM operating mode and write it only when mismatched; restore configured RAM gains/profile directly on every boot.
- Setting `bus_type` only allocates/updates the MCU Flash record. It does not access Dynamixel EEPROM/RAM
  or mark a new servo configured; setting `mode` completes the record and applies the servo configuration.
- Keep `ttl_rs485_mixed_` as the global grouping enable: `0` uses one selected-servo group and nonzero uses separate R/T masks.
- In Stage 2, derive the R/T masks from valid per-ID records instead of the temporary Stage 1 hardcoded ID lists.
- Do not rewrite MCU Flash when the requested persistent value is unchanged.

Configuration services update one physical-ID record at a time:

```text
SET_SERVO_BUS_TYPE     [physical_id, 'R' or 'T']
SET_SERVO_CONTROL_MODE [physical_id, DP/DEP/DV/DC/MIT]
SET_SERVO_MIT_PD       [physical_id, kp_integer, kp_decimal, kd_integer, kd_decimal]  # RAM only
```

A detected ID may initially have no record or an incomplete bus-only record, so it remains exposed for configuration. Torque remains inhibited until every detected ID has exactly one unique, complete record; an ID cannot belong to both R and T branches.

The actual Dynamixel ID is the only persistent configuration key. Discovery index and
`servo_serial_index` are runtime/UI ordering only. Flash lookup uses the physical ID, and the
derived R and T request groups follow discovery order. Persistent control mode, position gains,
profile velocity, flags, and encoder settings share the same record. MIT Kp/Kd are deliberately
RAM-only and are supplied together by YAML/RQT without a separate readiness flag.

The existing `direct_ttl_mode_` has a different meaning: it selects the MCU UART electrical mode for a direct half-duplex connection. It must not be derived from the downstream per-servo `R/T` map.

### 3.4 Stage 1 rejects invalid transport groups; Stage 2 owns control inhibit

Stage 1 does not add a new global inhibit state. An invalid temporary R/T split is not queued, increments an existing diagnostic counter, and leaves the reviewed timeout/CRC recovery behavior unchanged.

Stage 2 reports `bus_map_valid`; a false value directly blocks normal control. The following configuration and runtime conflict rules belong to Stage 2.

Configuration conflicts include:

- missing, duplicate, or unknown IDs;
- an invalid type other than `R` or `T`;
- a detected ID set that differs from the Flash map.

Runtime response conflicts include:

- an ID outside the current R/T request group;
- duplicate or out-of-order IDs in a Fast Sync aggregate response;
- a response length inconsistent with the expected group;

On a definite configuration conflict:

1. Keep the affected servo unconfigured so that `bus_map_valid = false`.
2. Reject new torque-on requests.
3. Stop normal position, velocity, current, and MIT goal writes.
4. If torque is already enabled, allow one safety torque-off operation.
5. Keep configuration and diagnostic interfaces available.
6. Publish the fault immediately and then at the normal diagnostic rate.
7. Never re-enable torque automatically after recovery.

A CRC error, timeout, or invalid response increments the existing Stage 1 counters and triggers RX recovery without changing the global configuration inhibit state. After the first valid position feedback, MIT current uses the latest cached position and velocity without an additional per-cycle feedback-completeness gate.

Do not reuse the existing Dynamixel hardware bits in `ServoState.error`. The current Stage 1 baseline `ServoCommStatus` message contains:

```text
transaction_success_count
timeout_count
crc_error_count
invalid_packet_count
recovery_count
deadline_miss_count
queue_overflow_count
unexpected_completion_count
queue_high_water_mark
servo_ids[]
cur_vel_pos_update_count[]
goal_write_count[]
```

`ServoCommStatus` reports cumulative communication health plus physical-ID-keyed feedback/goal counters at 1 Hz.
Stage 2 extends diagnostics with bus-map validity, conflict, and control-inhibit state and publishes fault transitions immediately.
`ServoInfo` and `BoardInfo` report the configured `R/T` value for each physical ID after persistent configuration is implemented.

### 3.5 UART transaction ownership

Only the 1 kHz servo task may transmit to or consume responses from the Dynamixel UART. ROS subscribers and services only update a target cache or submit a request.

Every response-producing command owns a transaction record containing:

```text
instruction
servo mask
expected response count/length
response format
absolute deadline
```

The servo-mask snapshot is passed unchanged to the command builder, pending transaction, and
parser. The response-format field, currently represented by `fast_sync`, selects the matching
parser without creating another queue. While a transaction is pending, no unrelated command is
sent. Timeout recovery clears the pending state, advances the DMA read pointer to a known boundary,
clears relevant HAL UART error flags, and permits a later cycle to retry.

The high-rate path is deliberately pipelined across servo-task ticks:

```text
tick N:   send one response-producing command and return
tick N+1: parse/finish that transaction, then send the next due command if allowed
```

It must not send a read command and synchronously wait for its response in the same 1 ms tick.
At the fixed 200 Hz rate this is typically send at `t0`, parse at `t1`, and send the next
feedback request at `t5`.

R and T transactions are serialized:

```text
send R Fast Sync Read
-> on the next task tick, receive and parse the complete R aggregate response
-> send T Fast Sync Read and return
-> on the following task tick, receive and parse the complete T aggregate response
-> finish the logical cycle while retaining every successfully validated group update
```

Sending both read commands before parsing the first response is prohibited.

Goal Sync Writes do not create pending responses, but R and T writes still use separate queue items and at most one new UART dispatch per servo-task iteration.

### 3.6 Stage 2 control modes and MIT behavior

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
- Output zero while `first_get_pos_flag_` is set; afterwards calculate from the latest cached position and velocity without a per-cycle group-completeness gate.
- Apply a timestamp-based target watchdog to DV, DC, and MIT.
- Change mode, `kp`, and `kd` only with torque disabled.
- Read configuration back before allowing torque-on.

MIT gain encoding remains:

```text
GAIN_DECIMAL_SCALE = 10000
gain = integer + decimal / 10000
```

### 3.7 Stage 2 host contract and Stage 3 topic compatibility

```text
PC/controller: rad, rad/s, Nm
servo_bridge: SI <-> raw conversion
Spinal/Neuron: raw position, velocity, and current
Dynamixel: register access and internal motor control
```

The unified target is:

```text
ServoControlCmd
  uint8[] index
  int16[] position
  int16[] velocity
  int16[] current
```

Torque enable remains a separate safety command.

Stage 2 owns the DirectServo message/YAML/`servo_bridge` migration. Stage 3 owns propagation into Spine/CAN/Neuron. DirectServo and Spine must not publish different message types on the same ROS topic during either stage; use a distinct migration topic or switch every consumer atomically.

## 4. Development Stages and Review Gates

There are four major stages. Stage 1 now contains one remaining focused communication change. Stage 2 completes the DirectServo system, Stage 3 separately integrates that system into Neuron, and Stage 4 collects final regression evidence.

## Stage 1: Complete `ttl_rs485_mixed_`-Controlled R/T Fast I/O

Stage 1 has one remaining implementation item. The communication foundation below is already the reviewed baseline and must not be reopened without a concrete test failure.

Current reviewed baseline:

- The 1 kHz servo task is the sole Dynamixel UART owner.
- One command queue stores `{command, servo_mask}`; bit `i` selects `servo_[i]`.
- At most one response-producing transaction is pending, with a finite deadline and bounded recovery.
- Present Current, Velocity, and Position are parsed from one contiguous Fast Sync response item per selected servo.
- Goal Sync Write and Current/Velocity/Position feedback run at the fixed 200 Hz development rate.
- Single-servo configuration services use one-bit masks and finite readback deadlines.
- `ServoCommStatus` publishes cumulative transport counters and per-servo feedback/goal counts at 1 Hz.

Remaining implementation scope:

- Keep `ttl_rs485_mixed_` as a boolean grouping enable, not as an R/T mask.
- With `ttl_rs485_mixed_ == 0`, preserve the existing path: enqueue the original selected mask once for Goal Sync Write or Current/Velocity/Position Fast Sync Read.
- With `ttl_rs485_mixed_ != 0`, use temporary hardcoded physical-ID lists in `dynamixel_serial.cpp` to derive R and T servo-array-index masks after discovery.
- Intersect both group masks with the operation's selected mask. Require no overlap and complete coverage of the selected servos; reject an invalid split without sending a partial command set.
- Enqueue one `{command, servo_mask}` item for each non-empty group on the existing queue, in deterministic R-then-T order. Do not add an R/T-specific queue or `selected_ids[]`.
- For Goal Write, transmit one masked Sync Write per non-empty group. Every selected servo must still receive approximately 200 goal writes per second.
- For feedback, transmit one Fast Sync Read per non-empty group. Complete or time out the first pending response before sending the second group.
- Keep single-servo services and existing low-rate temperature/moving/error reads unchanged in this stage.
- Apply each validated group result immediately. A later group failure does not roll back earlier servo updates; per-servo counters expose the affected group.
- Raise the ROS communication flag only after the last scheduled feedback group completes successfully; this publication rule does not undo earlier valid data updates.
- Do not add persistent ID-to-R/T records, new bus-map services, mode/MIT behavior, `servo_bridge` configuration, or Neuron changes in Stage 1.

Human review checklist:

- The mixed branch is entered only when `ttl_rs485_mixed_ != 0`; the flag itself is never decoded as a servo mask.
- Hardcoded values are physical Dynamixel IDs and are converted to mask bits through the discovered `servo_[i].id_` association.
- Homogeneous operation produces one group and mixed operation produces exactly two non-empty groups.
- The existing queue, pending transaction, parser, timeout recovery, and one-dispatch-per-task-iteration rules are reused.
- No periodic per-servo fallback remains in the high-rate goal or Current/Velocity/Position path.
- The change does not alter Flash layout, service payloads, control modes, or host interfaces.

Functional gate:

- With `ttl_rs485_mixed_ == 0`, verify the reviewed one-group behavior at 200 Hz.
- With `ttl_rs485_mixed_ != 0`, test all-R, all-T, 1R+N-T, N-R+1T, alternating R/T, and the maximum supported servo count.
- Confirm one Fast Sync Read per homogeneous feedback cycle and two per mixed cycle; apply the equivalent one/two-group rule to Goal Sync Write.
- Confirm every healthy active servo reports approximately 200 Current/Velocity/Position updates and 200 goal writes per one-second diagnostic window.
- Test overlap and missing coverage, including replacing an active ID with an unknown hardcoded ID, and confirm the invalid split is rejected without UART overlap or queue corruption.
- Disconnect and reconnect a servo in each group; healthy-group updates continue, affected counters lag, and recovery remains bounded.
- Repeatedly switch `ttl_rs485_mixed_` through the existing configuration path and reboot to confirm the global enable is honored.
- Queue overflow and unexpected completion remain zero in normal operation, and mixed feedback P99 remains below 4 ms at 200 Hz.

Stage 1 is complete when this gate passes. The hardcoded ID lists remain temporary transport-test fixtures and are replaced by the Stage 2 configuration source; they are not a user-facing configuration mechanism.

## Stage 2: DirectServo Configuration, Modes/MIT, and Host System Setup

Stage 2 is implemented as one cohesive development change. The reference path already provides nearly complete examples, so R/T configuration, mode/MIT behavior, messages, YAML, and `servo_bridge` are migrated together and pass one unified review gate. Stage 2 does not change Spine/CAN/Neuron.

Scope:

- Replace the Stage 1 hardcoded R/T lists with compact persistent records keyed by physical Dynamixel ID.
- Keep `ttl_rs485_mixed_` as the global grouping enable: the persistent map supplies group membership and the flag selects one-group or R/T-group scheduling.
- Store and read back R/T by physical Dynamixel ID, never by discovery index or `servo_serial_index`.
- Store ID, R/T bus, DP/DEP/DV/DC/MIT mode, packed flags, gains/profile, and encoder settings in each fixed 20-byte MCU Flash record.
- Expose missing IDs as unconfigured, reject duplicate/invalid records, and inhibit torque-on until every detected ID has one valid assignment.
- Check Dynamixel EEPROM before writing changed operating mode; write configured RAM gains/profile directly after every reboot.
- Port target position/velocity/current, RAM-only MIT gains, and DP/DEP/DV/DC/MIT goal generation; retain EM only as a runtime safe state.
- Use `int64_t` MIT intermediates, symmetric rounding, configured current limits, first-position-feedback protection, the latest cached position/velocity, a target watchdog, and emergency handling.
- Change persistent mode/configuration only while torque is off and require register/readback consistency before torque-on.
- Use a bounded Bulk Write when a cycle contains different per-servo goal registers.
- Upgrade `ServoControlCmd` to position/velocity/current and regenerate rosserial headers; do not add a parallel target message.
- Let Flash determine per-servo mode by default; YAML may explicitly update mode and supplies MIT Kp/Kd together when used.
- On startup, discover IDs, apply matching Flash configuration, expose unmatched IDs, then apply optional host YAML overrides.
- Implement mode-aware SI/raw conversion in `servo_bridge`; gains, signs, and offsets are converted exactly once.
- Publish all three target fields through `ServoControlCmd` on the existing DirectServo target topic.
- Keep the local goal/feedback loop at 200 Hz, decouple `servo/states` to approximately 50 Hz, and keep communication diagnostics near 1 Hz.

`INSTRUCTION_PACKET_SIZE` must cover the maximum selected Bulk Write or the operation must be split into bounded packets. No packet builder may write beyond its buffer.

Human review checklist:

- The implementation follows the reference behavior but is adapted to the current queue, scheduler, parser, data model, and ROS interfaces.
- R/T, mode, gains, and current limits remain associated with physical IDs across discovery-order changes and reboot.
- Valid configuration is required for torque-on; missing initial position feedback, stale-target, and emergency failures retain their defined handling.
- DP/DEP compatibility is preserved and MIT is never enabled as an unreviewed default.
- Firmware and host conversions do not duplicate sign, scale, offset, or gain conversion.
- Flash writes, Dynamixel register writes, and service results all have explicit readback semantics.
- Bulk Write and every message/service payload remain within their buffer and transport limits.

Functional gate:

- Firmware, ROS messages, rosserial headers, YAML, and `servo_bridge` build together as one change.
- Configure all-R, all-T, and mixed maps; reboot, change discovery order, and verify physical-ID-keyed readback.
- Verify invalid maps or modes inhibit torque-on while diagnostics, reconfiguration, and safety torque-off remain available.
- Configure and read back every mode, then test DP/DEP/DV/DC direction, target field, and limits.
- Verify `kp=kd=0` makes MIT equal current feedforward, then test low MIT gains and conservative current limits with propellers removed.
- Verify first-position-feedback protection, stale targets, EM, and current limits produce the defined output.
- Confirm YAML -> service -> Flash/register -> `ServoInfo`/`BoardInfo` readback is consistent by physical ID.
- Confirm persistent R/T masks reuse the Stage 1 grouped queue/parser without changing its one/two-group 200 Hz behavior.
- Confirm host command rate and approximately 50 Hz ROS publication do not drive or block the 200 Hz local UART/control schedule.

Stage 2 is complete only when the unified gate passes; R/T configuration, mode/MIT, and host integration are not separate release steps.

## Stage 3: Integrate the Verified DirectServo System into Spine/CAN/Neuron

Stage 3 is a separate architecture and implementation stage. The completed DirectServo path is the behavioral reference; its code is not blindly copied into Neuron.

Scope:

- Define and implement one clear responsibility split between Spinal and Neuron for target caching, R/T configuration, mode/gain storage, the 200 Hz control loop, Dynamixel scheduling, status aggregation, and safety ownership.
- Reuse a shared servo transport/control component or a Neuron adapter around the verified contract according to the current architecture; review the resulting boundary in the single Stage 3 gate.
- Run the high-rate feedback and MIT loop on the MCU that owns the Dynamixel UART, normally Neuron in the production chain.
- Define CAN messages for high-rate targets separately from low-rate R/T, mode, gain, current-limit, and diagnostic data.
- Upgrade the production Spinal/Neuron link to CAN FD with bit-rate switching, use bounded CAN-FD records with explicit DLC padding, sequence/acknowledgment rules, timeout behavior, and register/Flash readback, and retain Classic CAN only where compatibility is explicitly required.
- Preserve physical-ID-keyed configuration and the same one/two-group R/T behavior where the Neuron hardware topology requires it.
- Keep DirectServo operational as the validated reference and rollback path until the Neuron chain passes equivalent tests.
- Avoid publishing incompatible old/new message types on the same ROS topic.

### Stage 3 CubeMX and `.ioc` configuration

The CubeMX `.ioc` files are the source of truth for FDCAN peripheral configuration. Do not
hand-edit the generated `MX_FDCAN1_Init()` body as the implementation method: code generation
will overwrite changes outside `USER CODE` regions. Stage 3 changes hardware configuration in
CubeMX, commits the corresponding `.ioc` change, regenerates code, and reviews the generated diff.
Protocol, packing, scheduling, and safety logic remain normal source changes outside generated
CubeMX regions.

The two relevant CubeMX projects are:

```text
aerial_robot_nerve/neuron/neuron_g4/neuron_g4.ioc
aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/spinal.ioc
```

#### Neuron G4 configuration

Open `neuron_g4.ioc` with the matching STM32CubeMX/STM32CubeIDE version, select FDCAN1, and set:

```text
Parameter                         Value
Frame Format                     FD mode with Bit Rate Switching
Mode                             Normal
Auto Retransmission              Enable
Clock Divider                    1
Nominal Prescaler                1
Nominal Time Segment 1           63
Nominal Time Segment 2           16
Nominal Synchronization Jump     16
Data Prescaler                   1
Data Time Segment 1              7
Data Time Segment 2              2
Data Synchronization Jump        2
Standard Filters                 2, unless the reviewed ID layout changes
```

Keep the FDCAN kernel clock sourced from the PLL at 80 MHz. These settings give matching 80%
sample points and the intended rates:

```text
nominal = 80 MHz / (1 * (1 + 63 + 16)) = 1 Mbit/s
data    = 80 MHz / (1 * (1 + 7 + 2))   = 8 Mbit/s
```

After generation, verify that the generated Neuron initialization contains:

```cpp
hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
```

The current Neuron `.ioc` stores the nominal/data timing but does not store `FrameFormat`, so it
currently regenerates `FDCAN_FRAME_CLASSIC`. Adding the frame-format setting to the `.ioc` is a
required Stage 3 change.

#### Spinal H7_v2 configuration

The H7_v2 `.ioc` already selects `FDCAN_FRAME_FD_BRS`, the same 1 Mbit/s nominal and 8 Mbit/s
data timing, 64-byte RX/TX elements, and a ten-element TX FIFO. Preserve those values unless a
measured physical-layer test requires a lower data rate.

The current H7_v2 RX FIFO has only one element. That is vulnerable to a four-Neuron reply burst
even when average bus utilization is low. Stage 3 must size the hardware RX FIFO and the RTOS CAN
mail queue from the maximum scheduled burst. An initial RX FIFO candidate is eight 64-byte
elements, subject to message-RAM validation; acceptance is based on zero FIFO overrun and zero
software queue overflow during the Stage 3 stress test rather than on the candidate number alone.

#### CubeMX regeneration rules

1. Start from a clean or intentionally recorded worktree and record the CubeMX version used.
2. Enable "Keep User Code when re-generating" before code generation.
3. Make FDCAN clock, frame-format, timing, FIFO, and element-size changes in the `.ioc`, not by
   editing their generated assignments in `main.c`.
4. Generate once, inspect the complete Git diff, and reject unrelated clock, pin, middleware,
   linker, task, or interrupt changes.
5. Generate a second time without changing the `.ioc`; the second generation must produce no
   semantic diff.
6. Rebuild Spinal and Neuron, then first flash one Spinal/Neuron pair before enabling CAN FD on
   the complete robot bus.

Only code enclosed by the intended CubeMX `USER CODE` markers is expected to survive regeneration.
If Stage 3 discovers existing custom code outside those markers, move it to a maintained source
file or a valid user region as a separately reviewed change before relying on regeneration.

### Stage 3 CAN-FD protocol and bandwidth guardrails

CubeMX does not configure the application protocol. Stage 3 must separately update the common CAN
code and the Spinal/Neuron servo, IMU, motor, and scheduler code.

In particular, the current FDCAN send helper selects Classic CAN whenever `dlc <= 8` and selects
CAN FD only for larger payloads. Replace this implicit size rule with an explicit frame-mode
argument or equivalent reviewed policy. A high-rate payload of eight bytes or less may still need
FD+BRS; otherwise pad a bounded record to a supported 12-byte FD DLC. Accept only the CAN-FD DLC
sizes `0..8, 12, 16, 20, 24, 32, 48, 64`, clear every padding byte, and validate received DLC before
casting or decoding a payload.

The initial bandwidth calculation uses the current raw field widths:

```text
feedback per servo: position int16 + velocity int32 + current int16 + error uint8 = 9 bytes
P/C target per servo: position int16 + current int16 = 4 bytes
```

With four Neurons, six joints, eight gimbals, 11-bit identifiers, 1 Mbit/s arbitration, 8 Mbit/s
data, and conservative bit stuffing, the estimated servo-only utilization at 200 Hz is:

| CAN traffic | Application payload | Estimated bus utilization |
|---|---:|---:|
| Six-joint P/C targets and six-joint P/V/C/Error feedback | 15.6 kB/s | about 7.0% |
| All 14 servo P/C targets and six-joint feedback | 22.0 kB/s | about 8.4% |
| All 14 servo P/C targets and all 14 servo feedback | 36.4 kB/s | about 11.7% |
| All 14 servo P/V/C targets and all 14 servo feedback | 47.6 kB/s | about 13.1% |

CAN bandwidth is therefore not the primary obstacle, but retaining the existing Classic small-frame
layout wastes the available margin. If eight independent rotor commands remain as two Classic
eight-byte frames at 500 Hz, they consume up to about 13.5%. If all four Neuron IMUs retain three
Classic six-byte frames at 200 Hz, they consume up to about 27.6%. Together with the conservative
full-servo case, this is about 54.2% before retransmission and other traffic. Packing all eight rotor
commands into one 12-byte FD frame and each Neuron gyro/acc/mag set into one 20-byte FD frame reduces
the corresponding total estimate to about 20.8%.

Stage 3 therefore uses per-board packed target/state records, packs related IMU and rotor fields
where their update and safety semantics match, and gives control/emergency frames suitable CAN-ID
priority. Every high-rate record includes at least a protocol version or record type, a sequence
counter, bounded fields, and explicit stale/timeout behavior. The receiver must not treat successful
CAN delivery as confirmation that a Dynamixel register or Neuron Flash write succeeded.

The current Spinal scheduler provides 500 aggregate servo-board slots per second and round-robins
them across Neurons, which is only 125 Hz per board for four Neurons. Stage 3 must redesign that
scheduler rather than merely changing a frequency constant. Likewise, the current Neuron feedback
record has no velocity, position is sampled at 50 Hz, and current/error are sampled at 5 Hz. A
200 Hz CAN publication is not accepted unless the Neuron-owned Dynamixel transaction actually
produces fresh P/V/C data at that rate; repeated cached samples do not satisfy the gate.

All active controllers on a bus must be CAN-FD tolerant before transmitting the first FD frame.
An STM32F1/Classic-only Neuron cannot remain active on the same bus during the FD migration. Verify
the exact CAN transceiver part, termination, topology, and harness length at the selected 8 Mbit/s
data phase; FDCAN-capable MCUs alone do not prove that the physical layer supports that rate. If
8 Mbit/s fails the error-rate and recovery gate, change the data timing consistently in both `.ioc`
files and record the measured reason; do not create unmatched Spinal and Neuron timings.

Human review checklist:

- There is one clear owner for every target, configuration field, UART transaction, and safety transition.
- Position, velocity, current, gains, signs, and offsets are converted exactly once across ROS, Spinal, CAN, Neuron, and Dynamixel.
- CAN transport success is not treated as Neuron register or Flash readback success.
- No DirectServo-specific assumption leaks into Neuron without an explicit adapter or shared interface.
- Failure of CAN, Neuron, one R/T group, or stale host targets leads to a defined safe state.

Functional gate:

- Regenerating each CubeMX project from its committed `.ioc` produces the reviewed FD+BRS, timing,
  FIFO, and element-size configuration without hand edits to generated initialization code.
- A second generation without `.ioc` changes produces no semantic diff, and all preserved custom
  code remains inside maintained files or valid `USER CODE` regions.
- Every active node and transceiver passes single-pair bring-up before the complete bus is switched
  to CAN FD; no Classic-only node remains active when FD frames are transmitted.
- At 200 Hz and worst scheduled CAN load, FDCAN protocol errors, RX FIFO overruns, CAN mail-queue
  overflow, stale sequences, and missed control deadlines remain within their defined acceptance
  limits; normal utilization retains explicit retransmission and emergency-command margin.
- The full VIM4/ROS <-> Spinal <-> CAN <-> Neuron <-> Dynamixel chain builds and discovers/configures servos by physical ID.
- DP/DEP compatibility and the new DV/DC/MIT/EM semantics match the verified DirectServo behavior.
- The Neuron-owned local goal/feedback loop passes at 200 Hz while PC state publication remains approximately 50 Hz.
- All-R, all-T, mixed R/T, restart/readback, disconnect/reconnect, stale-target, emergency, and communication-load tests pass.
- DirectServo and Neuron results are compared with the same counters and acceptance criteria before DirectServo ceases to be the primary test path.

## Stage 4: Functional Regression, Stress Testing, and Release

Scope:

- Finalize firmware counters and a PC-side logging/analysis script.
- Run bus-layout, mode, rate, publication-load, and fault-injection tests.
- Confirm the fixed 200 Hz local goal/feedback release setting from measured results.
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

| Owner | Scenario | Required result |
|---|---|---|
| Stage 1 | `ttl_rs485_mixed_ == 0` | One selected-mask read/write group and continuous 200 Hz updates |
| Stage 1 | Mixed flag, all R or all T | One non-empty group; empty group sends nothing |
| Stage 1 | Mixed flag, 1R+N-T or N-R+1T | Exactly two serialized groups; every selected servo updates |
| Stage 1 | Mixed flag, invalid temporary ID split | No partial command set; diagnostic counter increases |
| Stage 1 | One servo disconnected | Bounded timeout/recovery; healthy updates continue; no permanent lock |
| Stage 2 | Flash/YAML unconfigured | Control inhibited; discovery and configuration remain available |
| Stage 2 | Duplicate/missing/invalid persistent map | No torque-on; distinct configuration status |
| Stage 2 | DP/DEP/DV/DC/MIT and EM | Correct readback, sign, target field, watchdog, and limits |
| Stage 3 | Full Spine/CAN/Neuron chain | Matches DirectServo behavior and safety semantics |

### 5.2 Fixed 200 Hz validation setting

| Item | Development and validation setting |
|---|---:|
| Goal write | 200 Hz / 5 ms |
| Current/velocity/position feedback | 200 Hz / 5 ms |
| Stage 1 transport-test publication | May follow completed feedback at 200 Hz |
| Stage 2+ PC state publication | 50 Hz / divider 4 |
| Temperature/error | 5 Hz |
| Communication diagnostics | 1 Hz |

Goal writes and feedback reads must not be scheduled on the same 1 ms tick, although the transaction state machine remains the final serialization authority. A 50 or 100 Hz run is optional diagnostic comparison only and never replaces the 200 Hz acceptance run.

### 5.3 Stress procedure

With propellers removed and conservative current limits:

1. In Stage 1, run each hardcoded all-R, all-T, and mixed R/T layout at 200 Hz for a short functional check.
2. Run the Stage 1 mixed path for 30 minutes with a stationary servo and a slow repeatable trajectory.
3. Continue the Stage 1 200 Hz run for 2 hours after the short and 30-minute checks pass.
4. Disconnect and reconnect one servo in each Stage 1 group while 200 Hz operation continues.
5. In Stage 2, repeat the R/T matrix from persistent configuration and exercise every mode with conservative limits.
6. In Stage 3, repeat the relevant matrix through the full Neuron chain under normal ROS, service, and CAN load.
7. Use 50 or 100 Hz only when a comparison helps diagnose a failure, then return to 200 Hz for acceptance.

Firmware statistics:

- grouped feedback command success/failure;
- successful response transactions per feedback cycle;
- CRC, timeout, invalid length, and unexpected ID;
- RX recovery count;
- queue high-water mark and overflow;
- control/feedback deadline misses;
- last/max logical feedback-cycle duration;
- physical servo IDs with per-servo current/velocity/position update counts;
- physical servo IDs with per-servo goal-write counts;
- ROS publish count.

PC-side statistics:

- mean `rostopic hz` and window variation;
- mean, standard deviation, P95, P99, and maximum `ServoStates.stamp` interval;
- per-servo maximum update gap;
- stationary raw-value variation;
- unexpected jumps or sign errors during slow motion;
- diagnostic-counter growth per minute.

### 5.4 Initial acceptance criteria

- Stage 2 valid maps result in `bus_map_valid=true`; every invalid map results in false.
- A Stage 1 mixed high-rate feedback cycle sends no more than two Fast Sync Read commands, and its goal cycle sends no more than two Sync Write commands.
- Every healthy active servo reports approximately 200 feedback updates and 200 goal writes per one-second diagnostic window.
- Stage 1 `invalid_packet_count`, `unexpected_completion_count`, and `queue_overflow_count` remain zero in normal operation.
- Injected failures never cause a permanent UART lock in DirectServo or Neuron.
- At 200 Hz, Stage 1 mixed complete-feedback P99 is below 4 ms; otherwise Stage 1 remains open until the cause is corrected.
- From Stage 2 onward, PC state publication stays near 50 Hz instead of following the 200 Hz local loop.
- At 200 Hz, the initial candidate CRC+timeout limit is 0.01% of feedback cycles and must be confirmed from test data.
- Stage 2/3 DP/DEP behavior remains compatible with the pre-migration baseline.
- Stage 2/3 DC/MIT never exceeds the configured current limit.
- Configuration conflict, missing initial MIT position feedback, target timeout, and EM never continue normal control.

The 200 Hz rate is a required operating condition, not an optional target or a lower-rate release fallback.

## 6. Main Modification Boundaries

Stage 1 write scope:

```text
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.cpp
```

Stage 1 should not modify Flash layout, ROS messages/services, YAML, `servo_bridge`, control-mode semantics, or Neuron code. A file outside this list requires a concrete compile fix or reviewed transport dependency.

Stage 2 DirectServo system scope:

```text
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/servo_base_driver.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/drivers/Dynamixel/dynamixel_serial.cpp
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/servo.h
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/servo.cpp
aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/flashmemory/flashmemory.*
aerial_robot_nerve/spinal/msg/ServoInfo.msg
aerial_robot_nerve/spinal/msg/ServoState.msg
aerial_robot_nerve/spinal/msg/ServoStates.msg
aerial_robot_nerve/spinal/msg/ServoCommStatus.msg
aerial_robot_nerve/spinal/msg/ServoControlCmd.msg
aerial_robot_nerve/spinal/srv/SetDirectServoConfig.srv
aerial_robot_model/include/aerial_robot_model/servo_bridge.h
aerial_robot_model/src/servo_bridge/servo_bridge.cpp
aerial_robot_nerve/spinal/launch/Servo.yaml
aerial_robot_nerve/spinal/launch/DirectServo.urdf
aerial_robot_nerve/spinal/launch/bridge.launch
```

Stage 3 Neuron integration scope:

Stage 3 may modify the required Spinal/CAN/Neuron files as one cohesive integration change. FDCAN
peripheral changes belong in the two CubeMX `.ioc` files. The corresponding generated regions may
change only as output from the committed `.ioc`; do not hand-edit them. Protocol implementation,
packing, scheduling, state acquisition, diagnostics, and safety behavior belong in maintained
Spinal/Neuron library source. Avoid unrelated flight-control code unless the unified Stage 3
implementation has a concrete requirement.

Minimum CubeMX/configuration scope:

```text
aerial_robot_nerve/neuron/neuron_g4/neuron_g4.ioc
aerial_robot_nerve/neuron/neuron_g4/Core/Src/main.c                  # generated from .ioc
aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/spinal.ioc
aerial_robot_nerve/spinal/mcu_project/boards/stm32H7_v2/Src/main.c   # generated from .ioc
```

## 7. Commit, Review, and Rollback Rules

Each major Stage is implemented as one cohesive change and delivers:

1. One focused Git commit.
2. A changed-file list and behavior summary.
3. Build commands and results.
4. Functional test procedure and actual result.
5. Diagnostic counter start/end values.
6. Known issues and prerequisites for the next step.
7. A recorded human review decision.

Rollback points:

- Stage 1: set `ttl_rs485_mixed_` to zero and retain the reviewed one-group 200 Hz queue/parser baseline while diagnosing the mixed grouping change.
- Stage 2: restore the previous DirectServo revision as one unit if the unified Flash/message/mode migration fails; no runtime default mode fallback is provided.
- Stage 3: keep DirectServo as the active hardware path and do not enable the Neuron path until end-to-end validation passes.

Temporary fallbacks must be removed or explicitly documented before release.

## 8. Goal-Alignment Audit

| Original goal | Plan coverage | Review point |
|---|---|---|
| Port proven reference behavior | Semantic port from communication and mode/MIT references | `f5e0a2db` remains failure evidence, not a mixed-bus template |
| Resolve documented UART problems | The reviewed Stage 1 baseline owns UART, queue, pending response, deadline, services, and recovery | The final Stage 1 change must reuse this baseline |
| Read/write mixed R/T servos efficiently | Stage 1 uses `ttl_rs485_mixed_` to select one group or temporary hardcoded R/T groups | Counters verify one homogeneous group and at most two mixed groups at 200 Hz |
| Configure R/T by physical ID | Stage 2 replaces test fixtures with persistent ID-keyed records | YAML/service/Flash/readback remains stable across discovery order |
| Preserve and extend control modes | Stage 2 ports DP/DEP/DV/DC/MIT/EM with limits and watchdogs | Low-gain/current-limited functional tests |
| Complete DirectServo host setup | Stage 2 owns messages, YAML, `servo_bridge`, startup application, and 50 Hz publication | End-to-end DirectServo configuration/readback and rate tests |
| Integrate the production Dragon path | Stage 3 separately designs and implements Spine/CAN/Neuron ownership | DirectServo and Neuron pass equivalent behavior and safety tests |
| Human review after each migration step | Stages 1-4 each have one cohesive implementation and one unified gate | No next stage before recorded approval |
| Validate without external instruments | Built-in counters, timestamps, disconnect tests, and stress runs | Stage 4 evidence package |
| Fail safely | Stage 1 handles transport faults; Stage 2 adds configuration/mode safety; Stage 3 propagates it through CAN/Neuron | No fault path continues unsafe control |

### Audit conclusion

The revised plan matches the initial development objective with the following explicit interpretation:

- Stage 1 has one remaining task: finish and validate `ttl_rs485_mixed_`-controlled grouped Fast Sync Read and masked Sync Write at 200 Hz.
- Stage 1 hardcoded physical-ID lists are temporary transport fixtures, not system configuration.
- Stage 2 owns persistent ID-to-R/T assignment together with mode/MIT, messages, YAML, and `servo_bridge` work.
- Stage 3 independently decides and implements how the verified DirectServo system is integrated into Spine/CAN/Neuron.
- Local closed-loop and UART feedback rates remain independent from PC state-publication rate.
- Testing is based on observable software behavior and stress statistics, not unavailable measurement equipment.
- The 200 Hz local goal/feedback rate is fixed; acceptance determines whether a stage passes, not which lower rate is released.

Parameters still requiring human confirmation during their owning stage are the exact DirectServo topic name, `TARGET_TIMEOUT_MS`, the persistent R/T record layout, and whether maximum-size Bulk Write uses a larger buffer or bounded packet splitting. These Stage 2 choices do not block the remaining Stage 1 grouped-transport work.
