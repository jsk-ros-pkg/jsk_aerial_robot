# Dragon Hardware Control Development Doc

This document records the planned reconstruction items for the current Dragon
hardware structure:

```text
VIM4 / ROS <=> Spinal <=> CAN <=> Neuron <=> Rotor / Dynamixel servo
```

The goal is to keep the high-rate command path simple and unified, while keeping
low-rate servo configuration explicit and safe, and also speed close-loop frequency

## 1. Add Velocity Command and Feedback

### Required Work

- Add Goal Velocity support in Neuron Dynamixel driver.
- Add velocity field to the Spinal/Neuron servo target path.
- Add velocity feedback to `ServoState` and bridge feedback conversion.
- Add `fastSyncRead` where useful to reduce Dynamixel UART read latency.

### Modification Boundary

- Neuron Dynamixel driver:
  `aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/dynamixel_serial.*`.
- Neuron servo state cache:
  `aerial_robot_nerve/neuron/neuronlib/Servo`.
- Spinal servo state message and publisher:
  `aerial_robot_nerve/spinal/msg/ServoState.msg`,
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/Spine/spine.cpp`.
- ROS bridge conversion:
  `aerial_robot_model/include/aerial_robot_model/servo_bridge.h`,
  `aerial_robot_model/src/servo_bridge/servo_bridge.cpp`.

### Impact

- MIT-style control needs reliable velocity feedback.
- More frequent or larger Dynamixel sync reads may increase UART bus load.
- TTL/RS485 mixed mode should be checked after adding new read fields.

## 2. Separate Control Mode and Servo Configuration

### Target Design

High-rate target values should not carry rarely changed configuration. Mode and
gains should be configured through a low-rate channel:

```yaml
# ServoInfo.msg
# old
uint8 id
uint16 p_gain
uint16 i_gain
uint16 d_gain
uint16 profile_velocity
uint16 current_limit
uint8 send_data_flag
uint8 external_encoder_flag
int16 joint_resolution
int16 servo_resolution

# new
uint8 id
uint8 mode
int16 kp_integer
int16 kp_decimal
int16 kd_integer
int16 kd_decimal
uint16 p_gain
uint16 i_gain
uint16 d_gain
uint16 profile_velocity
uint16 current_limit
uint8 send_data_flag
uint8 external_encoder_flag
int16 joint_resolution
int16 servo_resolution
```

Possible modes:

`DP` direct positon mode    : use target position, servo internal position loop
`DV` direct velocity mode   : use target velocity, servo internal velocity loop
`DC` direct effort mode     : use target current, servo internal current loop
`MIT` effort based MIT mode : tracking position and velocity with feedforward current，by target current, servo internal current loop
$$
\tau = k_p \cdot (q_{des} - q) + k_d \cdot (\dot{q}_{des} - \dot{q}) + \tau_{ff}
$$
`EM` emgerency mode: disable torque, just for emergcncy stop (Don't flash)

`kp` and `kd` should be set separately through low-rate config commands. Each
gain is encoded by two `int16` values: one for the integer part before the
decimal point and one for the fractional part after the decimal point.

```yaml
GAIN_DECIMAL_SCALE: 10000

kp = kp_integer + kp_decimal / GAIN_DECIMAL_SCALE
kd = kd_integer + kd_decimal / GAIN_DECIMAL_SCALE

SET_SERVO_MIT_KP:
  data: [slave_id, servo_index, kp_integer, kp_decimal]
SET_SERVO_MIT_KD:
  data: [slave_id, servo_index, kd_integer, kd_decimal]
```

With this format, one gain setting uses four payload bytes after `command` and
`servo_index`, so it fits in one CAN config frame. `kp_decimal` and `kd_decimal`
should be non-negative and smaller than `GAIN_DECIMAL_SCALE`. If negative gains
are ever required, define the sign rule explicitly before implementation.

### Modification Boundary

- ROS message and service definition:
  `aerial_robot_nerve/spinal/msg/ServoInfo.msg`,
  `aerial_robot_nerve/spinal/srv/SetBoardConfig.srv`.
- ROS-side config and unit source:
  `robots/dragon/config/quad/v1_5_202601/Servo.yaml`.
- ROS-side bridge/readback handling:
  `aerial_robot_model/include/aerial_robot_model/servo_bridge.h`,
  `aerial_robot_model/src/servo_bridge/servo_bridge.cpp`.
- Spinal rosserial service:
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/Spine/spine.cpp`.
  It advertises `get_board_info` and `set_board_config`; currently
  `set_board_config` returns success after sending the CAN request.
- Spinal CAN config packing and cached board info:
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/initializer/can_initializer.cpp`,
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/initializer/can_initializer.h`,
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/servo/can_servo.h`.
- Spinal CAN command constants:
  `aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/CAN/can_constants.h`.
- Neuron CAN command constants:
  `aerial_robot_nerve/neuron/neuronlib/CAN/can_constants.h`.
- Neuron config receiver and persistent storage:
  `aerial_robot_nerve/neuron/neuronlib/Initializer/initializer.cpp`,
  `aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/dynamixel_serial.h`,
  `aerial_robot_nerve/neuron/neuronlib/Servo/Dynamixel/dynamixel_serial.cpp`.
- ROS GUI tools, if these fields need manual tuning:
  `aerial_robot_nerve/spinal/src/spinal/board_configurator.py`,
  `aerial_robot_nerve/spinal/src/spinal/servo_monitor.py`.
- Optional compatibility path for direct servos on Spinal:
  `aerial_robot_nerve/spinal/srv/SetDirectServoConfig.srv`,
  `aerial_robot_nerve/spinal/mcu_project/lib/Jsk_Lib/servo/servo.cpp`.

### Impact

- Mode changes should be treated as low-frequency operations, preferably with
  torque disabled or under an explicit safety rule.
- Existing fields `p_gain/i_gain/d_gain` are Dynamixel internal gains. Do not
  reuse them silently as MIT `kp/kd`.
- Extra flash writes should be avoided during high-rate control.
- Current `set_board_config` is sent over classic CAN with 8-byte data frames.
  For per-servo config, byte0 is command and byte1 is servo index, so only
  byte2..byte7 remain for the actual setting payload.
- Single `uint8`, `uint16`, `int16`, `uint32`, and `int32` raw values fit in
  one config frame. Two `int16` values fit. Three `uint16` values fit. Two
  `int32` values do not fit and need separate commands or a multi-frame config
  protocol.
- `mode`, `kp`, and `kd` should use separate low-rate commands. `kp` and `kd`
  use two `int16` values each, so one gain setting fits in one CAN config frame.
- The fractional scale must be fixed globally, for example
  `GAIN_DECIMAL_SCALE = 10000`, otherwise VIM4, Spinal, and Neuron may decode
  different numeric gains from the same integer fields.
- Current Spinal service callback returns `success=true` immediately after
  sending the CAN config request. It does not wait for Neuron acknowledgement,
  flash write success, or Dynamixel register write/readback verification.
- Safety-critical settings such as mode, current limit, and MIT gains should be
  verified by a later `get_board_info` readback, or by adding explicit config
  acknowledgement from Neuron to Spinal.

## 3. Servo Bridge Mode Switching

### Target Design

`servo_bridge` should become a mode-aware command router. The upstream command
can still use `sensor_msgs/JointState`, but `servo_bridge` should interpret the
fields according to each servo's current mode:

Simulation and real machine should share the same upstream mode semantics, but
their downstream behavior is different:

```yaml
simulation:
  DP/DV/DC are routed to matching ros_control controllers.
  MIT is computed in servo_bridge and sent to an effort controller.

real_machine:
  DP/DV/DC are converted to raw ServoTargetStates fields.
  MIT sends raw position + velocity + current_ff to Neuron.
  Neuron computes the MIT current loop.
```

### Modification Boundary

- Mode storage and command parsing:
  `aerial_robot_model/include/aerial_robot_model/servo_bridge.h`,
  `aerial_robot_model/src/servo_bridge/servo_bridge.cpp`.
- Per-servo default mode, simulation controller type, and MIT gains:
  `robots/dragon/config/quad/v1_5_202601/Servo.yaml`.
- Real-machine command output:
  `aerial_robot_nerve/spinal/msg/ServoTargetStates.msg` or the replacement for
  `aerial_robot_nerve/spinal/msg/ServoControlCmd.msg`.
- Simulation controller loading and command publishers:
  `aerial_robot_model/src/servo_bridge/servo_bridge.cpp`.
- Mode readback/config sync with embedded boards:
  `aerial_robot_nerve/spinal/msg/ServoInfo.msg`,
  `aerial_robot_nerve/spinal/srv/SetBoardConfig.srv`.

### Impact

- `servo_bridge` is launched in both simulation and real-machine bringup, so
  mode switching changes both paths.
- Existing Dragon controllers currently publish position-only `joints_ctrl` and
  `gimbals_ctrl`; they should keep working by using `DP` as the default mode.
- In simulation, switching between position, velocity, and effort modes may
  require loading different ros_control controller types. Controller switching
  should be explicit to avoid competing controllers on the same joint.
- In real machine, `MIT` should not close the loop in `servo_bridge`; ROS/CAN
  latency and jitter are too high. `servo_bridge` should only convert units and
  forward raw targets to Neuron.
- Any mode switch can create command discontinuity. On mode change, servo shouble be disabled.

## 5. Define Unit Conversion Boundary

### Recommended Boundary

Keep SI units on VIM4/controller side and raw hardware units below
`servo_bridge`.

```yaml
controller: rad, rad/s, Nm
servo_bridge: converts SI <=> raw bits
Spinal: forwards raw command values
Neuron: runs low-level raw control and writes Dynamixel registers
```

### Conversion

```yaml
position_bit = zero + angle_sgn * q_rad / angle_scale
velocity_bit = angle_sgn * dq_rad_s / velocity_scale
current_bit  = angle_sgn * tau_Nm / torque_scale
```

If MIT gains are configured in SI units on VIM4:

```yaml
kp_raw = kp_SI * angle_scale / torque_scale
kd_raw = kd_SI * velocity_scale / torque_scale
```

Neuron-side MIT current command:

```yaml
goal_current =
  current_ff_raw
  + kp_raw * (target_position_raw - present_position_raw)
  + kd_raw * (target_velocity_raw - present_velocity_raw)
```

### Modification Boundary

### Impact

- Keeping conversion in `servo_bridge` avoids duplicating robot-specific
  `zero_point_offset`, sign, pulley, and scale logic in firmware.
- If conversion is moved into Spinal/Neuron later, every robot-specific scale
  must also move into embedded configuration.
- `kp` and `kd` are configured as two `int16` values: integer part and
  fractional part. The fractional scale must be identical in `servo_bridge`,
  Spinal, and Neuron.
- Current saturation rules must be defined before using MIT mode on hardware.


## 2. Unify High-Rate Servo Command

### Target Interface

Current 15bit feedback limit range:
```yaml
group	angle_scale	            zero	15-bit  raw position range
joints	0.00076699  rad/bit	    2047	[-14.13, 10.99] rad
gimbals	0.001534    rad/bit	    2047	[-28.27, 21.99] rad
--
group	velocity_scale	                15-bit raw velocity range
joints	0.01199041  rad/s/bit	        [-196.45, 196.44] rad/s
gimbals	0.02398082  rad/s/bit	        [-392.91, 392.89] rad/s
```

Use one high-rate servo target command:
```yaml
# old
# ServoControlCmd.msg
uint8[] index # from 0
int16[] angles # dynamixel: 15bit[-32768, 32767]
# ServoTorqueCmd.msg
uint8[] index # from 0
uint8[] torque_enable # 0 or 1

# ServoTargetStates.msg
uint8[] index
int16[] position
int16[] velocity
int16[] current
```

Keep torque enable as a separate safety command:

```shell
servo/target_states
servo/torque_enable
```

### Modification Boundary

- Add/replace ROS message in `aerial_robot_nerve/spinal/msg`.
- Update `aerial_robot_model/src/servo_bridge/servo_bridge.cpp`.
- Update Spinal servo subscribers in
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/Spine/spine.cpp`.
- Update Spinal CAN servo packing in
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/servo`.
- Update Neuron CAN servo decoding in
  `aerial_robot_nerve/neuron/neuronlib/Servo/servo.cpp`.

### Impact

- The current split command path, `servo/target_states` for position and
  `servo/target_current` for current, can be merged.
- Old publishers using `joints_ctrl`, `gimbals_ctrl`, remove `*_torque_ctrl`.


## 2. Expand Servo CAN Command Range

### Current Limitation

Current CAN servo position packing uses one `uint16` per servo:

```yaml
bit15: ack flag
bit14..0: signed 15-bit goal position
```

The effective goal position range is 32 bit:

```yaml
[-16384, 16383]
```

This does not reduce the raw LSB resolution inside the valid range. It reduces
the representable target range because the original Dynamixel Goal Position is a
4-byte register. To supped up CAN commumication efficiency and work with MIT mode, we set the `Position` and `Velocity` bit as `int16` and add stondalone `ack` function through CAN. 

### Modification Boundary

- Change `CANServo::sendData()` packing in Spinal.
- Change `Servo::receiveDataCallback()` decoding in Neuron.
- Add new CAN message IDs or multi-frame format for `int16 position` and
  `int16 velocity`.
- Add standalone `ack` logic.

### Impact

- Current single-turn Dynamixel targets such as `0..4095` are safe.
- Multi-turn position, external encoder absolute position, or large raw MIT
  targets can overflow or wrap in the current 15-bit format.

## 

## 6. Implement MIT Mode in Neuron

### Recommended Placement

Implement the MIT-style loop in Neuron, not VIM4, because Neuron is closest to
the Dynamixel UART timing and servo feedback.

### Modification Boundary

- Neuron servo update loop computes target current from raw position, velocity,
  and feedforward current.
- Spinal remains a router/cache and should not own robot-specific unit
  conversion.
- `current_limit` must clamp the final current command before writing
  `CTRL_GOAL_CURRENT`.

### Impact

- The loop frequency is limited by Neuron Dynamixel update timing, currently
  around 50 Hz for goal writes.
- Poor `kp/kd` or wrong `torque_scale` can create unstable current commands.
- Bench testing should start with rotors removed and low current limits.

## 7. Recheck Torque Scale and Servo Model

### Current Issue

Dragon `Servo.yaml` currently documents:

```text
torque_scale = 0.0106
0.0036 [A/bit] * (3.4 / 2.3) [Nm/A] * 2 pulley
```

This does not fully match the official XM430-W350 data. The formula itself is:

```text
torque_scale [Nm/bit]
  = current_unit [A/bit] * torque_constant [Nm/A] * transmission_ratio
```

For XM430-W350, using `2.69 mA/bit`, `4.1 Nm`, `2.3 A`, and a `2:1` pulley:

```text
0.00269 * (4.1 / 2.3) * 2 = 0.00959 Nm/bit
```

### Modification Boundary

- Verify actual Dynamixel model per servo: XM430-W350 vs XH430-W350, etc.
- Add `model_number` readback to `ServoInfo` or board info if needed.
- Correct `torque_scale` and comments in Dragon `Servo.yaml`.
- Calibrate with real load/current measurement if torque control becomes
  important.

### Impact

- Wrong `torque_scale` directly affects effort feedback, current feedforward,
  and MIT-mode gain conversion.
- Current config is roughly 10% higher than the simple XM430-W350 estimate.
- Stall torque/current are not continuous operating torque, so final values
  should be treated as estimates until calibrated.

## 8. Keep Fine Servo Operations Low-Rate

### Current Fine Operations

These should stay in `set_board_config` / `get_board_info` style services:

```text
homing offset
PID gain
profile velocity
current limit
send_data_flag
external_encoder_flag
joint_resolution / servo_resolution
pulley skip threshold
```

### Modification Boundary

- Spinal config forwarding:
  `aerial_robot_nerve/spinal/mcu_project/lib/Hydrus_Lib/CANDevice/initializer`.
- Neuron config application:
  `aerial_robot_nerve/neuron/neuronlib/Initializer`.
- ROS tools:
  `servo_monitor.py`, `board_configurator.py`, `servo_rough_calib.py`.

### Impact

- `external_encoder_flag` and homing changes should not be high-rate commands.
- Some settings should only apply when torque is disabled.
- Configuration paths write persistent settings, so they should remain explicit.

## 9. Servo Bridge Cleanup

### Current Issue

`servo_ctrl_sub_` and `servo_torque_ctrl_sub_` are conceptually mixed:

```text
servo_ctrl_sub_: position command, optional effort/current
servo_torque_ctrl_sub_: current-only command
```

### Modification Boundary

- Replace internal bridge output with unified `ServoTargetStates`.
- Keep group-level input topics temporarily:
  `joints_ctrl`, `gimbals_ctrl`, `joints_torque_ctrl`, `gimbals_torque_ctrl`.
- Deprecate `*_torque_ctrl` after downstream controllers publish unified target.
- Check the current effort-size condition in `servoTorqueCtrlCallback`.

### Impact

- The bridge becomes the only SI-to-raw conversion boundary.
- Fewer Spinal topics make embedded routing simpler.
- Old controllers need a staged migration.

## 10. Validation Plan

### Software Checks

- Regenerate ROS messages and rosserial headers.
- Build affected packages with `catkin build`.
- Unit-check bridge conversions for position, velocity, current, `kp`, and `kd`.
- Confirm old and new topic compatibility during migration.

### Hardware Checks

- Verify CAN decode with one Neuron before enabling all four.
- Confirm Dynamixel model number and current unit.
- Test torque enable/disable separately from target command.
- Test MIT mode with low current limit and rotors disabled.
