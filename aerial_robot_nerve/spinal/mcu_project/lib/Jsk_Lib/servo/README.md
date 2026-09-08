# Spinal: Direct and Neuron servo control
## What is this
This is a universal interface for servo motor drivers.
## How to use
### Dynamixel
#### Setting
The YAML keeps `id` as the global ROS control index and addresses configuration
with `board_id + servo_index`. `servo_index` is the stable board-local slot in
`BoardInfo.servos`; it is not the physical Dynamixel ID. The MCU resolves that
ID internally, and its Flash remains keyed by the discovered hardware ID.

```yaml
servo_controller:
  auto_configure_direct_servo: true
  ServoFeedBackFrequency: 50       # MCU -> PC feedback [Hz]
  ServoTargetWatchdog: 0.1         # DV/DC/MIT timeout [s]
  dynamixel_ttl_rs485_mixed: true

  joints:
    board_id: 1                   # 0: Spinal, 1..14: Neuron CAN ID
    angle_sgn: 1
    angle_scale: 0.001533981
    velocity_scale: 0.02398082
    torque_scale: 0.0008
    controller1:
      id: 0                        # global ServoControlCmd index
      servo_index: 0               # board-local BoardInfo.servos slot
      bus_type: R                  # optional if valid MCU Flash data exists
      mode: MIT                    # optional; DP, DEP, DV, DC, or MIT
      mit_kp_si: 0.45
      mit_kd_si: 0.008
      current_limit: 100
      # p_gain, i_gain, d_gain, profile_velocity, send_data_flag,
      # external_encoder_flag, joint_resolution, and servo_resolution
      # are optional; omitted fields retain the servo/MCU readback value.
```

For DirectServo, `servo_bridge` converts `sensor_msgs/JointState` into
`spinal/ServoControlCmd` on `servo/target_states`. Empty velocity/current
arrays are allowed; in MIT mode they mean zero target velocity and zero
feed-forward current. MIT Kp/Kd are RAM-only values supplied together by YAML
or RQT; no separate readiness flag is used. `auto_configure_direct_servo`
enables the same host startup check/write sequence for either DirectServo or
the discovered Neuron boards.

All configuration calls include `board_id` and `servo_index`: use board `0` for
DirectServo and `1..14` for a Neuron. Board-level commands ignore
`servo_index`. Per-servo payloads never include the physical Dynamixel ID.
Spinal sends each Neuron a target at 200 Hz; every arrival sets the
synchronization marker consumed by Neuron's continuously running 1 ms servo
task. The configured feedback frequency controls how often that board's servo
states are emitted on ROS.

Please follow these steps **in order**.
##### 1. Connect Spinal and a power source to servo motors.
##### 2. Reboot Spinal to find and initialize servo motors.
- Just after this reboot, you can see the LEDs of the servos blink once.
##### 3. Set basic parameters
- TTL - RS485 mixed flag
  - Flag on existence of mixed TTL and RS485 communication within the entire system.
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 6 data: [value (0 or 1)]" ``
- Per-servo R/T bus type
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 12 data: [82]" `` (`82` = R, `84` = T)
- Send data flag
  - Flag on reading of servo data
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 4 data: [value (0 or 1)]" ``
- External encoder flag
  - Flag on the existence of external encoders.
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 7 data: [value (0 or 1)]" ``
- Servo resolution ratio
  - The ratio of joint resolution to servo resolution
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 8 data: [value (joint resolution), value (servo resolution)]" ``
##### 4. Set Control parameters
- Homing offset
  - Homing offset of servo position.
  - Range: -1,044,479 ~ 1,044,479
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 1 data: [value(offset)]" ``
- PID gains
  - PID gains for positional control
  - Range: 0 ~ 16,383
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 2 data: [value(p gain), value(i gain), value(d gain)]" ``
- Profile velocity
  - Maximum velocity of servo
  - Range: 0 ~ 32767 (0 represents infinity)
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 3 data: [value(velocity)]" ``
- Current limit
  - Limitation of servo current
  - Range: 0 ~ 1,193
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 5 data: [value(current)]" ``
- Persistent logical mode (torque must be disabled)
  - Values: DP=0, DV=1, DC=2, MIT=3, DEP=5. EM=4 remains a runtime-only safety state.
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 9 data: [mode]" ``
- MIT PD gains (RAM only; torque must be disabled)
  - Gain is encoded as `integer + decimal / 10000`.
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 10 data: [kp integer, kp decimal, kd integer, kd decimal]" ``
- Board feedback frequency and watchdog
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 13 data: [frequency Hz]" ``
  - ``rosservice call /direct_servo_config "board_id: 0 servo_index: 0 command: 15 data: [watchdog ms]" ``
