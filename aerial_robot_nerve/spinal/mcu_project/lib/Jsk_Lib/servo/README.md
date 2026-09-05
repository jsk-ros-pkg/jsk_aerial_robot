# Spinal: Direct servo control
## What is this
This is a universal interface for servo motor drivers.
## How to use
### Dynamixel
#### Setting
Stage 2 DirectServo configuration commands use the physical Dynamixel ID, not
the discovery order or the ROS servo-array index. Each 20-byte Flash slot stores
the ID, bus, persistent mode, packed send/external flags, gains, profile velocity,
and encoder data. A detected ID without a matching slot remains unconfigured.

```yaml
servo_controller:
  auto_configure_direct_servo: true
  ServoFeedBackFrequency: 50       # MCU -> PC feedback [Hz]
  ServoTargetWatchdog: 0.1         # DV/DC/MIT timeout [s]
  dynamixel_ttl_rs485_mixed: true

  joints:
    angle_sgn: 1
    angle_scale: 0.001533981
    velocity_scale: 0.02398082
    torque_scale: 0.0008
    controller1:
      id: 0                        # ROS runtime index
      physical_id: 5               # Dynamixel ID
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
enables the host startup check/write sequence.

Please follow these steps **in order**.
##### 1. Connect Spinal and a power source to servo motors.
##### 2. Reboot Spinal to find and initialize servo motors.
- Just after this reboot, you can see the LEDs of the servos blink once.
##### 3. Set basic parameters
- TTL - RS485 mixed flag
  - Flag on existence of mixed TTL and RS485 communication within the entire system.
  - ``rosservice call /direct_servo_config "command: 6 data: [value (0 or 1)]" ``
- Per-servo R/T bus type
  - ``rosservice call /direct_servo_config "command: 12 data: [physical servo id, 82 (R) or 84 (T)]" ``
- Send data flag
  - Flag on reading of servo data
  - ``rosservice call /direct_servo_config "command: 4 data: [servo id, value (0 or 1)]" ``
- External encoder flag
  - Flag on the existence of external encoders.
  - ``rosservice call /direct_servo_config "command: 7 data: [servo id, value (0 or 1)]" ``
- Servo resolution ratio
  - The ratio of joint resolution to servo resolution
  - ``rosservice call /direct_servo_config "command: 8 data: [servo id, value (joint resolution), value (servo resolution)]" ``
##### 4. Set Control parameters
- Homing offset
  - Homing offset of servo position.
  - Range: -1,044,479 ~ 1,044,479
  - ``rosservice call /direct_servo_config "command: 1 data: [servo id, value(offset)]" ``
- PID gains
  - PID gains for positional control
  - Range: 0 ~ 16,383
  - ``rosservice call /direct_servo_config "command: 2 data: [servo id, value(p gain), value(i gain), value(d gain)]" ``
- Profile velocity
  - Maximum velocity of servo
  - Range: 0 ~ 32767 (0 represents infinity)
  - ``rosservice call /direct_servo_config "command: 3 data: [servo id, value(velocity)]" ``
- Current limit
  - Limitation of servo current
  - Range: 0 ~ 1,193
  - ``rosservice call /direct_servo_config "command: 5 data: [servo id, value(current)]" ``
- Persistent logical mode (torque must be disabled)
  - Values: DP=0, DV=1, DC=2, MIT=3, DEP=5. EM=4 remains a runtime-only safety state.
  - ``rosservice call /direct_servo_config "command: 9 data: [servo id, mode]" ``
- MIT PD gains (RAM only; torque must be disabled)
  - Gain is encoded as `integer + decimal / 10000`.
  - ``rosservice call /direct_servo_config "command: 10 data: [servo id, kp integer, kp decimal, kd integer, kd decimal]" ``
- Board feedback frequency and watchdog
  - ``rosservice call /direct_servo_config "command: 13 data: [frequency Hz]" ``
  - ``rosservice call /direct_servo_config "command: 15 data: [watchdog ms]" ``
