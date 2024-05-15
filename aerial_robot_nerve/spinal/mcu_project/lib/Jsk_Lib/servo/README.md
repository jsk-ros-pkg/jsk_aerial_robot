# Spinal: Direct servo control
## What is this
This is a universal interface for servo motor drivers.
## How to use
### Dynamixel
#### Setting
Please follow these steps **in order**.
##### 1. Connect Spinal and a power source to servo motors.
##### 2. Reboot Spinal to find and initialize servo motors.
- Just after this reboot, you can see the LEDs of the servos blink once.
##### 3. Set basic parameters
- TTL - RS485 mixed flag
  - Flag on existence of mixed TTL and RS485 communication within the entire system.
  - ``rosservice call /direct_servo_config "command: 6 data: [value (0 or 1)]" ``
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
  - ``rosservice call /direct_servo_config "command: 2 data: [servo id, value(p gain), value(d gain), value(i gain)]" ``
- Profile velocity
  - Maximum velocity of servo
  - Range: 0 ~ 32767 (0 represents infinity)
  - ``rosservice call /direct_servo_config "command: 3 data: [servo id, value(velocity)]" ``
- Current limit
  - Limitation of servo current
  - Range: 0 ~ 1,193
  - ``rosservice call /direct_servo_config "command: 5 data: [servo id, value(current)]" ``
