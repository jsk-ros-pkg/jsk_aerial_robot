float32 min_pwm
float32 max_pwm
float32 min_thrust # if min_thrust > 0, then the min_pwm is recalculated with min_thrust and voltage, important for yaw saturation process
float32 force_landing_thrust
uint8 pwm_conversion_mode
spinal/MotorInfo[] motor_info
