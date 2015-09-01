#this is unofficial way to send 3 axis gain to controller1

#rotate angle from imu board to cog
int16 angle_cos
int16 angle_sin

#gains
float32[4] roll_p_gain #motor1->4
float32[4] roll_i_gain #motor1->4
float32[4] roll_d_gain #motor1->4

float32[4] pitch_p_gain #motor1->4
float32[4] pitch_i_gain #motor1->4
float32[4] pitch_d_gain #motor1->4

float32[4] yaw_d_gain #motor1->4
