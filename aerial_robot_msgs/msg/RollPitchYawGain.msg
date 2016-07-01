#this is unofficial way to send 3 axis gain to flight controller
# 4 x 6 x 7 = 168 byte 

#gains
float32[6] roll_p_gain #motor1->6
float32[6] roll_i_gain #motor1->6
float32[6] roll_d_gain #motor1->6

float32[6] pitch_p_gain #motor1->6
float32[6] pitch_i_gain #motor1->6
float32[6] pitch_d_gain #motor1->6

float32[6] yaw_d_gain #motor1->6
