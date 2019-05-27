int32 motor_num

#feedback gain
float32[] pos_p_gain_pitch
float32[] pos_i_gain_pitch
float32[] pos_d_gain_pitch

float32[] pos_p_gain_roll
float32[] pos_i_gain_roll
float32[] pos_d_gain_roll

float32[] pos_p_gain_alt
float32[] pos_i_gain_alt
float32[] pos_d_gain_alt

float32[] pos_p_gain_yaw
float32[] pos_i_gain_yaw
float32[] pos_d_gain_yaw

#temporary for extra module weight control
float32 total_mass
