int16[2] angles  #roll and pitch[0.1degree]
int32[4] yaw_pi_term # f[N] * 10000 (scale by 10e4)
uint16[4] throttle_pid_term # PWM (1000 ~ 2000, exaclty: 1300 ~ 1750)
