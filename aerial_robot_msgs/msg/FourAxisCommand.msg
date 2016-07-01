float32[2] angles  #roll and pitch, [rad]   #old: 0.1degree
float32[6] yaw_pi_term # f[N](the m_f_rate is include in gain map)  old: f[N] * 10000 (scale by 10e4)
float32[6] throttle_pid_term # f[N], old:PWM (1000 ~ 2000, exaclty: 1300 ~ 1750)
