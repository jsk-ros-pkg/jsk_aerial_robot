uint8 neuron_id
float32[3] acc    # m/s^2
float32[3] gyro    # rad/s
float32[3] rpy  # rad

# low resolution for light weight
# int16[3] gyro    # bit (sacle: 2000.0f / 32767.0f * M_PI / 180.0f)
# int16[3] angles  # 1000 rad
