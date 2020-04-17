time stamp

uint8 FIX_TYPE_NO_FIX = 0
uint8 FIX_TYPE_DEAD_RECKONING_ONLY = 1
uint8 FIX_TYPE_2D = 2                           # Signal from only 3 SVs, constant altitude assumed
uint8 FIX_TYPE_3D = 3
uint8 FIX_TYPE_GNSS_DEAD_RECKONING_COMBINED = 4 # GNSS + Dead reckoning
uint8 FIX_TYPE_TIME_ONLY = 5                    # Time only fix (High precision
uint8 status           # GNSS fix Type, range 0..5 devices)

uint16 year             # Year (UTC)
uint8 month             # Month, range 1..12 (UTC)
uint8 day               # Day of month, range 1..31 (UTC)
uint8 hour              # Hour of day, range 0..23 (UTC)
uint8 min               # Minute of hour, range 0..59 (UTC)
uint8 sec               # Seconds of minute, range 0..60 (UTC)
int32 nano              # fraction of a second [ns], range -1e9 .. 1e9 (UTC)
uint8 time_valid         # Validity flags

float64[2] location
float32 h_acc             # Horizontal Position Accuracy Estimate [mm]
float32[2] velocity
float32 v_acc             # Velocity Accuracy Estimate [mm/s]

uint8 sat_num
