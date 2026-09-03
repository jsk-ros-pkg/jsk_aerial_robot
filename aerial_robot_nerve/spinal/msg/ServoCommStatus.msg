# Cumulative communication diagnostics since servo-driver initialization.
# One response-producing instruction counts as one transaction, regardless of
# the number of servo status packets contained in its response.

time stamp

uint32 transaction_success_count
uint32 timeout_count
uint32 crc_error_count
uint32 invalid_packet_count
uint32 recovery_count
uint32 deadline_miss_count
uint32 queue_overflow_count
uint32 unexpected_completion_count

uint16 queue_high_water_mark

# Per-servo cumulative counters. Arrays share the same index.
uint8[] servo_ids
# Accepted combined Current/Velocity/Position feedback blocks.
uint32[] cur_vel_pos_update_count
# Goal Sync Writes successfully transmitted by the MCU; not servo acknowledgments.
uint32[] goal_write_count
