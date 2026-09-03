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
