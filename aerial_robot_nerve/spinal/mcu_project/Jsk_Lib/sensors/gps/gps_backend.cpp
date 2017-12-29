#include "sensors/gps/gps_backend.h"


/* define static class member, should be in this *.cpp, ros realted files in special */
RingBufferFiFo<uint8_t, RING_BUFFER_SIZE> GPS_Backend::gps_rx_buf_;
uint8_t GPS_Backend::gps_rx_value_[GPS_RX_SIZE];
uint16_t GPS_Backend::gps_rx_size_;