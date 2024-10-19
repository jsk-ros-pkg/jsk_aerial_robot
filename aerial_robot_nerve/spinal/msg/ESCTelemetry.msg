# check the KISS protocol for details: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/

int8 temperature  # 1 degree
uint16 voltage  # 1000 = 10.00V
uint16 current  # 1000 = 10.00A
uint16 consumption  # 1mAh
uint32 rpm  # 1 rpm
uint8 crc_error  # calculated crc - received crc