uint32 index #from 0
int16 angle # 15bit[-32768, 32767]
int16 rpm # 15bit[-32768, 32767]
int16 current # power
uint8 state # state

uint8 IDLE = 0
uint8 ALIGNMENT = 2
uint8 START = 4
uint8 RUN = 6
uint8 STOP = 8
uint8 FAULT_NOW = 10
uint8 FAULT_OVER = 11
uint8 ICLWAIT = 12
uint8 CHARGE_BOOT_CAP = 16
uint8 OFFSET_CALIB = 17
uint8 SWITCH_OVER = 19
