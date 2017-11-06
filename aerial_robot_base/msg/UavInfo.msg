uint8 DRONE = 0 # 0x00, general multirotor with 4dof underactuated property
uint8 HYDRUS = 16 # 0x10, hydrus type with  4dof underactuated property
uint8 HYDRUS_XI = 17 # 0x11, hydrus type with full/over actuated property
uint8 DRAGON = 32 # 0x20, dragon type with full/over actuated property

uint8 motor_num # equal with link number
uint8 uav_model # drone/hydrus/ring/star/dragon
uint8 baselink # the link attached with flight controller board
