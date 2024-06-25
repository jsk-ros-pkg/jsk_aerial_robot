uint8 NO_NAVIGATION = 0
uint8 VEL_MODE = 1
uint8 POS_MODE = 2
uint8 ACC_MODE = 3
uint8 POS_VEL_MODE = 4
uint8 GPS_WAYPOINT_MODE = 5
uint8 STAY_HERE_MODE = 6

uint8 WORLD_FRAME = 0
uint8 LOCAL_FRAME = 1

uint8 BASELINK = 0
uint8 COG = 1

Header header

uint8 control_frame # reference frame: local or world, mainly for vel/acc nav
uint8 target # the target link: baselink or COG
uint8 pos_xy_nav_mode
float64 target_pos_x
float32 target_vel_x
float32 target_acc_x

float64 target_pos_y
float32 target_vel_y
float32 target_acc_y

uint8 roll_nav_mode

float32 target_omega_x
float32 target_roll

uint8 pitch_nav_mode

float32 target_omega_y
float32 target_pitch

uint8 yaw_nav_mode

float32 target_omega_z
float32 target_yaw

uint8 pos_z_nav_mode

float32 target_pos_z
float32 target_vel_z
float32 target_pos_diff_z

