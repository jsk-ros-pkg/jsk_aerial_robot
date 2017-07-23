uint8 DRONE = 0 # general untranformable drone
uint8 HYDRUS = 1 # 2 dimensional aerial robot without loop
uint8 RING = 2 # 2 dimensional aerial robot with closing loop
uint8 STAR = 3 # 2 dimensional aerial robot with several branches
uint8 DRAGON = 4 # 3 dimensional aerial robot

uint8 motor_num # equal with link number
uint8 uav_model # drone/hydrus/ring/star/dragon
uint8 baselink # the link attached with flight controller board
