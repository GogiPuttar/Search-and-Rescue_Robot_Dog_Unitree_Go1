# High level command message.

# Fixed variables
# Fixed
uint8[2] head
# Fixed control level being high level. 
uint8 level_flag
# Fixed
uint8 frame_reserve

#
#
uint32[2] sn
#
uint32[2] version
#
uint16 band_width
# Can be 0, 1, 2, 5, 6
uint8 mode 

#
#
uint8 gait_type		   
#
uint8 speed_level			   
#
float32 foot_raise_height		   
#
float32 body_height	   
#
float32[2] position 
# Roll, Pitch, Yaw (-0.2 - 0.2)
float32[3] euler	   
# X is straight ahead, Y is to the left, Z is up
float32[2] velocity 
#
float32 yaw_speed
#				   
BmsCmd bms
#
LED[4] led
#
uint8[40] wireless_remote
#
uint32 reserve

#
uint32 crc