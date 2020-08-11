Header header

#gear
uint8 gear_actived 
uint8 gear_state      # 1驻车 2倒挡 3空挡 4前进挡

#steer
uint8 steer_actived 
float64  steer_state  #0.043945 度

#wheel
uint8 speed_actived 
float64 speed_state          #0.04 m/s

#brake
uint8 brake_actived 
uint8 brake_state  #0 无制动 1制动中 2完全制动 3reserve

#parking
uint8 parking_actived 
uint8 parking_state  #0x01 parkapplied 0x03 completelyRelease

#wheel speed
uint8 wheelspeed_actived 
float64 left_wheelspeed 
float64 right_wheelspeed         #0.04

#wheel pulse
uint8 wheelpulse_actived 
uint32 left_wheelpulse 
uint32 right_wheelpilse 


#odom
uint8 odom_actived 
float64 odom_state #0.1km
uint8 car_state #0 auto 1 remote 2 stop

#light 
uint8 front_light_actived 
uint8 left_light_actived #
uint8 right_light_actived #

#speak
uint8 speaker_1_actived #
uint8 speaker_2_actived #
uint8 speaker_3_actived #
