/* 
 * yhs_state.hpp
 * 
 * Created on: Jun 11, 2019 08:48
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef YHS_STATE_HPP
#define YHS_STATE_HPP

#include <cstdint>
#include <iostream>

namespace wescore
{
struct YHSState{
    uint8_t gear_actived;
    uint8_t gear_value;
    uint8_t steer_actived;
    double steer_value;

    uint8_t wheel_actived;
    double wheel_value;

    uint8_t brake_actived;
    double brake_value;


    uint8_t parking_actived;
    uint8_t parking_value;

    uint8_t odom_actived;
    uint8_t car_mode;
    
};

struct YHSMotionCmd{
    
    double steer;
    double wheel;
    double brake;
    uint8_t parking;
    uint8_t gear;

};

struct YHSLightCmd{
    uint8_t front_actived;
    uint8_t left_actived;
    uint8_t right_actived;                                                                  

};


} // namespace wescore

#endif /* YHS_STATE_HPP */
