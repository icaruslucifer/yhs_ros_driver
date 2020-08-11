/* 
 * yhs_base.hpp
 * 
 * Created on: Jun 04, 2019 01:22
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef YHS_BASE_HPP
#define YHS_BASE_HPP

#include <string>
#include <cstdint>
#include <thread>
#include <mutex>
#include <functional>

#include "yhs_protocol.h"
#include "yhs_types.hpp"


#   include <stdio.h>
#   include <stdlib.h>
#   include <string.h>
#   include <strings.h>
#   include <unistd.h>
#   include <sys/types.h>
#   include <sys/stat.h>
#   include <fcntl.h>
#   include <pthread.h>
#   include "controlcan.h"
#   define msleep(ms)  usleep((ms)*1000)



#pragma pack(1)
struct CanFrame {
    uint32_t can_id;
    uint8_t data[8];
    uint32_t flags;
    uint8_t can_dlc;
};
#pragma pack()

namespace wescore
{
class YHSBase
{
public:
    YHSBase() ;
    ~YHSBase();

    // do not allow copy
    YHSBase(const YHSBase &yhs) = delete;
    YHSBase &operator=(const YHSBase &yhs) = delete;

public:
    // connect to roboot from CAN or serial
    void Connect(std::string can_name, std::string baudrate);

    // disconnect from roboot, only valid for serial port
    void Disconnect();

    // cmd thread runs at 100Hz (10ms) by default
    void SetCmdThreadPeriodMs(int32_t period_ms) { cmd_thread_period_ms_ = period_ms; };

    // motion control
    void SetMotionCommand(YHSMotionCmd cmd);

    // light control
    void SetLightCommand(YHSLightCmd cmd);
    void DisableLightCmdControl();

    // get robot state
    YHSState GetYHSState();

private:

    // CAN priority higher than serial if both connected
    bool can_connected_ = false;

    // serial port related variables
    uint8_t tx_cmd_len_;
    uint8_t tx_buffer_[YHS_CMD_BUF_LEN];

    // cmd/status update related variables
    std::thread cmd_thread_;
    std::thread receive_thread_;



    //yhs param
    double m_max_speed;
    double m_max_left_steer;
    double m_max_right_steer;

    std::mutex yhs_state_mutex_;
    std::mutex motion_cmd_mutex_;
    std::mutex light_cmd_mutex_;

    YHSState yhs_state_;
    YHSMotionCmd current_motion_cmd_;
    YHSLightCmd current_light_cmd_;

    int32_t cmd_thread_period_ms_ = 10;
    bool cmd_thread_started_ = false;
    bool recv_thread_started_ = false;

    bool light_ctrl_enabled_ = false;
    bool light_ctrl_requested_ = false;

    // internal functions
    void ConfigureCANBus(std::string can_name, std::string baudrate);

    void GetBaudrateValue(const char *str, int *para0, int *para1);

    void StartCmdThread();
    void ControlLoop(int32_t period_ms);

    void ReceiveLoop();

    void SendMotionCmd(uint8_t count);
    void SendLightCmd(uint8_t count);

    uint8_t ParseCANFrame(CanFrame canFrame);

    //和校验
    uint8_t CalcYHSCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc);

    //异或校验
    uint8_t CalcYHSCANCheckXOR(uint16_t id, uint8_t *data, uint8_t dlc);
    uint8_t getCANCount(uint8_t count);


    void SendGearControlFrame(uint8_t count,YHSMotionCmd & cmd);
    void SendSteerControlFrame(uint8_t count,YHSMotionCmd & cmd);
    void SendWheelControlFrame(uint8_t count,YHSMotionCmd & cmd);
    void SendBrakeControlFrame(uint8_t count,YHSMotionCmd & cmd);
    void SendParkingControlFrame(uint8_t count,YHSMotionCmd & cmd);


public:

};
} // namespace wescore

#endif /* YHS_BASE_HPP */
