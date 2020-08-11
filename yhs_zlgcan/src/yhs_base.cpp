#include "yhs_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

#include <ros/ros.h>

int  DEVICE_INDEX = 0;
int  CANID = 0;

namespace
{
// source: https://github.com/rxdu/stopwatch
struct StopWatch
{
    using Clock = std::chrono::high_resolution_clock;
    using time_point = typename Clock::time_point;
    using duration = typename Clock::duration;

    StopWatch() { tic_point = Clock::now(); };

    time_point tic_point;

    void tic()
    {
        tic_point = Clock::now();
    };

    double toc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count() / 1000000.0;
    };

    // for different precisions
    double stoc()
    {
        return std::chrono::duration_cast<std::chrono::seconds>(Clock::now() - tic_point).count();
    };

    double mtoc()
    {
        return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();
    };

    double utoc()
    {
        return std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();
    };

    double ntoc()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - tic_point).count();
    };

    // you have to call tic() before calling this function
    void sleep_until_ms(int64_t period_ms)
    {
        int64_t duration = period_ms - std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(duration));
    };

    void sleep_until_us(int64_t period_us)
    {
        int64_t duration = period_us - std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - tic_point).count();

        if (duration > 0)
            std::this_thread::sleep_for(std::chrono::microseconds(duration));
    };
};
} // namespace


namespace wescore{
    YHSBase::YHSBase(){
        can_connected_ = false;

        m_max_speed = 1.5;
        m_max_left_steer = 24.0;
        m_max_right_steer = 24.0;
    }

    YHSBase::~YHSBase(){
        if (cmd_thread_.joinable())
            cmd_thread_.join();
        if(receive_thread_.joinable())
            receive_thread_.join();
    }

    void YHSBase::Connect(std::string can_name, std::string baudrate){

        ConfigureCANBus(can_name,baudrate);
    }

    void YHSBase::Disconnect(){

    }

    void YHSBase::ConfigureCANBus(std::string can_name, std::string baudrate){

        bool created = true;

        if(can_name == "can1"){
            DEVICE_INDEX = 1;
            CANID = 1;
        }else{
            DEVICE_INDEX = 0;
            CANID = 0;
        }
        

        int stat = VCI_OpenDevice(VCI_USBCAN2, DEVICE_INDEX, 0);
        if(stat <= 0){
            created = false;
            ROS_INFO("USB CAN Open Failed!");
        }else{
            ROS_INFO("USB CAN Open Success!");
            VCI_INIT_CONFIG config = {0};
            config.AccCode = 0;
            config.AccMask = 0xFFFFFFFF;
            config.Reserved = 0;
            config.Filter = 1;
            config.Mode = 0;
            int time0 = 0x00;
            int time1 = 0x1C;
            //默认500K
            GetBaudrateValue(baudrate.c_str(),&time0,&time1);
            config.Timing0 = time0;
            config.Timing1 = time1;
            // init

            stat = VCI_InitCAN(VCI_USBCAN2, DEVICE_INDEX, CANID, &config);
            if (stat <= 0) {
                created = false;
                ROS_INFO("VCI_InitCAN error, channel=%d", CANID);
                VCI_CloseDevice(VCI_USBCAN2, DEVICE_INDEX);
            }
            // reset
            stat =  VCI_ResetCAN(VCI_USBCAN2, DEVICE_INDEX, CANID);
            if (stat <= 0) {
                created = false;
                ROS_INFO("VCI_ResetCAN error, channel=%d", CANID);
                VCI_CloseDevice(VCI_USBCAN2, DEVICE_INDEX);
            }
            // start
            stat = VCI_StartCAN(VCI_USBCAN2, DEVICE_INDEX, CANID);
            if (stat <= 0) {
                created = false;
                ROS_INFO("VCI_StartCAN error, channel=%d", CANID);
                VCI_CloseDevice(VCI_USBCAN2, DEVICE_INDEX);
            }
        }

        if(created){
            can_connected_ = true;
            receive_thread_ = std::thread(std::bind(&YHSBase::ReceiveLoop,this));
            recv_thread_started_ = true;
        }
    }

    void YHSBase::GetBaudrateValue(const char *str, int *para0, int *para1){
        if (str == NULL) {
            // default 500 kbps
            str = "500k";
        }
        if (str) {
            if (strcasecmp(str, "125k") == 0) {
                *para0 = 0x03;
                *para1 = 0x1C;
                ROS_INFO("baudrate = 125k");
            } else if (strcasecmp(str, "250k") == 0) {
                *para0 = 0x01;
                *para1 = 0x1C;
                ROS_INFO("baudrate = 250k");
            } else if (strcasecmp(str, "500k") == 0) {
                *para0 = 0x00;
                *para1 = 0x1C;
                ROS_INFO("baudrate = 500k");
            } else if (strcasecmp(str, "1000k") == 0) {
                *para0 = 0x00;
                *para1 = 0x14;
                ROS_INFO("baudrate = 1000k");
            } else {
                ROS_INFO("unsupport baudrate='%s', using default = 250k", str);
                *para0 = 0x01;
                *para1 = 0x1C;
            }
        }
    }


    void YHSBase::StartCmdThread(){

        cmd_thread_ = std::thread(std::bind(&YHSBase::ControlLoop, this, cmd_thread_period_ms_));
        cmd_thread_started_ = true;
    }

    void YHSBase::SendMotionCmd(uint8_t count)
    {
        // motion control message
        YHSMotionCmd cmd;
        motion_cmd_mutex_.lock();
        cmd = current_motion_cmd_;
        motion_cmd_mutex_.unlock();
        if (can_connected_){
            SendGearControlFrame(count,cmd);
            usleep(10);
            SendSteerControlFrame(count,cmd);
            usleep(10);
            SendWheelControlFrame(count,cmd);
            usleep(10);
            SendBrakeControlFrame(count,cmd);
            usleep(10);
            SendParkingControlFrame(count,cmd);    
        }


    }

    void YHSBase::SendLightCmd(uint8_t count){
        if (can_connected_){
            VCI_CAN_OBJ l_frame;
            l_frame.SendType = 0;
            l_frame.ID = CAN_MSG_LIGHT_CONTROL_CMD_ID;
            l_frame.DataLen = 8;
            l_frame.RemoteFlag = 0;
            l_frame.ExternFlag = 1;
            
            light_cmd_mutex_.lock();
            l_frame.Data[0] = current_light_cmd_.front_actived;
            l_frame.Data[1] = current_light_cmd_.right_actived;
            l_frame.Data[2] = current_light_cmd_.left_actived;
            light_ctrl_requested_ = false;
            light_cmd_mutex_.unlock();

            l_frame.Data[3] = 0;
            l_frame.Data[4] = 0;
            l_frame.Data[5] = 0;
            l_frame.Data[6] = getCANCount(count);
            l_frame.Data[7] = CalcYHSCANCheckXOR(l_frame.ID, l_frame.Data, l_frame.DataLen);
            int stat = VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CANID, &l_frame, 1);
            if (stat < 0) {
                ROS_INFO("ch%d VCI_Transmit light error", CANID);
            }else{
                std::cout << "can light cmd: ";
                for (int i = 0; i < 8; ++i)
                    std::cout << static_cast<int>(l_frame.Data[i]) << " ";
                std::cout <<"\n";
            }
        }
    }

    void YHSBase::ControlLoop(int32_t period_ms){
        StopWatch ctrl_sw;
        uint8_t cmd_count = 0;
        uint8_t light_cmd_count = 0;
        while (ros::ok()){
            ctrl_sw.tic();
            if(cmd_count == 3){
                cmd_count = 0;
            }
            // motion control message
            SendMotionCmd(cmd_count++);

            // check if there is request for light control
            if (light_ctrl_requested_)
                SendLightCmd(light_cmd_count++);

            ctrl_sw.sleep_until_ms(period_ms);
            // std::cout << "control loop update frequency: " << 1.0 / ctrl_sw.toc() << std::endl;
        }
    }

    YHSState YHSBase::GetYHSState(){
        return yhs_state_;
    }

    void YHSBase::SetMotionCommand(YHSMotionCmd cmd){
        if (!cmd_thread_started_)
            StartCmdThread();
        {
            std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
            current_motion_cmd_ = cmd;
        }
    }


    void YHSBase::SetLightCommand(YHSLightCmd cmd){
        if (!cmd_thread_started_)
            StartCmdThread();
        {
            std::lock_guard<std::mutex> guard(light_cmd_mutex_);
            current_light_cmd_ = cmd;
        }
        
        light_ctrl_enabled_ = true;
        light_ctrl_requested_ = true;
    }

    void YHSBase::DisableLightCmdControl(){
        std::lock_guard<std::mutex> guard(light_cmd_mutex_);
        light_ctrl_enabled_ = false;
        light_ctrl_requested_ = true;
    }


    void YHSBase::ReceiveLoop(){
        int stat = 0;
        while (ros::ok()) {
            VCI_CAN_OBJ candata = {0};
            stat = VCI_Receive(VCI_USBCAN2, DEVICE_INDEX, CANID, &candata, 1, 10);
            if (stat > 0) {
                CanFrame frame = {0};
                unsigned char data[8];
                frame.can_id = candata.ID;
                if (candata.ExternFlag) {
                    frame.flags =2;
                }
                if (candata.RemoteFlag) {
                    frame.flags = 3;
                }
                for (int i = 0; i < candata.DataLen; i++) {
                    data[i] = candata.Data[i];
                }
                frame.can_dlc = 8;
                memcpy(frame.data, data, candata.DataLen * sizeof(uint8_t));
                ParseCANFrame(frame);
            }
            usleep(50);
        }
    }

    uint8_t YHSBase::ParseCANFrame(CanFrame canFrame){
        CanFrame  * rx_frame = & canFrame;
            
        // validate checksum, discard frame if fails
        if (!rx_frame->data[7] == CalcYHSCANCheckXOR(rx_frame->can_id, rx_frame->data, rx_frame->can_dlc)){
            std::cerr << "ERROR: checksum mismatch, discard frame with id " << rx_frame->can_id << std::endl;
            return -1;
        }
        // otherwise, update robot state with new frame
        {
            std::lock_guard<std::mutex> guard(yhs_state_mutex_);
            switch (rx_frame->can_id){
                case CAN_MSG_GEAR_FEEDBACK_ID    :
                {
                    yhs_state_.gear_actived = rx_frame->data[0];
                    yhs_state_.gear_value = rx_frame->data[1];
                    break;
                }
                case CAN_MSG_STEER_FEEDBACK_ID   :
                {
                    yhs_state_.steer_actived = rx_frame->data[0];

                    uint16_t f = rx_frame->data[2] << 8;
                    uint8_t b = rx_frame->data[1];
                    double m = static_cast<double>(f+b);
                    yhs_state_.steer_value = m*CAN_STEER_UNIT-90;
                    break;
                }
                case CAN_MSG_WHEEL_FEEDBACK_ID   :
                {
                    yhs_state_.wheel_actived = rx_frame->data[0];
                    double m = static_cast<double>(rx_frame->data[1]);
                    yhs_state_.wheel_value = m*CAN_WHEEL_UNIT;
                    break;
                }
                case CAN_MSG_BRAKE_FEEDBACK_ID   :
                {
                    yhs_state_.brake_actived = rx_frame->data[0];
                    double m = static_cast<double>(rx_frame->data[1]);
                    yhs_state_.brake_value = m*CAN_BRAKE_UNIT;
                    break;
                }
                case CAN_MSG_PARKING_FEEDBACK_ID :
                {
                    yhs_state_.parking_actived = rx_frame->data[0];
                    yhs_state_.parking_value = rx_frame->data[1];
                    break;
                }
                case CAN_MSG_ODOM_CONTROL_CMD_ID    :
                {
                    yhs_state_.odom_actived = rx_frame->data[0];
                    yhs_state_.car_mode = rx_frame->data[5];
                    break;
                }
                case CAN_MSG_ODOM_FEEDBACK_ID   :
                {
                    break;
                }
                case CAN_MSG_LIGHT_FEEDBACK_ID :
                {
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        return 1;
    }

    void YHSBase::SendGearControlFrame(uint8_t count,YHSMotionCmd & cmd){
        VCI_CAN_OBJ gear_frame;
        gear_frame.SendType = 0;
        gear_frame.ID = CAN_MSG_GEAR_CONTROL_CMD_ID;
        gear_frame.DataLen = 8;
        gear_frame.RemoteFlag = 0;
        gear_frame.ExternFlag = 1;
        
        gear_frame.Data[0] = 1;
        gear_frame.Data[1] = cmd.gear;
        gear_frame.Data[2] = 0;
        gear_frame.Data[3] = 0;
        gear_frame.Data[4] = 0;
        gear_frame.Data[5] = 0;
        gear_frame.Data[6] = getCANCount(count);
        gear_frame.Data[7] = CalcYHSCANCheckXOR(gear_frame.ID, gear_frame.Data, gear_frame.DataLen);
        int stat = VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CANID, &gear_frame, 1);
        if (stat < 0) {
            ROS_INFO("ch%d VCI_Transmit gear error", CANID);
        }else{
            std::cout << "can gear cmd: ";
            for (int i = 0; i < 8; ++i)
                std::cout << static_cast<int>(gear_frame.Data[i]) << " ";
            std::cout <<"\n";
        }
        
    }
    void YHSBase::SendSteerControlFrame(uint8_t count,YHSMotionCmd & cmd){
        if(cmd.steer > m_max_left_steer){
            cmd.steer = m_max_left_steer;
        }
        if(cmd.steer < -m_max_right_steer){
            cmd.steer = -m_max_right_steer;
        }

        VCI_CAN_OBJ steer_frame;
        steer_frame.SendType = 0;
        steer_frame.ID = CAN_MSG_STEER_CONTROL_CMD_ID;
        steer_frame.DataLen = 8;
        steer_frame.RemoteFlag = 0;
        steer_frame.ExternFlag = 1;

        uint16_t v = static_cast<uint16_t>((cmd.steer+90)/CAN_STEER_UNIT);

        steer_frame.Data[0] = 1;
        steer_frame.Data[2] = static_cast<uint8_t>(v >> 8);
        steer_frame.Data[3] = 0;
        steer_frame.Data[4] = 0;
        steer_frame.Data[5] = 0;
        steer_frame.Data[6] = getCANCount(count);
        steer_frame.Data[7] = CalcYHSCANCheckXOR(steer_frame.ID, steer_frame.Data, steer_frame.DataLen);
        
        int stat = VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CANID, &steer_frame, 1);
        if (stat < 0) {
            ROS_INFO("ch%d VCI_Transmit steer error", CANID);
        }else{
            std::cout << "can steer cmd: ";
            for (int i = 0; i < 8; ++i)
                std::cout << static_cast<int>(steer_frame.Data[i]) << " ";
            std::cout <<"\n";
        }
        
    }
    void YHSBase::SendWheelControlFrame(uint8_t count,YHSMotionCmd & cmd){

        if(cmd.wheel > m_max_speed){
            cmd.wheel = m_max_speed;
        }
        if(cmd.wheel <0){
            cmd.wheel = 0;
        }

        VCI_CAN_OBJ wheel_frame;
        wheel_frame.SendType = 0;
        wheel_frame.ID = CAN_MSG_WHEEL_CONTROL_CMD_ID;
        wheel_frame.DataLen = 8;
        wheel_frame.RemoteFlag = 0;
        wheel_frame.ExternFlag = 1;

        int v = static_cast<uint8_t>(cmd.wheel/CAN_WHEEL_UNIT);
        wheel_frame.Data[0] = 1;
        wheel_frame.Data[1] = v;
        wheel_frame.Data[2] = 0;
        wheel_frame.Data[3] = 0;
        wheel_frame.Data[4] = 0;
        wheel_frame.Data[5] = 0;
        wheel_frame.Data[6] = getCANCount(count);
        wheel_frame.Data[7] = CalcYHSCANCheckXOR(wheel_frame.ID, wheel_frame.Data, wheel_frame.DataLen);


        int stat = VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CANID, &wheel_frame, 1);
        if (stat < 0) {
            ROS_INFO("ch%d VCI_Transmit wheel error", CANID);
        }else{
            std::cout << "can wheel cmd: ";
            for (int i = 0; i < 8; ++i)
                std::cout << static_cast<int>(wheel_frame.Data[i]) << " ";
            std::cout <<"\n";
        }
    }
    void YHSBase::SendBrakeControlFrame(uint8_t count,YHSMotionCmd & cmd){
        VCI_CAN_OBJ brake_frame;
        brake_frame.SendType = 0;
        brake_frame.ID = CAN_MSG_BRAKE_CONTROL_CMD_ID;
        brake_frame.DataLen = 8;
        brake_frame.RemoteFlag = 0;
        brake_frame.ExternFlag = 1;
        uint8_t v = static_cast<uint8_t>(cmd.brake/CAN_BRAKE_UNIT);
        brake_frame.Data[0] = 1;
        brake_frame.Data[1] = v;
        brake_frame.Data[2] = 0;
        brake_frame.Data[3] = 0;
        brake_frame.Data[4] = 0;
        brake_frame.Data[5] = 0;
        brake_frame.Data[6] = getCANCount(count);
        brake_frame.Data[7] = CalcYHSCANCheckXOR(brake_frame.ID, brake_frame.Data, brake_frame.DataLen);

        int stat = VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CANID, &brake_frame, 1);
        if (stat < 0) {
            ROS_INFO("ch%d VCI_Transmit brake error", CANID);
        }else{
            std::cout << "can brake cmd: ";
            for (int i = 0; i < 8; ++i)
                std::cout << static_cast<int>(brake_frame.Data[i]) << " ";
            std::cout <<"\n";
        }
    }


    void YHSBase::SendParkingControlFrame(uint8_t count,YHSMotionCmd & cmd){
        VCI_CAN_OBJ parking_frame;
        parking_frame.SendType = 0;
        parking_frame.ID = CAN_MSG_PARKING_CONTROL_CMD_ID;
        parking_frame.DataLen = 8;
        parking_frame.RemoteFlag = 0;
        parking_frame.ExternFlag = 1;

        parking_frame.Data[0] = 1;
        parking_frame.Data[1] = cmd.parking;
        parking_frame.Data[2] = 0;
        parking_frame.Data[3] = 0;
        parking_frame.Data[4] = 0;
        parking_frame.Data[5] = 0;
        parking_frame.Data[6] = getCANCount(count);
        parking_frame.Data[7] = CalcYHSCANCheckXOR(parking_frame.ID, parking_frame.Data, parking_frame.DataLen);
        int stat = VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CANID, &parking_frame, 1);
        if (stat < 0) {
            ROS_INFO("ch%d VCI_Transmit parking error", CANID);
        }else{
            std::cout << "can parking cmd: ";
            for (int i = 0; i < 8; ++i)
                std::cout << static_cast<int>(parking_frame.Data[i]) << " ";
            std::cout <<"\n";
        }
        
    }



    uint8_t YHSBase::CalcYHSCANCheckXOR(uint16_t id, uint8_t *data, uint8_t dlc){
        uint8_t check = data[0];
        for (int i = 1; i < (dlc - 1); ++i)
            check = check ^ data[i];
        return check;
    }

    uint8_t YHSBase::CalcYHSCANChecksum(uint16_t id, uint8_t *data, uint8_t dlc){
        uint8_t checksum = 0x00;
        checksum = (uint8_t)(id & 0x00ff) + (uint8_t)(id >> 8) + dlc;
        for (int i = 0; i < (dlc - 1); ++i)
            checksum += data[i];
        return checksum;
    }


    uint8_t YHSBase::getCANCount(uint8_t count){
        return 16*count;
    }


} // namespace wescore
