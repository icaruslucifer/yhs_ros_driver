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
    }

    YHSBase::~YHSBase(){
        if (cmd_thread_.joinable())
            cmd_thread_.join();
        if (receive_thread_.joinable())
            receive_thread_.join();
    }

    void YHSBase::Connect(std::string can_name, std::string baudrate){

        ConfigureCANBus(can_name,baudrate);
    }

    void YHSBase::Disconnect(){

    }

    void YHSBase::ConfigureCANBus(std::string can_name, std::string baudrate){

        bool created = true;

        int s;
        struct sockaddr_can addr;
        struct ifreq ifr = {0};
        s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (s == -1) {
            ROS_INFO("socket() error, %s", strerror(errno));
            created = false;
        }
        snprintf(ifr.ifr_name, sizeof(ifr.ifr_name), "%s", can_name.c_str());
        if (ioctl(s, SIOCGIFINDEX, &ifr) == -1) {
            ROS_INFO("deivce '%s' error, ", can_name.c_str(), strerror(errno));
            close(s);
            created = false;
        }
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        /* bind */
        if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
            ROS_INFO("bind() error, %s", strerror(errno));
            close(s);
            created = false;
        }

        if(!created){
            can_connected_ = true;
            can_if_ = s;
            receive_thread_ = std::thread(std::bind(&YHSBase::ReceiveLoop,this));
            recv_thread_started_ = true;
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
            // send to can bus
            can_frame l_frame;
            l_frame.can_id = CAN_MSG_LIGHT_CONTROL_CMD_ID;
            l_frame.can_dlc = 8;
            light_cmd_mutex_.lock();
            l_frame.data[0] = current_light_cmd_.front_actived;
            l_frame.data[1] = current_light_cmd_.left_actived;
            l_frame.data[2] = current_light_cmd_.right_actived;
            light_ctrl_requested_ = false;
            light_cmd_mutex_.unlock();

            l_frame.data[3] = 0;
            l_frame.data[4] = 0;
            l_frame.data[5] = 0;
            l_frame.data[6] = count;
            l_frame.data[7] = CalcYHSCANCheckXOR(l_frame.can_id, l_frame.data, l_frame.can_dlc);
            if (write(can_if_, &l_frame, sizeof(l_frame)) == -1) {
                ROS_INFO("write canbus error, %s", strerror(errno));
            }

            std::cout << "cmd: " << static_cast<int>(current_light_cmd_.front_actived) << " , " << static_cast<int>(current_light_cmd_.left_actived) << " , "
                      << static_cast<int>(current_light_cmd_.right_actived)  << std::endl;
            std::cout << "can: ";
            for (int i = 0; i < 8; ++i)
                std::cout << static_cast<int>(l_frame.data[i]) << " ";
        }
    }

    void YHSBase::ControlLoop(int32_t period_ms){
        StopWatch ctrl_sw;
        uint8_t cmd_count = 0;
        uint8_t light_cmd_count = 0;
        while (ros::ok())
        {
            ctrl_sw.tic();
            if(cmd_count == 3){
                cmd_count = 0;
            }
            std::cout<<"count:"<<cmd_count<<"\n";
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
        std::lock_guard<std::mutex> guard(yhs_state_mutex_);
        return yhs_state_;
    }

    void YHSBase::SetMotionCommand(YHSMotionCmd cmd){
        if (!cmd_thread_started_)
            StartCmdThread();
        // make sure cmd thread is started before attempting to send commands
        {
            std::lock_guard<std::mutex> guard(motion_cmd_mutex_);
            current_motion_cmd_ = cmd;
        }
    }


    void YHSBase::SetLightCommand(YHSLightCmd cmd){
        if (!cmd_thread_started_)
            StartCmdThread();

        std::lock_guard<std::mutex> guard(light_cmd_mutex_);
        current_light_cmd_ = cmd;
        light_ctrl_enabled_ = true;
        light_ctrl_requested_ = true;
    }

    void YHSBase::DisableLightCmdControl(){
        std::lock_guard<std::mutex> guard(light_cmd_mutex_);
        light_ctrl_enabled_ = false;
        light_ctrl_requested_ = true;
    }




    void YHSBase::ReceiveLoop(){
        struct timeval tv;
        fd_set fds;
        int maxfd = can_if_;
        while (ros::ok()) {
            FD_ZERO(&fds);
            FD_SET(can_if_, &fds);
            tv.tv_sec = 0;
            tv.tv_usec = 1000;
            int rv = select(maxfd + 1, &fds, NULL, NULL, &tv);
            if (rv == -1) {
                ROS_INFO("select() error, %s", strerror(errno));
                continue;
            } else if (rv > 0) {
                if (FD_ISSET(can_if_, &fds)) {
                    if (ParseCANFrame(can_if_, 1) == -1) {
                        continue;
                    }
                }
            }
        }
    }

    uint8_t YHSBase::ParseCANFrame(int fd,int idx){
        unsigned char pack_buffer[1024];
        struct can_frame can_msg = {0};
        int rv = read(fd, &can_msg, sizeof(can_msg));
        if (rv == -1) {
            ROS_INFO("read() can error, %s", strerror(errno));
            return -1;
        } else if (rv == sizeof(can_msg)) {
            can_frame  * rx_frame = & can_msg;
            //
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

                        uint8_t f = rx_frame->data[2] << 8;
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
                    case CAN_MSG_ODOM_FEEDBACK_ID    :
                    {
                        yhs_state_.odom_actived = rx_frame->data[0];
                        yhs_state_.car_mode = rx_frame->data[5];
                        break;
                    }
                    case CAN_MSG_LIGHT_FEEDBACK_ID   :
                    {
                        break;
                    }
                    case CAN_MSG_SPEAKER_FEEDBACK_ID :
                    {
                        break;
                    }
                    default:
                    {
                        break;
                    }
                }
            }
        }
        return 1;
    }

    void YHSBase::SendGearControlFrame(uint8_t count,YHSMotionCmd & cmd){
        can_frame gear_frame;
        gear_frame.can_id = CAN_MSG_GEAR_CONTROL_CMD_ID;
        gear_frame.can_dlc = 8;
        gear_frame.data[0] = 1;
        gear_frame.data[1] = cmd.gear;
        gear_frame.data[2] = 0;
        gear_frame.data[3] = 0;
        gear_frame.data[4] = 0;
        gear_frame.data[5] = 0;
        gear_frame.data[6] = getCANCount(count);
        gear_frame.data[7] = CalcYHSCANCheckXOR(gear_frame.can_id, gear_frame.data, gear_frame.can_dlc);
        if (write(can_if_, &gear_frame, sizeof(gear_frame)) == -1) {
            ROS_INFO("write canbus gear error, %s", strerror(errno));
        }
        std::cout << "can gear cmd: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(gear_frame.data[i]) << " ";
    }
    void YHSBase::SendSteerControlFrame(uint8_t count,YHSMotionCmd & cmd){
        can_frame steer_frame;
        steer_frame.can_id = CAN_MSG_STEER_CONTROL_CMD_ID;
        steer_frame.can_dlc = 8;

        uint16_t v = static_cast<uint16_t>((cmd.steer+90)/CAN_STEER_UNIT);

        steer_frame.data[0] = 1;
        steer_frame.data[1] = static_cast<uint8_t>(v & 0xFF);
        steer_frame.data[2] = static_cast<uint8_t>(v >> 8);
        steer_frame.data[3] = 0;
        steer_frame.data[4] = 0;
        steer_frame.data[5] = 0;
        steer_frame.data[6] = getCANCount(count);
        steer_frame.data[7] = CalcYHSCANCheckXOR(steer_frame.can_id, steer_frame.data, steer_frame.can_dlc);
        if (write(can_if_, &steer_frame, sizeof(steer_frame)) == -1) {
            ROS_INFO("write canbus steer error, %s", strerror(errno));
        }
        std::cout << "can steer cmd: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(steer_frame.data[i]) << " ";
    }
    void YHSBase::SendWheelControlFrame(uint8_t count,YHSMotionCmd & cmd){
        can_frame wheel_frame;
        wheel_frame.can_id = CAN_MSG_STEER_CONTROL_CMD_ID;
        wheel_frame.can_dlc = 8;
        int v = static_cast<uint8_t>(cmd.wheel/CAN_WHEEL_UNIT);
        wheel_frame.data[0] = 1;
        wheel_frame.data[1] = v;
        wheel_frame.data[2] = 0;
        wheel_frame.data[3] = 0;
        wheel_frame.data[4] = 0;
        wheel_frame.data[5] = 0;
        wheel_frame.data[6] = getCANCount(count);
        wheel_frame.data[7] = CalcYHSCANCheckXOR(wheel_frame.can_id, wheel_frame.data, wheel_frame.can_dlc);
        if (write(can_if_, &wheel_frame, sizeof(wheel_frame)) == -1) {
            ROS_INFO("write canbus wheel error, %s", strerror(errno));
        }
        std::cout << "can wheel cmd: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(wheel_frame.data[i]) << " ";
    }
    void YHSBase::SendBrakeControlFrame(uint8_t count,YHSMotionCmd & cmd){
        can_frame brake_frame;
        brake_frame.can_id = CAN_MSG_STEER_CONTROL_CMD_ID;
        brake_frame.can_dlc = 8;
        uint8_t v = static_cast<uint8_t>(cmd.brake/CAN_BRAKE_UNIT);
        brake_frame.data[0] = 1;
        brake_frame.data[1] = v;
        brake_frame.data[2] = 0;
        brake_frame.data[3] = 0;
        brake_frame.data[4] = 0;
        brake_frame.data[5] = 0;
        brake_frame.data[6] = getCANCount(count);
        brake_frame.data[7] = CalcYHSCANCheckXOR(brake_frame.can_id, brake_frame.data, brake_frame.can_dlc);
        if (write(can_if_, &brake_frame, sizeof(brake_frame)) == -1) {
            ROS_INFO("write canbus brake error, %s", strerror(errno));
        }
        std::cout << "can brake cmd: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(brake_frame.data[i]) << " ";
    }
    void YHSBase::SendParkingControlFrame(uint8_t count,YHSMotionCmd & cmd){
        can_frame parking_frame;
        parking_frame.can_id = CAN_MSG_STEER_CONTROL_CMD_ID;
        parking_frame.can_dlc = 8;
        parking_frame.data[0] = 1;
        parking_frame.data[1] = cmd.parking;
        parking_frame.data[2] = 0;
        parking_frame.data[3] = 0;
        parking_frame.data[4] = 0;
        parking_frame.data[5] = 0;
        parking_frame.data[6] = getCANCount(count);
        parking_frame.data[7] = CalcYHSCANCheckXOR(parking_frame.can_id, parking_frame.data, parking_frame.can_dlc);
        if (write(can_if_, &parking_frame, sizeof(parking_frame)) == -1) {
            ROS_INFO("write canbus parking error, %s", strerror(errno));
        }
        std::cout << "can parking cmd: ";
        for (int i = 0; i < 8; ++i)
            std::cout << static_cast<int>(parking_frame.data[i]) << " ";
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
