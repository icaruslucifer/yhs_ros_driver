/* 
 * YHS_messenger.cpp
 * 
 * Created on: Apr 26, 2019 22:14
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "yhs_messenger.hpp"

#include "yhs_msgs/YHSStatus.h"

namespace wescore
{
YHSROSMessenger::YHSROSMessenger(ros::NodeHandle *nh) : yhs_(nullptr), nh_(nh){

}

YHSROSMessenger::YHSROSMessenger(YHSBase *yhs, ros::NodeHandle *nh) : yhs_(yhs), nh_(nh){

}

void YHSROSMessenger::SetupSubscription()
{
    // odometry publisher
    odom_publisher_ = nh_->advertise<nav_msgs::Odometry>("odom", 50);
    status_publisher_ = nh_->advertise<yhs_msgs::YHSStatus>("/yhs_status", 10);

    // cmd subscriber
    motion_cmd_subscriber_ = nh_->subscribe<yhs_msgs::YHSMotionCmd>("/yhs_cmd_vel", 5, &YHSROSMessenger::MotionCmdCallback, this); //不启用平滑包则订阅“cmd_vel”
    light_cmd_subscriber_ = nh_->subscribe<yhs_msgs::YHSLightCmd>("/yhs_light_control", 5, &YHSROSMessenger::LightCmdCallback, this);
}

void YHSROSMessenger::MotionCmdCallback(const yhs_msgs::YHSMotionCmd::ConstPtr &msg){
    if (!simulated_robot_){
        YHSMotionCmd cmd;
        {
            std::lock_guard<std::mutex> guard(motioncmd_mutex_);
            cmd.steer = msg->steer_angle;
            cmd.wheel = msg->speed;
            cmd.gear = msg->gear;
        }
        yhs_->SetMotionCommand(cmd);
    }else{
        std::lock_guard<std::mutex> guard(motioncmd_mutex_);
        current_roscmd_ = *msg.get();
    }
}

void YHSROSMessenger::GetCurrentMotionCmdForSim(double &linear, double &angular){
    // std::lock_guard<std::mutex> guard(twist_mutex_);
    // linear = current_twist_.linear.x;
    // angular = current_twist_.angular.z;
}

void YHSROSMessenger::LightCmdCallback(const yhs_msgs::YHSLightCmd::ConstPtr &msg){
    if (!simulated_robot_){
        YHSLightCmd cmd;
        cmd.front_actived = msg->front_enable;
        cmd.left_actived = msg->left_enable;
        cmd.right_actived = msg->right_enable;
        yhs_->SetLightCommand(cmd);
    }else{
        std::cout << "simulated robot received light control cmd" << std::endl;
    }
}

void YHSROSMessenger::PublishStateToROS(){
    current_time_ = ros::Time::now();
    double dt = (current_time_ - last_time_).toSec();

    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    auto state = yhs_->GetYHSState();

    // publish YHS state message
    yhs_msgs::YHSStatus status_msg;

    status_msg.header.stamp = current_time_;

    status_msg.gear_actived = state.gear_actived;
    status_msg.gear_state = state.gear_value;

    status_msg.steer_actived = state.steer_actived;
    status_msg.steer_state = state.steer_value;
    status_msg.speed_actived = state.wheel_actived;
    status_msg.speed_state = state.wheel_value;


    status_msg.brake_actived = state.brake_actived;
    status_msg.brake_state = state.brake_value;
    status_msg.parking_actived = state.parking_actived;
    status_msg.parking_state = state.parking_value;

    status_msg.odom_actived = state.odom_actived;
    status_msg.car_state = state.car_mode;

    status_publisher_.publish(status_msg);

    // publish odometry and tf
    // PublishOdometryToROS(state.linear_velocity, state.angular_velocity, dt);

    // record time for next integration
    last_time_ = current_time_;
}


} // namespace wescore