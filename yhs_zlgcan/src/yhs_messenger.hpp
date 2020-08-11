/* 
 * yhs_messenger.hpp
 * 
 * Created on: Jun 14, 2019 10:24
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef YHS_MESSENGER_HPP
#define YHS_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "yhs_msgs/YHSLightCmd.h"
#include "yhs_msgs/YHSMotionCmd.h"
#include "yhs_base.hpp"

namespace wescore
{
class YHSROSMessenger
{
public:
    explicit YHSROSMessenger(ros::NodeHandle *nh);
    YHSROSMessenger(YHSBase *yhs, ros::NodeHandle *nh);

    std::string odom_frame_;
    std::string base_frame_;

    bool simulated_robot_ = false;
    int sim_control_rate_ = 50;

    void SetupSubscription();

    void PublishStateToROS();

private:
    YHSBase *yhs_;
    ros::NodeHandle *nh_;

    std::mutex motioncmd_mutex_;
    yhs_msgs::YHSMotionCmd current_roscmd_;

    ros::Publisher odom_publisher_;
    ros::Publisher status_publisher_;
    ros::Subscriber motion_cmd_subscriber_;
    ros::Subscriber light_cmd_subscriber_;

    // speed variables
    double linear_speed_ = 0.0;
    double angular_speed_ = 0.0;
    double position_x_ = 0.0;
    double position_y_ = 0.0;
    double theta_ = 0.0;

    ros::Time last_time_;
    ros::Time current_time_;

    void MotionCmdCallback(const yhs_msgs::YHSMotionCmd::ConstPtr &msg);
    void LightCmdCallback(const yhs_msgs::YHSLightCmd::ConstPtr &msg);
};
} // namespace wescore

#endif /* yhs_MESSENGER_HPP */
