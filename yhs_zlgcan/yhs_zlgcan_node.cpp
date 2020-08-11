
#include <string>

#include <ros/ros.h>

#include "src/yhs_base.hpp"
#include "src/yhs_messenger.hpp"


#include <thread>
using namespace wescore;

int main(int argc, char **argv){
    // setup ROS node
    ros::init(argc, argv, "yhs_zlgcan");
    ros::NodeHandle node(""), private_node("~");

    // instantiate a robot object
    YHSBase robot;
    YHSROSMessenger messenger(&robot, &node);

    // fetch parameters before connecting to robot
    std::string can_name;
    private_node.param<std::string>("can_name", can_name, std::string("can0"));
    std::string baudrate;
    private_node.param<std::string>("baudrate", baudrate, std::string("500k"));


    private_node.param<std::string>("odom_frame", messenger.odom_frame_, std::string("odom"));
    private_node.param<std::string>("base_frame", messenger.base_frame_, std::string("base_link"));

    robot.Connect(can_name,baudrate);
    ROS_INFO("Using ZLG CAN  to talk with the robot");


    messenger.SetupSubscription();

    ros::Rate rate_20hz(20); // 20Hz
    while (ros::ok()){
        messenger.PublishStateToROS();
        ros::spinOnce();
        rate_20hz.sleep();
    }
    return 0;
}