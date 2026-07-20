#include <dm_imu/imu_driver.h>
#include <iostream>
#include <thread>
#include <condition_variable>

#if defined(USE_ROS1)
#include "ros/ros.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "dm_imu_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Rate r(1000);
    std::shared_ptr<dmbot_serial::DmImu> imuInterface;
    imuInterface=std::make_shared<dmbot_serial::DmImu>(nh, nh_private);
    while (ros::ok()) 
    {   
      r.sleep();
    }
    return 0;
}
#elif defined(USE_ROS2)
#include "rclcpp/rclcpp.hpp"
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialize ROS2
    auto node = std::make_shared<dmbot_serial::DmImu>();  // Create DmImu instance
    rclcpp::spin(node);  // Start the ROS2 node processing loop
    rclcpp::shutdown();  // Shut down ROS2
}
#endif
