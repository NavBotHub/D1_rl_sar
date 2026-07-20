#ifndef _IMU_DRIVER_H_
#define _IMU_DRIVER_H_
#include <iostream>

#if defined(USE_ROS1)
    // ROS1
    #include "ros/ros.h"
    #include <sensor_msgs/JointState.h>
    #include <sensor_msgs/Imu.h>
    #include <tf/transform_datatypes.h>
#elif defined(USE_ROS2)
    // ROS2
    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    #include <sensor_msgs/msg/imu.hpp>
    #include "geometry_msgs/msg/pose_stamped.hpp"
    #include "std_msgs/msg/float64_multi_array.hpp"
    #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif
#include <thread>
#include <initializer_list>
#include <fstream>
#include <array>
#include <serial/serial.h>

// #include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include "dm_imu/bsp_crc.h"

namespace dmbot_serial
{
#pragma pack(1)
  typedef struct
  {
    uint8_t FrameHeader1;
    uint8_t flag1;
    uint8_t slave_id1;
    uint8_t reg_acc;
    uint32_t accx_u32;
    uint32_t accy_u32;
    uint32_t accz_u32;
    uint16_t crc1;
    uint8_t FrameEnd1;

    uint8_t FrameHeader2;
    uint8_t flag2;
    uint8_t slave_id2;
    uint8_t reg_gyro;
    uint32_t gyrox_u32;
    uint32_t gyroy_u32;
    uint32_t gyroz_u32;
    uint16_t crc2;
    uint8_t FrameEnd2;

    uint8_t FrameHeader3;
    uint8_t flag3;
    uint8_t slave_id3;
    uint8_t reg_euler;//r-p-y
    uint32_t roll_u32;
    uint32_t pitch_u32;
    uint32_t yaw_u32;
    uint16_t crc3;
    uint8_t FrameEnd3;
  }IMU_Receive_Frame;
#pragma pack()

  typedef struct
  {
    float accx;
    float accy;
    float accz;
    float gyrox;
    float gyroy;
    float gyroz;
    float roll;
    float pitch;
    float yaw;
  }IMU_Data;

#if defined(USE_ROS1)
    // ROS1
  class DmImu
  {
    public:
        DmImu(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
        ~DmImu();
        void init_imu_serial();
        //Thread for reading serial IMU data
        void get_imu_data_thread();

        void enter_setting_mode();
        void turn_on_accel();
        void turn_on_gyro();
        void turn_on_euler();
        void turn_off_quat();
        void set_output_1000HZ();
        void save_imu_para();
        void exit_setting_mode();
        void restart_imu();

    private:
        int imu_seial_baud;
        std::string imu_serial_port;
        serial::Serial serial_imu; //Declare serial port object
        std::thread rec_thread;

        ros::Publisher imu_pose_pub_;
        ros::Publisher imu_pub_;
        ros::Publisher euler2_pub_;
        sensor_msgs::Imu imu_msgs;

        std::atomic<bool> stop_thread_{false} ;
        IMU_Receive_Frame receive_data{};//receive data frame
        IMU_Data data{};
  };
#elif defined(USE_ROS2)
  // ROS2
  class DmImu : public rclcpp::Node
  {
    public:

        DmImu();
        ~DmImu();
        void init_imu_serial();
        //Thread for reading serial IMU data
        void get_imu_data_thread();

        void enter_setting_mode();
        void turn_on_accel();
        void turn_on_gyro();
        void turn_on_euler();
        void turn_off_quat();
        void set_output_1000HZ();
        void save_imu_para();
        void exit_setting_mode();
        void restart_imu();

    private:
        int imu_seial_baud;
        std::string imu_serial_port;

        serial::Serial serial_imu; //Declare serial port object
        std::thread rec_thread;

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr euler2_pub_;

        sensor_msgs::msg::Imu imu_msgs;

        std::atomic<bool> stop_thread_{false} ;
        IMU_Receive_Frame receive_data{};//receive data frame
        IMU_Data data{};
  };
#endif
}
#endif


