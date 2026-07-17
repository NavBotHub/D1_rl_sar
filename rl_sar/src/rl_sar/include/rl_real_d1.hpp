/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_REAL_DM_HPP
#define RL_REAL_DM_HPP

// #define PLOT
// #define CSV_LOGGER
// #define USE_ROS

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_d1.hpp"
#include <dmbot_serial/protocol/damiao.h>
#include "joystick.hh"
#include "dog_usb_receiver.hpp"

#include <chrono>
#include <cstdint>
#include <csignal>
#include <memory>
#include <string>
#include <vector>

#if defined(USE_ROS1)
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#elif defined(USE_ROS2)
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#endif

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
struct DmMotorData
{
  double pos_, vel_, tau_;
  double pos_des_, vel_des_, kp_, kd_, ff_;
};
struct DmImuData
{
  double ori[4];
  double ori_cov[9];
  double angular_vel[3];
  double angular_vel_cov[9];
  double linear_acc[3];
  double linear_acc_cov[9];
};
class Button
{
public:
    Button() {}

    void update(bool state)
    {
        on_press = state ? state != pressed : false;
        on_release = state ? false : state != pressed;
        pressed = state;
    }

    bool pressed = false;
    bool on_press = false;
    bool on_release = false;
};
class RL_Real : public RL
{
public:
    RL_Real(int argc, char **argv);
    ~RL_Real();

#if defined(USE_ROS2)
    std::shared_ptr<rclcpp::Node> ros2_node;
#endif

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    bool setupImu();
    void ApplyDogUsbControl(bool emit_events);
    bool DogUsbShouldBlockFallback() const;
    void TriggerDogUsbGamepad(Input::Gamepad gamepad, const char* name);
    void RequestDogUsbExit(const char* reason);
    void HoldDogUsbExitInputs();
    void UpdateDogUsbExitShutdown();
    void ShutdownDogUsbExitIfReady();
    bool DogUsbExitTimedOut() const;
    std::string CurrentFsmStateName() const;
    void LogDogUsbStatus(bool safe_state, bool usb_timeout, bool remote_timeout, bool serial_connected);
    std::vector<float> SmoothCommands(const std::vector<float>& target_commands, float dt);
    void ResetCommandSmoothing() override;
    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();
    
    //damiao motor
    std::shared_ptr<damiao::Motor_Control> motorsInterface;
    std::vector<damiao::DmActData> CAN1;
    std::shared_ptr<damiao::Motor_Control> motorsInterface2;
    std::vector<damiao::DmActData> CAN2;

    std::vector<int> joint_mapping;
    std::vector<std::string> joint_names_;//Used for topic publishing
    DmMotorData dmSendcmd_[7];
    DmMotorData dmSendcmd2_[7];
    DmMotorData rl_date[12];
    const std::vector<int> CAN1_MAP {1, 2, 3, 4, 5, 6};
    const std::vector<int> CAN2_MAP {1, 2, 3, 4, 5, 6};
    const std::vector<int> directionMotor_Front{1, 1, 1,  1, -1, -1};//123 456
    const std::vector<int> directionMotor_Back{-1, 1, 1,  -1, -1, -1};//123 456
    //imu
    DmImuData imuData_{};
    #if defined(USE_ROS1)
    ros::Subscriber odom_sub_;
    sensor_msgs::Imu yesenceIMU_;
    void OdomCallBack(const sensor_msgs::Imu::ConstPtr &odom) {
        yesenceIMU_ = *odom;
    }
    #elif defined(USE_ROS2)
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr odom_sub_;
    sensor_msgs::msg::Imu yesenceIMU_;
    void OdomCallBack(const sensor_msgs::msg::Imu::ConstSharedPtr &odom) {
        yesenceIMU_ = *odom;
    }
    #endif
    // joystick
    std::unique_ptr<Joystick> sys_js;
    JoystickEvent sys_js_event;

    Button sys_js_button[20];
    int sys_js_axis[10] = {0};
    bool sys_js_active = false;
    float axis_deadzone = 0.05f;
    int sys_js_max_value = (1 << (16 - 1));
    void SetupSysJoystick(const std::string& device, int bits);
    void GetSysJoystick();
    bool keyboard_enable = true;
    std::string sys_joystick_device = "/dev/input/js0";

    // dog USB DOG_CTRL input
    std::unique_ptr<DogUsbReceiver> dog_usb_receiver;
    bool dog_usb_enable = false;
    std::string dog_usb_device;
    int dog_usb_baud = 115200;
    int dog_usb_timeout_ms = 300;
    int dog_remote_timeout_ms = 1500;
    bool dog_usb_allow_fallback = false;
    bool dog_usb_l1_off_exit = false;
    int dog_usb_l1_button_bit = 8;
    int dog_usb_l1_exit_timeout_ms = 8000;
    bool dog_usb_l1_seen_on = false;
    bool dog_usb_exit_requested = false;
    bool dog_usb_exit_shutdown_after_command = false;
    std::chrono::steady_clock::time_point dog_usb_exit_started_at{};
    bool dog_usb_has_taken_control = false;
    uint16_t dog_usb_prev_buttons = 0;
    bool dog_usb_status_initialized = false;
    bool dog_usb_last_safe_state = false;
    bool dog_usb_last_usb_timeout = false;
    bool dog_usb_last_remote_timeout = false;
    bool dog_usb_last_serial_connected = true;

    std::vector<float> smoothed_commands_{0.0f, 0.0f, 0.0f};
    std::vector<float> raw_command_inputs_{0.0f, 0.0f, 0.0f};
    std::string command_input_source_{"control"};
    bool command_smoothing_initialized_ = false;
    bool command_smoothing_reset_requested_ = true;

#if defined(USE_ROS1)
    geometry_msgs::Twist cmd_vel;
    ros::Subscriber cmd_vel_subscriber;
    ros::Publisher joint_state_pub_;
    sensor_msgs::JointState joint_state_msg_;
    void CmdvelCallback(const geometry_msgs::Twist::ConstPtr &msg);
#elif defined(USE_ROS2)
    geometry_msgs::msg::Twist cmd_vel;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    sensor_msgs::msg::JointState joint_state_msg_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber;
    void CmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
#endif
};

#endif // RL_REAL_D1_HPP
