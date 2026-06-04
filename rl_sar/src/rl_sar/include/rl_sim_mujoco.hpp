/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RL_SIM_HPP
#define RL_SIM_HPP

// #define PLOT
// #define CSV_LOGGER

#include "rl_sdk.hpp"
#include "observation_buffer.hpp"
#include "inference_runtime.hpp"
#include "loop.hpp"
#include "fsm_all.hpp"
#include "dog_usb_receiver.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <vector>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include <sys/wait.h>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <memory>

#include <mujoco/mujoco.h>
#include "joystick.hh"
#include "mujoco_utils.hpp"

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

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

class RL_Sim : public RL
{
public:
    RL_Sim(int argc, char **argv);
    ~RL_Sim();

    std::unique_ptr<mj::Simulate> sim;
    static RL_Sim* instance;

private:
    // rl functions
    std::vector<float> Forward() override;
    void GetState(RobotState<float> *state) override;
    void SetCommand(const RobotCommand<float> *command) override;
    void RunModel();
    void RobotControl();
    void GetKeyboard();
    void ParseDogUsbArgs(int argc, char **argv);
    void ApplyDogUsbControl(bool emit_events);
    bool DogUsbShouldBlockFallback() const;
    void TriggerDogUsbGamepad(Input::Gamepad gamepad, const char *name);
    void LogDogUsbStatus(bool safe_state, bool usb_timeout, bool remote_timeout, bool serial_connected);
    void UpdateCsvAutoRecord(double t);
    void LoadReplayQdes();
    void ApplyReplayQdes(double t);
    float CommandQWithDeployOffset(const RobotCommand<float> *command, int index) const;
    int JointQposIdFromActuator(int actuator_id) const;
    int JointDofIdFromActuator(int actuator_id) const;
    double JointQ(int actuator_id) const;
    double JointDq(int actuator_id) const;
    float TorqueLimit(int index) const;

    // loop
    std::shared_ptr<LoopFunc> loop_keyboard;
    std::shared_ptr<LoopFunc> loop_joystick;
    std::shared_ptr<LoopFunc> loop_control;
    std::shared_ptr<LoopFunc> loop_rl;
    std::shared_ptr<LoopFunc> loop_plot;

    // plot
    const int plot_size = 100;
    std::vector<int> plot_t;
    std::vector<std::vector<float>> plot_real_joint_pos, plot_target_joint_pos;
    void Plot();

    // mujoco
    mjData *mj_data;
    mjModel *mj_model;
    std::string scene_name;
    mjData *mj_diag_data_ = nullptr;
    bool mujoco_headless_ = false;

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

    // dog USB DOG_CTRL input
    std::unique_ptr<DogUsbReceiver> dog_usb_receiver;
    bool dog_usb_enable = false;
    std::string dog_usb_device;
    int dog_usb_baud = 115200;
    int dog_usb_timeout_ms = 300;
    int dog_remote_timeout_ms = 1500;
    bool dog_usb_allow_fallback = false;
    bool dog_usb_has_taken_control = false;
    uint16_t dog_usb_prev_buttons = 0;
    bool dog_usb_status_initialized = false;
    bool dog_usb_last_safe_state = false;
    bool dog_usb_last_usb_timeout = false;
    bool dog_usb_last_remote_timeout = false;
    bool dog_usb_last_serial_connected = true;

    // others
    std::string gazebo_model_name;
    std::map<std::string, float> joint_positions;
    std::map<std::string, float> joint_velocities;
    std::map<std::string, float> joint_efforts;
    void StartJointController(const std::string& ros_namespace, const std::vector<std::string>& names);

    //CSV
    std::ofstream csv_ofs;
    bool csv_header_written = false;
    std::chrono::steady_clock::time_point csv_start_time;
    std::string csv_output_path = "mujoco_joint_state.csv";
    bool csv_auto_record = false;
    double csv_auto_duration = 104.0;
    bool csv_auto_getup_sent = false;
    bool csv_auto_locomotion_sent = false;
    bool csv_auto_exit_sent = false;
    std::string csv_segment_name = "manual";
    std::array<float, 12> deploy_qdes_offset_ = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
    bool deploy_qdes_offset_enabled_ = false;
    bool pd_every_mj_step_ = false;
    bool replay_disable_contact_ = false;
    uint64_t robot_control_count_ = 0;
    uint64_t set_command_count_ = 0;
    uint64_t run_model_count_ = 0;
    uint64_t mj_step_count_ = 0;

    // Foot order matches policy order: FL, FR, RL, RR.
    std::array<int, 4> foot_geom_ids_ = {-1, -1, -1, -1};
    std::array<float, 4> foot_air_time_ = {0.f, 0.f, 0.f, 0.f};
    std::array<std::array<float, 3>, 4> prev_foot_pos_ = {{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}}};
    bool foot_prev_valid_ = false;

    // Open-loop q_des replay diagnostics. q_des rows are 12 comma-separated joint targets.
    bool replay_qdes_enabled_ = false;
    bool replay_fixed_base_ = false;
    bool replay_headless_ = false;
    bool replay_started_ = false;
    double replay_start_t_ = 0.0;
    double replay_root_height_ = 0.65;
    std::string replay_qdes_path_;
    std::vector<std::array<float, 12>> replay_qdes_;
};

#endif // RL_SIM_HPP
