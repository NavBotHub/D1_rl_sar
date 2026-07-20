/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Real robot <-> MuJoCo mapping validation tool.
 *
 * Purpose: only validate whether IMU direction, motor direction, and joint order match the model.
 *      No RL inference, no FSM, and no physics simulation; it only draws real robot data in MuJoCo.
 *
 * Workflow:
 *   1. Open can1/can2 and automatically enable 12 motors; in each cycle send
 *      MIT commands with kp=kd=tau=0, keeping motors passive and hand-movable while still returning q/dq/tau.
 *   2. Subscribe to the /imu/data topic published by the dm_imu node.
 *   3. Start the MuJoCo simulation window with sim->run = 0 (forward kinematics only, no integration).
 *   4. At a fixed period, write the IMU quat to qpos[3..6] and the 12 motor positions to
 *      qpos[7+joint_mapping[i]], then call mj_forward() to refresh rendering.
 *
 * Validation method:
 *   - IMU: lift the base and rotate it around roll/pitch/yaw, then observe whether the MuJoCo base
 *     rotates in the same direction and by the same amount.
 *   - Motors: hand-rotate one joint on a leg and check whether the corresponding MuJoCo joint moves in the same direction and by the same amount.
 *     If it is reversed, modify the corresponding sign in directionMotor_Front / directionMotor_Back.
 *
 */
#ifndef RL_REAL2MUJOCO_HPP
#define RL_REAL2MUJOCO_HPP

#include "rl_sdk.hpp"
#include "loop.hpp"
#include "logger.hpp"

#include <dmbot_serial/protocol/damiao.h>

#include <mujoco/mujoco.h>
#include "mujoco_utils.hpp"

#include <array>
#include <atomic>
#include <csignal>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class RL_Real2Mujoco
{
public:
    explicit RL_Real2Mujoco(const std::vector<std::string> &args);
    ~RL_Real2Mujoco();

    std::shared_ptr<rclcpp::Node> ros2_node;
    std::unique_ptr<mj::Simulate> sim;

    static RL_Real2Mujoco *instance;

private:
    void SyncLoop();
    void PrintLoop();
    void SendPassiveCommand();
    void ReadMotorState(std::vector<float> &q,
                        std::vector<float> &dq,
                        std::vector<float> &tau) const;

    // ---- MuJoCo ----
    mjModel *mj_model = nullptr;
    mjData *mj_data = nullptr;
    std::string robot_name;
    std::string scene_name;

    // free joint qpos[0..2] (base position) is fixed in mapping mode so the model does not fly away.
    std::array<double, 3> base_pos_init{0.0, 0.0, 0.4};

    // ---- Mapping parameters read from base.yaml ----
    YamlParams params;
    int num_of_dofs = 12;
    std::vector<int> joint_mapping;
    std::vector<std::string> joint_names;

    // ---- Damiao CAN motors (kept consistent with rl_real_d1)----
    std::shared_ptr<damiao::Motor_Control> motorsInterface;
    std::shared_ptr<damiao::Motor_Control> motorsInterface2;
    std::vector<damiao::DmActData> CAN1;
    std::vector<damiao::DmActData> CAN2;

    // Mapping between CAN id and local 6-tuple index, kept consistent with rl_real_d1.cpp.
    const std::vector<int> CAN1_MAP{1, 2, 3, 4, 5, 6};
    const std::vector<int> CAN2_MAP{1, 2, 3, 4, 5, 6};

    // Motor direction correction: after multiplication, joint values are in the model coordinate system.
    // If an axis direction is reversed, change the corresponding sign.
    std::vector<int> directionMotor_Front{1, 1, 1, 1, -1, -1};
    std::vector<int> directionMotor_Back{-1, 1, 1, -1, -1, -1};

    // ---- IMU (subscribes to /imu/data)----
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    sensor_msgs::msg::Imu imu_msg_;
    std::atomic<bool> imu_received_{false};
    void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    // Publish joint_states for other tools (rviz / plotting), using mapped joint values.
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // ---- Loops ----
    std::shared_ptr<LoopFunc> loop_sync;
    std::shared_ptr<LoopFunc> loop_print;

    // ROS2 needs an independent spin thread; otherwise MuJoCo RenderLoop occupies the main thread,
    // and the IMU callback is never scheduled.
    std::thread ros_spin_thread_;
    std::atomic<bool> ros_spin_running_{false};

    // MuJoCo physics thread. It must be joined, not detached; otherwise it may still access
    // sim / m / d during destruction, causing a UAF segfault (exit code = -11/SIGSEGV).
    std::thread physics_thread_;

    // ---- Config switches ----
    bool send_passive_cmd = true;   // Whether to send passive kp=kd=tau=0 commands (so motors can be hand-rotated)
    bool freeze_base_pos = true;    // Whether to freeze base position (observe orientation only, not translation)
    bool print_debug = true;        // Whether to periodically print IMU / joint readings
    float sync_period = 0.005f;     // Matches the real robot dt = 200 Hz
    float print_period = 0.2f;      // 5 Hz printing
};

#endif // RL_REAL2MUJOCO_HPP
