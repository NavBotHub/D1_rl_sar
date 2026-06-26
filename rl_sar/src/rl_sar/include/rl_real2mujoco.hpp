/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * 真机 <-> MuJoCo 映射校验工具。
 *
 * 目的：只用来验证 IMU 方向 / 电机方向 / 关节顺序是否和模型一致。
 *      不做 RL 推理，不做 FSM，不做物理仿真，只把真机数据画到 MuJoCo。
 *
 * 工作流程：
 *   1. 打开 can1/can2，自动使能 12 个电机；在每个周期下发 kp=kd=tau=0 的
 *      MIT 命令，使电机保持在被动（可用手转动）状态，同时仍然回传 q/dq/tau。
 *   2. 订阅 /imu/data 话题（由 dm_imu 节点发布）。
 *   3. 启动 MuJoCo 仿真窗口，但 sim->run = 0（仅前向运动学，不积分）。
 *   4. 按固定周期把 IMU quat 写入 qpos[3..6]，把 12 个电机位置写入
 *      qpos[7+joint_mapping[i]]，然后 mj_forward() 刷新渲染。
 *
 * 校验方法：
 *   - IMU：抬起机身做 roll/pitch/yaw 三个方向的旋转，观察 MuJoCo 里的机身
 *     是否以同样方向同样幅度旋转。
 *   - 电机：用手转动某条腿的某个关节，观察 MuJoCo 里对应关节是否同向同幅
 *     运动；若相反，修改 directionMotor_Front / directionMotor_Back 中对
 *     应位的符号。
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

    // free joint qpos[0..2]（机身位置）在映射模式下固定，不让模型飞起来。
    std::array<double, 3> base_pos_init{0.0, 0.0, 0.4};

    // ---- 从 base.yaml 读取的映射参数 ----
    YamlParams params;
    int num_of_dofs = 12;
    std::vector<int> joint_mapping;
    std::vector<std::string> joint_names;

    // ---- 达妙 CAN 电机（与 rl_real_d1 保持一致）----
    std::shared_ptr<damiao::Motor_Control> motorsInterface;
    std::shared_ptr<damiao::Motor_Control> motorsInterface2;
    std::vector<damiao::DmActData> CAN1;
    std::vector<damiao::DmActData> CAN2;

    // CAN id 与本地 6 元组下标的映射，和 rl_real_d1.cpp 保持一致。
    const std::vector<int> CAN1_MAP{1, 2, 3, 4, 5, 6};
    const std::vector<int> CAN2_MAP{1, 2, 3, 4, 5, 6};

    // 电机方向修正：乘完之后得到模型坐标系下的关节量。
    // 若某轴方向反了，改对应位符号即可。
    std::vector<int> directionMotor_Front{1, 1, 1, 1, -1, -1};
    std::vector<int> directionMotor_Back{-1, 1, 1, -1, -1, -1};

    // ---- IMU（订阅 /imu/data）----
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    sensor_msgs::msg::Imu imu_msg_;
    std::atomic<bool> imu_received_{false};
    void ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg);

    // 发布供其他工具（rviz / 绘图）使用的 joint_states（映射后的关节量）。
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

    // ---- 循环 ----
    std::shared_ptr<LoopFunc> loop_sync;
    std::shared_ptr<LoopFunc> loop_print;

    // ROS2 需要独立线程 spin，否则 MuJoCo 的 RenderLoop 会把主线程吃光，
    // IMU 回调永远不会被调度。
    std::thread ros_spin_thread_;
    std::atomic<bool> ros_spin_running_{false};

    // MuJoCo 物理线程。**必须 join**（不 detach），否则析构时它还会访问
    // sim / m / d，造成 UAF 段错误（退出时 exit code = -11/SIGSEGV）。
    std::thread physics_thread_;

    // ---- 配置开关 ----
    bool send_passive_cmd = true;   // 是否下发 kp=kd=tau=0 的被动命令（让电机能被手转动）
    bool freeze_base_pos = true;    // 是否冻结机身位置（只看姿态不看平移）
    bool print_debug = true;        // 是否周期打印 IMU / 关节读数
    float sync_period = 0.005f;     // 与真机 dt 一致 = 200 Hz
    float print_period = 0.2f;      // 5 Hz 打印
};

#endif // RL_REAL2MUJOCO_HPP
