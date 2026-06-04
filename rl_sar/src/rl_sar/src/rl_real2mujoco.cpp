/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * 真机 <-> MuJoCo 映射校验工具。详见 rl_real2mujoco.hpp 注释。
 */
#include "rl_real2mujoco.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <thread>

RL_Real2Mujoco *RL_Real2Mujoco::instance = nullptr;

// ------------------------------------------------------------------
// 构造：打开 CAN + 启动 MuJoCo + ROS2 订阅 IMU + 启动同步循环
// ------------------------------------------------------------------
RL_Real2Mujoco::RL_Real2Mujoco(const std::vector<std::string> &args)
{
    instance = this;

    // 命令行：rl_real2mujoco <robot_name> <scene_name>
    // 默认跑 d1 + scene。args[0] 是可执行文件名。
    this->robot_name = (args.size() >= 2) ? args[1] : std::string("d1");
    this->scene_name = (args.size() >= 3) ? args[2] : std::string("scene");

    // -------- 1. 读取 base.yaml --------
    // 和 rl_real_d1 一样走 POLICY_DIR/<robot>/base.yaml，保证关节顺序 /
    // joint_mapping / num_of_dofs 和真机节点一致。
    std::string config_path = std::string(POLICY_DIR) + "/" + this->robot_name + "/base.yaml";
    try
    {
        YAML::Node cfg = YAML::LoadFile(config_path)[this->robot_name];
        for (auto it = cfg.begin(); it != cfg.end(); ++it)
        {
            std::string key = it->first.as<std::string>();
            this->params.config_node[key] = it->second;
        }
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::ERROR << "[RL_Real2Mujoco] Failed to load " << config_path
                  << " : " << e.what() << std::endl;
        throw;
    }

    this->num_of_dofs = this->params.Get<int>("num_of_dofs", 12);
    this->joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    this->joint_names = this->params.Get<std::vector<std::string>>("joint_names");

    if (static_cast<int>(this->joint_mapping.size()) != this->num_of_dofs)
    {
        // yaml 里没写或维度不对：退化成 0..N-1
        this->joint_mapping.resize(this->num_of_dofs);
        for (int i = 0; i < this->num_of_dofs; ++i) this->joint_mapping[i] = i;
        std::cout << LOGGER::WARNING
                  << "[RL_Real2Mujoco] joint_mapping missing or wrong size, fallback to identity"
                  << std::endl;
    }

    // -------- 2. 启动 MuJoCo --------
    std::cout << LOGGER::INFO << "[RL_Real2Mujoco][MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }
    scanPluginLibraries();

    mjvCamera cam;  mjv_defaultCamera(&cam);
    mjvOption opt;  mjv_defaultOption(&opt);
    mjvPerturb pert; mjv_defaultPerturb(&pert);
    this->sim = std::make_unique<mj::Simulate>(
        std::make_unique<mj::GlfwAdapter>(),
        &cam, &opt, &pert, /* is_passive = */ false);

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) +
                           "/../rl_sar_zoo/" + this->robot_name +
                           "_description/mjcf/" + this->scene_name + ".xml";
    std::cout << LOGGER::INFO << "[RL_Real2Mujoco][MuJoCo] Loading " << filename << std::endl;

    // 不要 detach！否则 PhysicsThread 可能在析构流程中继续访问 sim/m/d，
    // 造成 UAF 段错误。我们在 RenderLoop 退出后显式 join。
    this->physics_thread_ = std::thread(&PhysicsThread, sim.get(), filename.c_str());

    // 等到 physicsthread 把 m/d 装载好
    while (1)
    {
        if (d)
        {
            std::cout << LOGGER::INFO << "[RL_Real2Mujoco][MuJoCo] Data prepared" << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    this->mj_model = m;
    this->mj_data  = d;

    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
        if (this->mj_model->nkey > 0)
        {
            mj_resetDataKeyframe(this->mj_model, this->mj_data, 0);
        }
        // 记录 keyframe 给出的初始 base 位置，后续冻结用
        this->base_pos_init[0] = this->mj_data->qpos[0];
        this->base_pos_init[1] = this->mj_data->qpos[1];
        this->base_pos_init[2] = this->mj_data->qpos[2];
        mj_forward(this->mj_model, this->mj_data);
    }

    // 最关键：停掉物理仿真，只靠我们每个周期改 qpos + mj_forward。
    // 如果不停，物理积分会把机身往下砸，覆盖掉我们写的 qpos。
    this->sim->run = 0;

    // -------- 3. 打开真机 CAN（12 个电机） --------
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x01, .mst_id = 0x11});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x02, .mst_id = 0x12});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x03, .mst_id = 0x13});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x04, .mst_id = 0x14});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x05, .mst_id = 0x15});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x06, .mst_id = 0x16});

    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x01, .mst_id = 0x11});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x02, .mst_id = 0x12});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x03, .mst_id = 0x13});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x04, .mst_id = 0x14});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x05, .mst_id = 0x15});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P, .mode = damiao::MIT_MODE, .can_id = 0x06, .mst_id = 0x16});

    try
    {
        this->motorsInterface  = std::make_shared<damiao::Motor_Control>("can1", &this->CAN1, damiao::canfd);
        this->motorsInterface2 = std::make_shared<damiao::Motor_Control>("can2", &this->CAN2, damiao::canfd);
    }
    catch (const std::exception &e)
    {
        std::cout << LOGGER::ERROR << "[RL_Real2Mujoco] Open CAN failed: " << e.what() << std::endl;
        throw;
    }

    // -------- 4. ROS2 订阅 /imu/data + 发布 /joint_states --------
    this->ros2_node = std::make_shared<rclcpp::Node>("rl_real2mujoco");
    // 和 rl_real_d1 一样用默认 reliable QoS；dm_imu_node 也是用默认 10
    // 深度 publish，若用 SensorDataQoS(best-effort) 会 QoS 不兼容收不到。
    this->imu_sub_ = this->ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&RL_Real2Mujoco::ImuCallback, this, std::placeholders::_1));
    this->joint_state_pub_ = this->ros2_node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    // MuJoCo RenderLoop 会独占主线程，所以必须把 ROS2 的 spin 放到独立线程。
    //
    // 这个线程同时还负责"Ctrl+C 退出桥接"：rclcpp 的默认 SIGINT handler 会
    // 把 rclcpp::ok() 置为 false，但不会让 MuJoCo 的 RenderLoop 退出。
    // 我们在这里检测到 rclcpp shutdown 后，主动把 sim->exitrequest 设 1，
    // MuJoCo 下一帧就会退出 RenderLoop，主线程才能走到析构、失能电机。
    this->ros_spin_running_.store(true);
    this->ros_spin_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(this->ros2_node);
        while (this->ros_spin_running_.load() && rclcpp::ok())
        {
            exec.spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // 到这里要么是析构主动停了我们（ros_spin_running_=false），
        // 要么是 Ctrl+C 让 rclcpp shutdown。后者必须通知 MuJoCo 退出。
        if (this->sim)
        {
            this->sim->exitrequest.store(1);
        }
    });

    // -------- 5. 启动循环 --------
    this->loop_sync = std::make_shared<LoopFunc>(
        "loop_sync", this->sync_period,
        std::bind(&RL_Real2Mujoco::SyncLoop, this));
    this->loop_sync->start();

    if (this->print_debug)
    {
        this->loop_print = std::make_shared<LoopFunc>(
            "loop_print", this->print_period,
            std::bind(&RL_Real2Mujoco::PrintLoop, this));
        this->loop_print->start();
    }

    std::cout << LOGGER::INFO << "[RL_Real2Mujoco] Ready. "
              << "用手转动真机关节，MuJoCo 中对应关节应同步；"
              << "旋转机身，MuJoCo 中 base 应同步。"
              << std::endl;

    // 进入 MuJoCo 渲染循环（阻塞）
    sim->RenderLoop();

    // -------- RenderLoop 返回后的有序清理 --------
    // 这里的顺序很重要，任何一步打乱都可能 SIGSEGV：
    // 1) 先停 SyncLoop/PrintLoop，它们还会读 mj_data。
    if (this->loop_print) { this->loop_print->shutdown(); this->loop_print.reset(); }
    if (this->loop_sync)  { this->loop_sync->shutdown();  this->loop_sync.reset(); }

    // 2) 停 ROS2 spin 线程，避免它继续 spin_some 访问正在被拆解的上下文。
    this->ros_spin_running_.store(false);
    if (this->ros_spin_thread_.joinable())
    {
        this->ros_spin_thread_.join();
    }

    // 3) 等 PhysicsThread 自然结束（它会看到 exitrequest != 0 退出循环并
    //    释放 m/d）。必须 join 到它结束，否则下面 mj_data=nullptr、sim.reset
    //    之后它还在跑就会访问悬空指针。
    if (this->physics_thread_.joinable())
    {
        this->physics_thread_.join();
    }

    // 4) 现在 PhysicsThread 已结束，m/d 已被 mj_delete*，不能再访问。
    this->mj_model = nullptr;
    this->mj_data  = nullptr;
}

RL_Real2Mujoco::~RL_Real2Mujoco()
{
    instance = nullptr;

    // 构造函数末尾可能已经 shutdown 过，这里做一次幂等收尾。
    if (this->loop_print) { this->loop_print->shutdown(); this->loop_print.reset(); }
    if (this->loop_sync)  { this->loop_sync->shutdown();  this->loop_sync.reset(); }

    this->ros_spin_running_.store(false);
    if (this->ros_spin_thread_.joinable())
    {
        this->ros_spin_thread_.join();
    }

    // PhysicsThread 在 RenderLoop 退出后已经 join 过了；这里做兜底，
    // 万一构造函数提前抛异常，保证进程退出时 PhysicsThread 不再运行，
    // 避免 unique_ptr<Simulate>/m/d 被销毁时线程仍在访问。
    if (this->physics_thread_.joinable())
    {
        // 先强制把 exitrequest 设上，免得卡在 PhysicsLoop 里
        if (this->sim) this->sim->exitrequest.store(1);
        this->physics_thread_.join();
    }

    // 主动失能电机：依赖 shared_ptr 自动析构也能走到 disable_all()，
    // 但这里再显式调一次作为保险（比如有人 hold 了 shared_ptr 副本、
    // 或者 Motor_Control 析构顺序异常时，至少电机已经断力矩）。
    try
    {
        if (this->motorsInterface)  this->motorsInterface->disable_all();
        if (this->motorsInterface2) this->motorsInterface2->disable_all();
    }
    catch (...) {}

    std::cout << LOGGER::INFO << "[RL_Real2Mujoco] motors disabled, exit" << std::endl;
}

// ------------------------------------------------------------------
// IMU 回调：只做缓存，真正采样在 SyncLoop 里做
// ------------------------------------------------------------------
void RL_Real2Mujoco::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    this->imu_msg_ = *msg;
    this->imu_received_.store(true);
}

// ------------------------------------------------------------------
// 读取 12 个电机的位置 / 速度 / 力矩（含方向修正）
// 关节顺序：FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
//           RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf
// ------------------------------------------------------------------
void RL_Real2Mujoco::ReadMotorState(std::vector<float> &q,
                                    std::vector<float> &dq,
                                    std::vector<float> &tau) const
{
    q.assign(this->num_of_dofs, 0.0f);
    dq.assign(this->num_of_dofs, 0.0f);
    tau.assign(this->num_of_dofs, 0.0f);

    for (int i = 0; i < 6; ++i)
    {
        int read_id = this->CAN1_MAP[i];
        auto mot = this->motorsInterface->getMotor(read_id);
        if (!mot) continue;
        q[i]   = mot->Get_Position() * this->directionMotor_Front[i];
        dq[i]  = mot->Get_Velocity() * this->directionMotor_Front[i];
        tau[i] = mot->Get_tau()      * this->directionMotor_Front[i];
    }
    for (int i = 0; i < 6; ++i)
    {
        int read_id = this->CAN2_MAP[i];
        auto mot = this->motorsInterface2->getMotor(read_id);
        if (!mot) continue;
        q[i + 6]   = mot->Get_Position() * this->directionMotor_Back[i];
        dq[i + 6]  = mot->Get_Velocity() * this->directionMotor_Back[i];
        tau[i + 6] = mot->Get_tau()      * this->directionMotor_Back[i];
    }
}

// ------------------------------------------------------------------
// 下发 kp=kd=tau=0 的 MIT 命令：
//   电机保持在使能状态（持续回传 q/dq/tau），但不输出任何力矩，
//   相当于"空挡"，可以用手任意转动。
// 注意：send_passive_cmd 置 false 时，不下发任何命令——部分电机固件下
// 会进入超时失能，从而停止回传。如不确定，保持 true 即可。
// ------------------------------------------------------------------
void RL_Real2Mujoco::SendPassiveCommand()
{
    if (!this->send_passive_cmd) return;

    for (int i = 0; i < 6; ++i)
    {
        int id = this->CAN1_MAP[i];
        auto mot = this->motorsInterface->getMotor(id);
        if (mot)
        {
            this->motorsInterface->control_mit(*mot, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
    for (int i = 0; i < 6; ++i)
    {
        int id = this->CAN2_MAP[i];
        auto mot = this->motorsInterface2->getMotor(id);
        if (mot)
        {
            this->motorsInterface2->control_mit(*mot, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }
}

// ------------------------------------------------------------------
// 每个控制周期：真机读数 -> 写 MuJoCo qpos -> 前向运动学 -> 渲染
// ------------------------------------------------------------------
void RL_Real2Mujoco::SyncLoop()
{
    // 退出信号一到，就不再访问 MuJoCo 数据。PhysicsThread 接下来会加锁
    // 释放 m/d；若我们还在 lock 里操作 qpos，就会和它抢锁形成长时间的
    // 阻塞，或在极端竞争下访问到正在被销毁的对象。
    if (!this->sim || this->sim->exitrequest.load() != 0) return;

    // 1) 让真机电机维持在被动状态，同时刷新回传。
    this->SendPassiveCommand();

    // 2) 读最新一帧电机状态（已乘方向修正，就是模型坐标系）
    std::vector<float> q, dq, tau;
    this->ReadMotorState(q, dq, tau);

    // 3) 拿 IMU quat（w,x,y,z）；IMU 未收到时用 identity
    double quat_w = 1.0, quat_x = 0.0, quat_y = 0.0, quat_z = 0.0;
    if (this->imu_received_.load())
    {
        quat_w = this->imu_msg_.orientation.w;
        quat_x = this->imu_msg_.orientation.x;
        quat_y = this->imu_msg_.orientation.y;
        quat_z = this->imu_msg_.orientation.z;
        double n = std::sqrt(quat_w * quat_w + quat_x * quat_x +
                             quat_y * quat_y + quat_z * quat_z);
        if (n > 1e-6)
        {
            quat_w /= n; quat_x /= n; quat_y /= n; quat_z /= n;
        }
        else
        {
            quat_w = 1.0; quat_x = quat_y = quat_z = 0.0;
        }
    }

    // 4) 写入 MuJoCo：必须持有 sim 锁，避免和 PhysicsLoop 争 m/d。
    //    每次都从全局 m/d 重取（PhysicsThread 退出时会在锁内把它们置空），
    //    一旦发现 null 就放弃本次写入，保证不会 UAF。
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

        mjModel *mdl = m;
        mjData  *dat = d;
        if (!mdl || !dat) return;  // PhysicsThread 已释放

        if (this->freeze_base_pos)
        {
            dat->qpos[0] = this->base_pos_init[0];
            dat->qpos[1] = this->base_pos_init[1];
            dat->qpos[2] = this->base_pos_init[2];
        }
        dat->qpos[3] = quat_w;
        dat->qpos[4] = quat_x;
        dat->qpos[5] = quat_y;
        dat->qpos[6] = quat_z;

        // 关节：按 joint_mapping 写 qpos[7 + idx]。
        for (int i = 0; i < this->num_of_dofs; ++i)
        {
            int mj_idx = this->joint_mapping[i];
            dat->qpos[7 + mj_idx] = q[i];
        }

        // qvel 全置零（我们不关心速度，不让残余速度导致渲染抖动）
        for (int i = 0; i < mdl->nv; ++i)
        {
            dat->qvel[i] = 0.0;
        }

        mj_forward(mdl, dat);
    }

    // 5) 发布 /joint_states 以便 rviz 或绘图查看（shutdown 后不再访问 rclcpp）
    if (rclcpp::ok() && this->joint_state_pub_)
    {
        sensor_msgs::msg::JointState js;
        js.header.stamp = this->ros2_node->now();
        js.name = this->joint_names;
        js.position.assign(q.begin(), q.end());
        js.velocity.assign(dq.begin(), dq.end());
        js.effort.assign(tau.begin(), tau.end());
        try
        {
            this->joint_state_pub_->publish(js);
        }
        catch (...)
        {
            // shutdown 过程中偶发异常忽略，避免打断电机失能路径
        }
    }
}

// ------------------------------------------------------------------
// 终端打印，便于人工核对 IMU / 关节映射
// ------------------------------------------------------------------
void RL_Real2Mujoco::PrintLoop()
{
    std::vector<float> q, dq, tau;
    this->ReadMotorState(q, dq, tau);

    auto fmt = [](float v) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << std::setw(7) << v;
        return oss.str();
    };

    std::cout << "\n--- [RL_Real2Mujoco] ---\n";

    // IMU
    if (this->imu_received_.load())
    {
        std::cout << "IMU quat  (w,x,y,z) = "
                  << fmt(this->imu_msg_.orientation.w) << ", "
                  << fmt(this->imu_msg_.orientation.x) << ", "
                  << fmt(this->imu_msg_.orientation.y) << ", "
                  << fmt(this->imu_msg_.orientation.z) << "\n";
        std::cout << "IMU gyro  (x,y,z)   = "
                  << fmt(this->imu_msg_.angular_velocity.x) << ", "
                  << fmt(this->imu_msg_.angular_velocity.y) << ", "
                  << fmt(this->imu_msg_.angular_velocity.z) << "\n";
    }
    else
    {
        std::cout << "IMU       : (未收到 /imu/data，检查 dm_imu_node 是否启动)\n";
    }

    // 关节
    static const char *name[12] = {
        "FL_hip ", "FL_thi ", "FL_cal ",
        "FR_hip ", "FR_thi ", "FR_cal ",
        "RL_hip ", "RL_thi ", "RL_cal ",
        "RR_hip ", "RR_thi ", "RR_cal ",
    };
    std::cout << "joint q (rad, 已乘 direction 修正):\n";
    for (int i = 0; i < this->num_of_dofs; ++i)
    {
        std::cout << "  [" << std::setw(2) << i << "] " << name[i] << " = "
                  << fmt(q[i]);
        if ((i + 1) % 3 == 0) std::cout << "\n";
    }
    std::cout.flush();
}

// ------------------------------------------------------------------
// 正常 Ctrl-C 通路：
//   rclcpp 默认 SIGINT handler  ->  rclcpp::ok()=false  ->  spin 线程
//   发现后主动把 sim->exitrequest=1  ->  RenderLoop 返回  ->
//   main 走到 app 析构  ->  Motor_Control::~ -> disable_all  ->  电机失能
//
// 下面这个 handler 只是个"第二次 Ctrl-C 强退"的兜底，防止极端情况下
// RenderLoop 卡死导致用户无法退出。第一次 Ctrl-C 不走它（rclcpp 那套
// 处理完一切，我们既不抢占也不干扰）。
// ------------------------------------------------------------------
static std::atomic<int> g_signal_count{0};
static void signalHandler(int signum)
{
    int n = ++g_signal_count;
    if (n == 1)
    {
        std::cerr << "\n[RL_Real2Mujoco] signal " << signum
                  << " received, shutting down..." << std::endl;
        if (RL_Real2Mujoco::instance && RL_Real2Mujoco::instance->sim)
        {
            RL_Real2Mujoco::instance->sim->exitrequest.store(1);
        }
        return;
    }
    std::cerr << "[RL_Real2Mujoco] forced exit (motors may not be disabled!)"
              << std::endl;
    std::_Exit(130);
}

int main(int argc, char **argv)
{
    // rclcpp::init 会自己注册 SIGINT/SIGTERM。Ctrl-C 时 rclcpp::ok() 变
    // false，spin 线程检测到之后会把 sim->exitrequest 置 1，MuJoCo 自然
    // 退出，main 回到析构流程，电机由 Motor_Control 析构时 disable_all。
    const std::vector<std::string> filtered_args =
        rclcpp::init_and_remove_ros_arguments(argc, argv);

    // 只做"第二次 Ctrl-C 兜底"。用 sigaction 叠加在 rclcpp 的 handler 上。
    // 注意 signal() 会覆盖 rclcpp 的 handler，因此改用 sigaction 的默认
    // 方式（无 SA_RESETHAND）直接替换——rclcpp 的逻辑由 spin 线程里
    // rclcpp::ok() 的轮询来承担，不依赖它的 handler 本身。
    struct sigaction sa{};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // 构造函数内部会启动 ROS2 spin 线程，然后阻塞在 MuJoCo RenderLoop，
    // 直到用户关闭窗口或按 Ctrl-C。析构时再清理所有线程/CAN 资源。
    RL_Real2Mujoco app(filtered_args);

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
