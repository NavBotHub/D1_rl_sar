/*
 * Copyright (c) 2024-2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * Real robot <-> MuJoCo mapping validation tool.See comments in rl_real2mujoco.hpp.
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
// Constructor: open CAN + start MuJoCo + subscribe to IMU in ROS2 + start sync loops
// ------------------------------------------------------------------
RL_Real2Mujoco::RL_Real2Mujoco(const std::vector<std::string> &args)
{
    instance = this;

    // Command line: rl_real2mujoco <robot_name> <scene_name>
    // Default is d1 + scene. args[0] is the executable name.
    this->robot_name = (args.size() >= 2) ? args[1] : std::string("d1");
    this->scene_name = (args.size() >= 3) ? args[2] : std::string("scene");

    // -------- 1. Read base.yaml --------
    // Use POLICY_DIR/<robot>/base.yaml like rl_real_d1 to keep joint order /
    // joint_mapping / num_of_dofs consistent with the real robot node.
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
        // If yaml is missing it or has the wrong dimension, fall back to 0..N-1
        this->joint_mapping.resize(this->num_of_dofs);
        for (int i = 0; i < this->num_of_dofs; ++i) this->joint_mapping[i] = i;
        std::cout << LOGGER::WARNING
                  << "[RL_Real2Mujoco] joint_mapping missing or wrong size, fallback to identity"
                  << std::endl;
    }

    // -------- 2. Start MuJoCo --------
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

    // Do not detach; otherwise PhysicsThread may keep accessing sim/m/d during destruction,
    // causing a UAF segfault. Join explicitly after RenderLoop exits.
    this->physics_thread_ = std::thread(&PhysicsThread, sim.get(), filename.c_str());

    // Wait until physicsthread has loaded m/d
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
        // Record the initial base position from the keyframe for later freezing
        this->base_pos_init[0] = this->mj_data->qpos[0];
        this->base_pos_init[1] = this->mj_data->qpos[1];
        this->base_pos_init[2] = this->mj_data->qpos[2];
        mj_forward(this->mj_model, this->mj_data);
    }

    // Most important: stop physics simulation and only update qpos + mj_forward each cycle.
    // If not stopped, physics integration will pull the base down and overwrite qpos.
    this->sim->run = 0;

    // -------- 3. Open real robot CAN (12 motors) --------
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

    // -------- 4. ROS2 subscribe to /imu/data + publish /joint_states --------
    this->ros2_node = std::make_shared<rclcpp::Node>("rl_real2mujoco");
    // Use the default reliable QoS like rl_real_d1; dm_imu_node also publishes with default depth 10
    // so SensorDataQoS(best-effort) is QoS-incompatible and will not receive data.
    this->imu_sub_ = this->ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&RL_Real2Mujoco::ImuCallback, this, std::placeholders::_1));
    this->joint_state_pub_ = this->ros2_node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);

    // MuJoCo RenderLoop occupies the main thread, so ROS2 spin must run in a separate thread.
    //
    // This thread also handles the Ctrl+C exit bridge: rclcpp default SIGINT handler
    // sets rclcpp::ok() to false, but does not make MuJoCo RenderLoop exit.
    // After detecting rclcpp shutdown here, set sim->exitrequest to 1 explicitly,
    // so MuJoCo exits RenderLoop on the next frame and the main thread can destruct and disable motors.
    this->ros_spin_running_.store(true);
    this->ros_spin_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(this->ros2_node);
        while (this->ros_spin_running_.load() && rclcpp::ok())
        {
            exec.spin_some(std::chrono::milliseconds(10));
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // At this point either destruction stopped us explicitly (ros_spin_running_=false),
        // or Ctrl+C caused rclcpp shutdown. The latter must notify MuJoCo to exit.
        if (this->sim)
        {
            this->sim->exitrequest.store(1);
        }
    });

    // -------- 5. Start loops --------
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
              << "Hand-rotate real robot joints; corresponding MuJoCo joints should sync;"
              << "rotate the base; the MuJoCo base should sync."
              << std::endl;

    // Enter the MuJoCo render loop (blocking)
    sim->RenderLoop();

    // -------- Ordered cleanup after RenderLoop returns --------
    // This order is important; changing any step may cause SIGSEGV:
    // 1) Stop SyncLoop/PrintLoop first because they still read mj_data.
    if (this->loop_print) { this->loop_print->shutdown(); this->loop_print.reset(); }
    if (this->loop_sync)  { this->loop_sync->shutdown();  this->loop_sync.reset(); }

    // 2) Stop the ROS2 spin thread so it does not keep calling spin_some on a context being torn down.
    this->ros_spin_running_.store(false);
    if (this->ros_spin_thread_.joinable())
    {
        this->ros_spin_thread_.join();
    }

    // 3) Wait for PhysicsThread to finish naturally (it sees exitrequest != 0, exits the loop,
    //    and releases m/d). It must be joined before mj_data=nullptr and sim.reset below,
    //    otherwise it may still run and access dangling pointers.
    if (this->physics_thread_.joinable())
    {
        this->physics_thread_.join();
    }

    // 4) PhysicsThread has now ended, and m/d were deleted by mj_delete*, so they must not be accessed.
    this->mj_model = nullptr;
    this->mj_data  = nullptr;
}

RL_Real2Mujoco::~RL_Real2Mujoco()
{
    instance = nullptr;

    // The constructor may have already shut down at the end; do idempotent cleanup here.
    if (this->loop_print) { this->loop_print->shutdown(); this->loop_print.reset(); }
    if (this->loop_sync)  { this->loop_sync->shutdown();  this->loop_sync.reset(); }

    this->ros_spin_running_.store(false);
    if (this->ros_spin_thread_.joinable())
    {
        this->ros_spin_thread_.join();
    }

    // PhysicsThread should already have been joined after RenderLoop exited; this is a fallback,
    // so if the constructor throws early, PhysicsThread is not still running when the process exits,
    // avoiding thread access after unique_ptr<Simulate>/m/d are destroyed.
    if (this->physics_thread_.joinable())
    {
        // Force exitrequest first to avoid getting stuck in PhysicsLoop
        if (this->sim) this->sim->exitrequest.store(1);
        this->physics_thread_.join();
    }

    // Disable motors explicitly: shared_ptr destruction should also call disable_all(),
    // but call it explicitly here as a safeguard, for example if someone holds a shared_ptr copy
    // or Motor_Control destruction order is abnormal; at least motor torque is already disabled.
    try
    {
        if (this->motorsInterface)  this->motorsInterface->disable_all();
        if (this->motorsInterface2) this->motorsInterface2->disable_all();
    }
    catch (...) {}

    std::cout << LOGGER::INFO << "[RL_Real2Mujoco] motors disabled, exit" << std::endl;
}

// ------------------------------------------------------------------
// IMU callback: cache only; real sampling happens in SyncLoop
// ------------------------------------------------------------------
void RL_Real2Mujoco::ImuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg)
{
    this->imu_msg_ = *msg;
    this->imu_received_.store(true);
}

// ------------------------------------------------------------------
// Read positions / velocities / torques for 12 motors (with direction correction)
// Joint order:FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
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
// Send kp=kd=tau=0 MIT commands:
//   Motors remain enabled and keep returning q/dq/tau, but output no torque,
//   equivalent to neutral, so they can be rotated freely by hand.
// Note: when send_passive_cmd is false, no commands are sent; with some motor firmware,
// motors may timeout-disable and stop feedback. Keep it true if unsure.
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
// Each control cycle: real robot readings -> write MuJoCo qpos -> forward kinematics -> render
// ------------------------------------------------------------------
void RL_Real2Mujoco::SyncLoop()
{
    // Once an exit signal arrives, stop accessing MuJoCo data. PhysicsThread will then lock
    // and release m/d; if we are still operating on qpos under the lock, we may contend with it
    // and block for a long time, or access objects being destroyed under extreme races.
    if (!this->sim || this->sim->exitrequest.load() != 0) return;

    // 1) Keep real robot motors passive while refreshing feedback.
    this->SendPassiveCommand();

    // 2) Read the latest motor state (already direction-corrected into the model coordinate system)
    std::vector<float> q, dq, tau;
    this->ReadMotorState(q, dq, tau);

    // 3) Get IMU quat (w,x,y,z); use identity if no IMU message was received
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

    // 4) Write into MuJoCo: must hold the sim lock to avoid racing PhysicsLoop over m/d.
    //    Fetch global m/d every time (PhysicsThread sets them to null under the lock when exiting),
    //    and skip this write if either is null to avoid UAF.
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);

        mjModel *mdl = m;
        mjData  *dat = d;
        if (!mdl || !dat) return;  // PhysicsThread has released it

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

        // Joints: write qpos[7 + idx] according to joint_mapping.
        for (int i = 0; i < this->num_of_dofs; ++i)
        {
            int mj_idx = this->joint_mapping[i];
            dat->qpos[7 + mj_idx] = q[i];
        }

        // Set all qvel to zero (velocity is irrelevant, and residual velocity should not shake rendering)
        for (int i = 0; i < mdl->nv; ++i)
        {
            dat->qvel[i] = 0.0;
        }

        mj_forward(mdl, dat);
    }

    // 5) Publish /joint_states for rviz or plotting (do not access rclcpp after shutdown)
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
            // Ignore occasional exceptions during shutdown to avoid interrupting the motor-disable path
        }
    }
}

// ------------------------------------------------------------------
// Terminal output for manual IMU / joint mapping checks
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
        std::cout << "IMU       : (no /imu/data received; check whether dm_imu_node is running)\n";
    }

    // Joints
    static const char *name[12] = {
        "FL_hip ", "FL_thi ", "FL_cal ",
        "FR_hip ", "FR_thi ", "FR_cal ",
        "RL_hip ", "RL_thi ", "RL_cal ",
        "RR_hip ", "RR_thi ", "RR_cal ",
    };
    std::cout << "joint q (rad, after direction correction):\n";
    for (int i = 0; i < this->num_of_dofs; ++i)
    {
        std::cout << "  [" << std::setw(2) << i << "] " << name[i] << " = "
                  << fmt(q[i]);
        if ((i + 1) % 3 == 0) std::cout << "\n";
    }
    std::cout.flush();
}

// ------------------------------------------------------------------
// Normal Ctrl-C path:
//   rclcpp default SIGINT handler -> rclcpp::ok()=false -> spin thread
//   detects it and sets sim->exitrequest=1 -> RenderLoop returns ->
//   main reaches app destruction -> Motor_Control::~ -> disable_all -> motors disabled
//
// The handler below is only a fallback for"a second Ctrl-C forced exit"to prevent cases where
// RenderLoop gets stuck and the user cannot exit. The first Ctrl-C does not use it; the rclcpp path
// handles everything, and we neither preempt nor interfere.
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
    // rclcpp::init registers SIGINT/SIGTERM itself. On Ctrl-C, rclcpp::ok() becomes
    // false; the spin thread detects it and sets sim->exitrequest to 1, so MuJoCo naturally
    // exits, main returns to destruction, and Motor_Control disables motors in its destructor.
    const std::vector<std::string> filtered_args =
        rclcpp::init_and_remove_ros_arguments(argc, argv);

    // Only provide a second Ctrl-C fallback. Use sigaction on top of rclcpp handler behavior.
    // Note signal() would overwrite the rclcpp handler, so use sigaction with the default
    // mode (without SA_RESETHAND) as a direct replacement; rclcpp logic is handled by polling
    // rclcpp::ok() in the spin thread and does not depend on its handler itself.
    struct sigaction sa{};
    sa.sa_handler = signalHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT,  &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);

    // The constructor starts the ROS2 spin thread, then blocks in MuJoCo RenderLoop
    // until the user closes the window or presses Ctrl-C. The destructor then cleans all thread/CAN resources.
    RL_Real2Mujoco app(filtered_args);

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
