/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "rl_sim_mujoco.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

RL_Sim* RL_Sim::instance = nullptr;

namespace
{
int FindFootGeom(const mjModel* model, const char* geom_name, const char* body_name)
{
    if (!model)
    {
        return -1;
    }

    const int named_geom = mj_name2id(model, mjOBJ_GEOM, geom_name);
    if (named_geom >= 0)
    {
        return named_geom;
    }

    const int body_id = mj_name2id(model, mjOBJ_BODY, body_name);
    if (body_id < 0)
    {
        return -1;
    }

    for (int geom_id = 0; geom_id < model->ngeom; ++geom_id)
    {
        if (model->geom_bodyid[geom_id] == body_id && model->geom_type[geom_id] == mjGEOM_SPHERE)
        {
            return geom_id;
        }
    }

    return -1;
}
} // namespace

void RL_Sim::ParseDogUsbArgs(int argc, char **argv)
{
    for (int i = 3; i < argc; ++i)
    {
        const std::string arg = argv[i];
        auto next_value = [&](const char *name) -> const char *
        {
            if (i + 1 >= argc)
            {
                std::cout << LOGGER::ERROR << "[dog_usb] missing value for " << name << std::endl;
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--dog-usb-enable")
        {
            this->dog_usb_enable = true;
        }
        else if (arg == "--dog-usb-disable")
        {
            this->dog_usb_enable = false;
        }
        else if (arg == "--dog-usb-device")
        {
            if (const char *value = next_value("--dog-usb-device"))
            {
                this->dog_usb_device = value;
                this->dog_usb_enable = true;
            }
        }
        else if (arg == "--dog-usb-baud")
        {
            if (const char *value = next_value("--dog-usb-baud"))
            {
                this->dog_usb_baud = std::atoi(value);
            }
        }
        else if (arg == "--dog-usb-timeout-ms")
        {
            if (const char *value = next_value("--dog-usb-timeout-ms"))
            {
                this->dog_usb_timeout_ms = std::atoi(value);
            }
        }
        else if (arg == "--dog-remote-timeout-ms")
        {
            if (const char *value = next_value("--dog-remote-timeout-ms"))
            {
                this->dog_remote_timeout_ms = std::atoi(value);
            }
        }
        else if (arg == "--dog-usb-allow-fallback")
        {
            this->dog_usb_allow_fallback = true;
        }
        else if (arg == "--dog-usb-no-fallback")
        {
            this->dog_usb_allow_fallback = false;
        }
        else if (arg == "--csv-auto-record")
        {
            this->csv_auto_record = true;
        }
        else if (arg == "--csv-out")
        {
            if (const char *value = next_value("--csv-out"))
            {
                this->csv_output_path = value;
            }
        }
        else if (arg == "--csv-duration")
        {
            if (const char *value = next_value("--csv-duration"))
            {
                this->csv_auto_duration = std::atof(value);
                if (this->csv_auto_duration < 5.0) { this->csv_auto_duration = 5.0; }
            }
        }
        else if (arg == "--replay-qdes-file")
        {
            if (const char *value = next_value("--replay-qdes-file"))
            {
                this->replay_qdes_path_ = value;
                this->replay_qdes_enabled_ = true;
            }
        }
        else if (arg == "--replay-fixed-base")
        {
            this->replay_fixed_base_ = true;
        }
        else if (arg == "--replay-disable-contact" || arg == "--disable-contact")
        {
            this->replay_disable_contact_ = true;
        }
        else if (arg == "--replay-headless")
        {
            this->replay_headless_ = true;
            this->mujoco_headless_ = true;
        }
        else if (arg == "--mujoco-headless" || arg == "--headless")
        {
            this->mujoco_headless_ = true;
        }
        else if (arg == "--pd-every-mj-step")
        {
            this->pd_every_mj_step_ = true;
        }
        else if (arg == "--replay-root-height")
        {
            if (const char *value = next_value("--replay-root-height"))
            {
                this->replay_root_height_ = std::atof(value);
            }
        }
        else if (arg == "--deploy-thigh-offset")
        {
            if (const char *value = next_value("--deploy-thigh-offset"))
            {
                const float offset = static_cast<float>(std::atof(value));
                for (int index : {1, 4, 7, 10})
                {
                    this->deploy_qdes_offset_[index] += offset;
                }
                this->deploy_qdes_offset_enabled_ = true;
            }
        }
        else if (arg == "--deploy-calf-offset")
        {
            if (const char *value = next_value("--deploy-calf-offset"))
            {
                const float offset = static_cast<float>(std::atof(value));
                for (int index : {2, 5, 8, 11})
                {
                    this->deploy_qdes_offset_[index] += offset;
                }
                this->deploy_qdes_offset_enabled_ = true;
            }
        }
        else
        {
            std::cout << LOGGER::WARNING << "[dog_usb] unknown option: " << arg << std::endl;
        }
    }

    if (this->deploy_qdes_offset_enabled_)
    {
        std::cout << LOGGER::INFO << "[Deploy] q_des offsets:";
        for (float offset : this->deploy_qdes_offset_)
        {
            std::cout << " " << offset;
        }
        std::cout << std::endl;
    }

    if (this->dog_usb_baud < 1) {this->dog_usb_baud = 115200;}
    if (this->dog_usb_timeout_ms < 1) {this->dog_usb_timeout_ms = 300;}
    if (this->dog_remote_timeout_ms < 1) {this->dog_remote_timeout_ms = 1500;}
}

RL_Sim::RL_Sim(int argc, char **argv)
{
    // Set static instance pointer early for signal handler
    instance = this;

    if (argc < 3)
    {
        std::cout << LOGGER::ERROR << "Usage: " << argv[0] << " robot_name scene_name" << std::endl;
        throw std::runtime_error("Invalid arguments");
    }
    else
    {
        this->robot_name = argv[1];
        this->scene_name = argv[2];
    }
    this->ParseDogUsbArgs(argc, argv);

    this->ang_vel_axis = "body";

    // now launch mujoco
    std::cout << LOGGER::INFO << "[MuJoCo] Launching..." << std::endl;

    // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
    if (rosetta_error_msg)
    {
        DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
        std::exit(1);
    }
#endif

    // print version, check compatibility
    std::cout << LOGGER::INFO << "[MuJoCo] Version: " << mj_versionString() << std::endl;
    if (mjVERSION_HEADER != mj_version())
    {
        mju_error("Headers and library have different versions");
    }

    // scan for libraries in the plugin directory to load additional plugins
    scanPluginLibraries();

    std::string filename = std::string(CMAKE_CURRENT_SOURCE_DIR) + "/../rl_sar_zoo/" + this->robot_name + "_description/mjcf/" + this->scene_name + ".xml";

    if (this->mujoco_headless_)
    {
        char load_error[1024] = "";
        m = mj_loadXML(filename.c_str(), nullptr, load_error, sizeof(load_error));
        if (!m)
        {
            throw std::runtime_error(std::string("Failed to load MuJoCo XML: ") + load_error);
        }
        d = mj_makeData(m);
        if (!d)
        {
            mj_deleteModel(m);
            m = nullptr;
            throw std::runtime_error("Failed to allocate MuJoCo data");
        }
        std::cout << LOGGER::INFO << "[MuJoCo] Headless data prepared" << std::endl;
    }
    else
    {
        mjvCamera cam;
        mjv_defaultCamera(&cam);

        mjvOption opt;
        mjv_defaultOption(&opt);

        mjvPerturb pert;
        mjv_defaultPerturb(&pert);

        // simulate object encapsulates the UI
        sim = std::make_unique<mj::Simulate>(
            std::make_unique<mj::GlfwAdapter>(),
            &cam, &opt, &pert, /* is_passive = */ false);

        // start physics thread
        std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename.c_str());
        physicsthreadhandle.detach();

        while (1)
        {
            if (d)
            {
                std::cout << LOGGER::INFO << "[MuJoCo] Data prepared" << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    this->mj_model = m;
    this->mj_data = d;

    if (this->replay_disable_contact_ && this->mj_model)
    {
        this->mj_model->opt.disableflags |= mjDSBL_CONTACT;
        std::cout << LOGGER::INFO << "[Replay] MuJoCo contact disabled for diagnostics" << std::endl;
    }
    if (this->mj_model)
    {
        this->mj_diag_data_ = mj_makeData(this->mj_model);
    }

    if (this->sim)
    {
        const std::lock_guard<std::recursive_mutex> lock(sim->mtx);
        if (this->mj_model->nkey > 0)
        {
            mj_resetDataKeyframe(this->mj_model, this->mj_data, 0);
        }
        mj_forward(this->mj_model, this->mj_data);
    }
    else
    {
        if (this->mj_model->nkey > 0)
        {
            mj_resetDataKeyframe(this->mj_model, this->mj_data, 0);
        }
        mj_forward(this->mj_model, this->mj_data);
    }

    this->foot_geom_ids_ = {
        FindFootGeom(this->mj_model, "FL_foot_collision", "FL_calf"),
        FindFootGeom(this->mj_model, "FR_foot_collision", "FR_calf"),
        FindFootGeom(this->mj_model, "RL_foot_collision", "RL_calf"),
        FindFootGeom(this->mj_model, "RR_foot_collision", "RR_calf")
    };
    if (std::any_of(this->foot_geom_ids_.begin(), this->foot_geom_ids_.end(), [](int geom_id) { return geom_id < 0; }))
    {
        std::cout << LOGGER::WARNING << "[CSV] Some foot collision geom ids are missing; corresponding foot_z/contact fields default to 0."
                   << std::endl;
    }

    this->SetupSysJoystick("/dev/input/js0", 16); // 16 bits joystick   //  ls -l /dev/input/js*

    // read params from yaml
    this->ReadYaml(this->robot_name, "base.yaml");

    // auto load FSM by robot_name
    if (FSMManager::GetInstance().IsTypeSupported(this->robot_name))
    {
        auto fsm_ptr = FSMManager::GetInstance().CreateFSM(this->robot_name, this);
        if (fsm_ptr)
        {
            this->fsm = *fsm_ptr;
        }
    }
    else
    {
        std::cout << LOGGER::ERROR << "[FSM] No FSM registered for robot: " << this->robot_name << std::endl;
    }

    // init robot
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->LoadReplayQdes();

    if (this->dog_usb_enable)
    {
        if (this->dog_usb_device.empty())
        {
            std::cout << LOGGER::ERROR
                      << "[dog_usb] --dog-usb-enable set but --dog-usb-device is empty; not starting dog USB receiver."
                      << std::endl;
        }
        else
        {
            this->dog_usb_receiver = std::make_unique<DogUsbReceiver>();
            if (!this->dog_usb_receiver->Start(this->dog_usb_device, this->dog_usb_baud))
            {
                std::cout << LOGGER::ERROR << "[dog_usb] receiver start failed for "
                          << this->dog_usb_device << std::endl;
                this->dog_usb_receiver.reset();
            }
        }
    }

    // loop
    if (!this->replay_headless_ && !this->mujoco_headless_)
    {
        this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Sim::RobotControl, this));
        this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Sim::RunModel, this));
        this->loop_control->start();
        this->loop_rl->start();

        // keyboard
        this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Sim::GetKeyboard, this));
        this->loop_keyboard->start();

        // joystick
        this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Sim::GetSysJoystick, this));
        this->loop_joystick->start();
    }

#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.001, std::bind(&RL_Sim::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
    //CSV init
    this->csv_ofs.open(this->csv_output_path);
    this->csv_start_time = std::chrono::steady_clock::now();
    if (this->csv_auto_record)
    {
        std::cout << LOGGER::INFO << "[CSV] auto record enabled, output=" << this->csv_output_path
                  << " duration=" << this->csv_auto_duration << "s" << std::endl;
    }
    if (this->replay_qdes_enabled_ || (this->mujoco_headless_ && this->csv_auto_record))
    {
        simulation_running = true;
        if (this->sim)
        {
            this->sim->run = 1;
        }
        std::cout << LOGGER::INFO << "[MuJoCo] simulation auto-started" << std::endl;
    }
    std::cout << LOGGER::INFO << "RL_Sim start" << std::endl;

    if (this->mujoco_headless_ && (this->replay_qdes_enabled_ || this->csv_auto_record))
    {
        const double dt = std::max(static_cast<double>(this->params.Get<float>("dt")), 1.0e-4);
        const double duration = this->csv_auto_duration > 0.0 ? this->csv_auto_duration : 12.0;
        const int steps = std::max(1, static_cast<int>(std::ceil(duration / dt)));
        const auto wall_start = std::chrono::steady_clock::now();
        const double mujoco_dt = (this->mj_model && this->mj_model->opt.timestep > 0.0)
            ? this->mj_model->opt.timestep
            : dt;
        double sim_step_accum = 0.0;
        std::cout << LOGGER::INFO << "[MuJoCo] headless loop steps=" << steps
                  << " control_dt=" << dt << " mujoco_dt=" << mujoco_dt << std::endl;
        const int control_decimation = std::max(1, this->params.Get<int>("decimation"));
        for (int step = 0; step < steps; ++step)
        {
            const double sim_t = step * dt;
            this->csv_start_time = std::chrono::steady_clock::now()
                - std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(sim_t));
            if (!this->replay_qdes_enabled_ && step % control_decimation == 0)
            {
                this->RunModel();
            }
            this->RobotControl();
            if (this->csv_auto_exit_sent)
            {
                break;
            }
            if (this->mj_model && this->mj_data)
            {
                sim_step_accum += dt;
                while (sim_step_accum + 1.0e-12 >= mujoco_dt)
                {
                    if (this->pd_every_mj_step_)
                    {
                        this->GetState(&this->robot_state);
                        this->SetCommand(&this->robot_command);
                    }
                    mj_step(this->mj_model, this->mj_data);
                    ++this->mj_step_count_;
                    sim_step_accum -= mujoco_dt;
                }
            }
        }
        std::cout << LOGGER::INFO << "[MuJoCo] counters RobotControl=" << this->robot_control_count_
                  << " SetCommand=" << this->set_command_count_
                  << " RunModel=" << this->run_model_count_
                  << " mj_step=" << this->mj_step_count_ << std::endl;
        this->csv_start_time = wall_start;
        if (this->csv_ofs.is_open())
        {
            this->csv_ofs.flush();
        }
        return;
    }

    // start simulation UI loop (blocking call)
    if (sim)
    {
        sim->RenderLoop();
    }
}

RL_Sim::~RL_Sim()
{
    // Clear static instance pointer
    instance = nullptr;

    if (this->loop_keyboard) { this->loop_keyboard->shutdown(); }
    if (this->loop_joystick) { this->loop_joystick->shutdown(); }
    if (this->loop_control) { this->loop_control->shutdown(); }
    if (this->loop_rl) { this->loop_rl->shutdown(); }
    if (this->dog_usb_receiver)
    {
        this->dog_usb_receiver->Stop();
    }
    //CSV close
    if (this->csv_ofs.is_open())
    {
        this->csv_ofs.close();
    }
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    if (this->mj_diag_data_)
    {
        mj_deleteData(this->mj_diag_data_);
        this->mj_diag_data_ = nullptr;
    }
    if (this->mujoco_headless_)
    {
        if (d)
        {
            mj_deleteData(d);
            d = nullptr;
        }
        if (m)
        {
            mj_deleteModel(m);
            m = nullptr;
        }
    }
    std::cout << LOGGER::INFO << "RL_Sim exit" << std::endl;
}

void RL_Sim::GetState(RobotState<float> *state)
{
    if (mj_data)
    {
        state->imu.quaternion[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 0];
        state->imu.quaternion[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 1];
        state->imu.quaternion[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 2];
        state->imu.quaternion[3] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 3];

        state->imu.gyroscope[0] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 4];
        state->imu.gyroscope[1] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 5];
        state->imu.gyroscope[2] = mj_data->sensordata[3 * this->params.Get<int>("num_of_dofs") + 6];

        const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            const int actuator_id = i < static_cast<int>(joint_mapping.size()) ? joint_mapping[i] : -1;
            state->motor_state.q[i] = static_cast<float>(this->JointQ(actuator_id));
            state->motor_state.dq[i] = static_cast<float>(this->JointDq(actuator_id));
            state->motor_state.tau_est[i] = (mj_model && actuator_id >= 0 && actuator_id < mj_model->nu)
                ? static_cast<float>(mj_data->actuator_force[actuator_id])
                : 0.f;
        }
    }
}

float RL_Sim::CommandQWithDeployOffset(const RobotCommand<float> *command, int index) const
{
    if (!command || index < 0 || index >= static_cast<int>(this->deploy_qdes_offset_.size()))
    {
        return 0.0f;
    }
    return command->motor_command.q[index] + this->deploy_qdes_offset_[index];
}

int RL_Sim::JointQposIdFromActuator(int actuator_id) const
{
    if (!this->mj_model || actuator_id < 0 || actuator_id >= this->mj_model->nu)
    {
        return -1;
    }
    const int joint_id = this->mj_model->actuator_trnid[2 * actuator_id];
    if (joint_id < 0 || joint_id >= this->mj_model->njnt)
    {
        return -1;
    }
    return this->mj_model->jnt_qposadr[joint_id];
}

int RL_Sim::JointDofIdFromActuator(int actuator_id) const
{
    if (!this->mj_model || actuator_id < 0 || actuator_id >= this->mj_model->nu)
    {
        return -1;
    }
    const int joint_id = this->mj_model->actuator_trnid[2 * actuator_id];
    if (joint_id < 0 || joint_id >= this->mj_model->njnt)
    {
        return -1;
    }
    return this->mj_model->jnt_dofadr[joint_id];
}

double RL_Sim::JointQ(int actuator_id) const
{
    const int qpos_id = this->JointQposIdFromActuator(actuator_id);
    if (!this->mj_data || qpos_id < 0 || qpos_id >= this->mj_model->nq)
    {
        return 0.0;
    }
    return this->mj_data->qpos[qpos_id];
}

double RL_Sim::JointDq(int actuator_id) const
{
    const int qvel_id = this->JointDofIdFromActuator(actuator_id);
    if (!this->mj_data || qvel_id < 0 || qvel_id >= this->mj_model->nv)
    {
        return 0.0;
    }
    return this->mj_data->qvel[qvel_id];
}

float RL_Sim::TorqueLimit(int index) const
{
    const auto limits = this->params.Get<std::vector<float>>("torque_limits");
    if (index >= 0 && index < static_cast<int>(limits.size()) && limits[index] > 0.f)
    {
        return std::abs(limits[index]);
    }
    return 1.0e9f;
}

void RL_Sim::SetCommand(const RobotCommand<float> *command)
{
    if (mj_data)
    {
        ++this->set_command_count_;
        const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        const int dof_count = std::min(this->params.Get<int>("num_of_dofs"), static_cast<int>(joint_mapping.size()));
        for (int i = 0; i < dof_count; ++i)
        {
            const int actuator_id = joint_mapping[i];
            const float q_des = this->CommandQWithDeployOffset(command, i);
            const double q = this->JointQ(actuator_id);
            const double dq = this->JointDq(actuator_id);
            const double tau_raw =
                command->motor_command.tau[i] +
                command->motor_command.kp[i] * (q_des - q) +
                command->motor_command.kd[i] * (command->motor_command.dq[i] - dq);
            const double limit = this->TorqueLimit(i);
            mj_data->ctrl[actuator_id] = std::clamp(tau_raw, -limit, limit);
        }
    }
}

void RL_Sim::LoadReplayQdes()
{
    if (!this->replay_qdes_enabled_)
    {
        return;
    }

    std::ifstream file(this->replay_qdes_path_);
    if (!file.is_open())
    {
        throw std::runtime_error("Failed to open replay q_des file: " + this->replay_qdes_path_);
    }

    std::string line;
    while (std::getline(file, line))
    {
        if (line.empty())
        {
            continue;
        }
        std::replace(line.begin(), line.end(), ',', ' ');
        std::istringstream iss(line);
        std::array<float, 12> qdes{};
        bool ok = true;
        for (float &value : qdes)
        {
            if (!(iss >> value))
            {
                ok = false;
                break;
            }
        }
        if (ok)
        {
            this->replay_qdes_.push_back(qdes);
        }
    }

    if (this->replay_qdes_.empty())
    {
        throw std::runtime_error("Replay q_des file has no valid 12-column rows: " + this->replay_qdes_path_);
    }

    std::cout << LOGGER::INFO << "[Replay] loaded " << this->replay_qdes_.size()
              << " q_des rows from " << this->replay_qdes_path_;
    if (this->replay_fixed_base_)
    {
        std::cout << " with fixed base height=" << this->replay_root_height_;
    }
    std::cout << std::endl;
}

void RL_Sim::ApplyReplayQdes(double t)
{
    if (!this->replay_qdes_enabled_ || this->replay_qdes_.empty())
    {
        return;
    }
    bool first_replay_step = false;
    if (!this->replay_started_)
    {
        this->replay_started_ = true;
        this->replay_start_t_ = t;
        first_replay_step = true;
    }

    const double replay_t = std::max(0.0, t - this->replay_start_t_);
    const double dt = std::max(static_cast<double>(this->params.Get<float>("dt")), 1.0e-4);
    const size_t row_index = std::min(
        static_cast<size_t>(replay_t / dt),
        this->replay_qdes_.size() - 1);
    const auto &qdes = this->replay_qdes_[row_index];
    auto kp = this->params.Get<std::vector<float>>("rl_kp");
    auto kd = this->params.Get<std::vector<float>>("rl_kd");
    if (kp.empty()) { kp = this->params.Get<std::vector<float>>("fixed_kp"); }
    if (kd.empty()) { kd = this->params.Get<std::vector<float>>("fixed_kd"); }
    const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    const int dof_count = std::min(12, this->params.Get<int>("num_of_dofs"));
    const int command_count = std::min(
        dof_count,
        std::min(static_cast<int>(kp.size()),
                 std::min(static_cast<int>(kd.size()), static_cast<int>(this->robot_command.motor_command.q.size()))));

    for (int i = 0; i < command_count; ++i)
    {
        this->robot_command.motor_command.q[i] = qdes[i];
        this->robot_command.motor_command.dq[i] = 0.0f;
        this->robot_command.motor_command.tau[i] = 0.0f;
        this->robot_command.motor_command.kp[i] = kp[i];
        this->robot_command.motor_command.kd[i] = kd[i];
    }

    if (first_replay_step && this->mj_model && this->mj_data)
    {
        for (int i = 0; i < command_count && i < static_cast<int>(joint_mapping.size()); ++i)
        {
            const int actuator_id = joint_mapping[i];
            if (actuator_id < 0 || actuator_id >= this->mj_model->nu) { continue; }
            const int joint_id = this->mj_model->actuator_trnid[2 * actuator_id];
            if (joint_id < 0 || joint_id >= this->mj_model->njnt) { continue; }
            const int qpos_id = this->mj_model->jnt_qposadr[joint_id];
            const int qvel_id = this->mj_model->jnt_dofadr[joint_id];
            if (qpos_id >= 0 && qpos_id < this->mj_model->nq)
            {
                this->mj_data->qpos[qpos_id] = qdes[i];
            }
            if (qvel_id >= 0 && qvel_id < this->mj_model->nv)
            {
                this->mj_data->qvel[qvel_id] = 0.0;
            }
        }
        mj_forward(this->mj_model, this->mj_data);
    }

    this->csv_segment_name = this->replay_fixed_base_ ? "replay_fixed" : "replay_ground";

    if (this->replay_fixed_base_ && this->mj_model && this->mj_data && this->mj_model->nq >= 7 && this->mj_model->nv >= 6)
    {
        this->mj_data->qpos[0] = 0.0;
        this->mj_data->qpos[1] = 0.0;
        this->mj_data->qpos[2] = this->replay_root_height_;
        this->mj_data->qpos[3] = 1.0;
        this->mj_data->qpos[4] = 0.0;
        this->mj_data->qpos[5] = 0.0;
        this->mj_data->qpos[6] = 0.0;
        for (int i = 0; i < 6; ++i)
        {
            this->mj_data->qvel[i] = 0.0;
        }
        mj_forward(this->mj_model, this->mj_data);
    }
}

bool RL_Sim::DogUsbShouldBlockFallback() const
{
    if (!this->dog_usb_enable || !this->dog_usb_receiver)
    {
        return false;
    }

    const DogUsbState state = this->dog_usb_receiver->GetLatestState();
    if (!state.valid)
    {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool usb_fresh =
        this->dog_usb_receiver->IsConnected() &&
        (now - state.last_valid_frame_time <= std::chrono::milliseconds(this->dog_usb_timeout_ms));

    if (!this->dog_usb_allow_fallback)
    {
        return true;
    }
    return usb_fresh;
}

void RL_Sim::TriggerDogUsbGamepad(Input::Gamepad gamepad, const char *name)
{
    this->control.SetGamepad(gamepad);
    std::cout << LOGGER::INFO << "[dog_usb] event " << name << std::endl;
}

void RL_Sim::LogDogUsbStatus(bool safe_state, bool usb_timeout, bool remote_timeout, bool serial_connected)
{
    if (!this->dog_usb_status_initialized || this->dog_usb_last_safe_state != safe_state)
    {
        std::cout << (safe_state ? LOGGER::WARNING : LOGGER::INFO)
                  << "[dog_usb] safe_state=" << (safe_state ? "ON" : "OFF") << std::endl;
    }
    if (!this->dog_usb_status_initialized || this->dog_usb_last_usb_timeout != usb_timeout)
    {
        std::cout << (usb_timeout ? LOGGER::WARNING : LOGGER::INFO)
                  << "[dog_usb] usb_timeout=" << (usb_timeout ? "ON" : "OFF") << std::endl;
    }
    if (!this->dog_usb_status_initialized || this->dog_usb_last_remote_timeout != remote_timeout)
    {
        std::cout << (remote_timeout ? LOGGER::WARNING : LOGGER::INFO)
                  << "[dog_usb] remote_timeout=" << (remote_timeout ? "ON" : "OFF") << std::endl;
    }
    if (!this->dog_usb_status_initialized || this->dog_usb_last_serial_connected != serial_connected)
    {
        std::cout << (serial_connected ? LOGGER::INFO : LOGGER::WARNING)
                  << "[dog_usb] serial_connected=" << (serial_connected ? "ON" : "OFF") << std::endl;
    }

    this->dog_usb_status_initialized = true;
    this->dog_usb_last_safe_state = safe_state;
    this->dog_usb_last_usb_timeout = usb_timeout;
    this->dog_usb_last_remote_timeout = remote_timeout;
    this->dog_usb_last_serial_connected = serial_connected;
}

void RL_Sim::ApplyDogUsbControl(bool emit_events)
{
    if (!this->dog_usb_enable || !this->dog_usb_receiver)
    {
        return;
    }

    const DogUsbState state = this->dog_usb_receiver->GetLatestState();
    if (!state.valid)
    {
        return;
    }

    this->dog_usb_has_taken_control = true;

    const auto now = std::chrono::steady_clock::now();
    const bool serial_connected = this->dog_usb_receiver->IsConnected();
    const bool usb_fresh =
        serial_connected &&
        (now - state.last_valid_frame_time <= std::chrono::milliseconds(this->dog_usb_timeout_ms));
    const bool remote_fresh =
        state.last_remote_age_ms <= static_cast<uint32_t>(this->dog_remote_timeout_ms);
    const bool dog_safe = state.safe_state;
    const bool usb_timeout = !usb_fresh;
    const bool remote_timeout = !remote_fresh;
    const bool channel_owns = !this->dog_usb_allow_fallback || usb_fresh;
    const bool dog_control_valid = channel_owns && usb_fresh && remote_fresh && !dog_safe;

    if (emit_events)
    {
        this->LogDogUsbStatus(dog_safe, usb_timeout, remote_timeout, serial_connected);
        if (channel_owns)
        {
            this->control.current_keyboard = Input::Keyboard::None;
            this->control.last_keyboard = Input::Keyboard::None;
            this->control.current_gamepad = Input::Gamepad::None;
        }
    }

    if (dog_control_valid)
    {
        this->control.x = static_cast<float>(state.x);
        this->control.y = static_cast<float>(state.y);
        this->control.yaw = static_cast<float>(state.yaw);
    }
    else if (channel_owns || dog_safe || remote_timeout || !serial_connected)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
    }

    if (!emit_events || !usb_fresh)
    {
        return;
    }

    constexpr uint16_t kBtnSwL2 = 1u << 9;
    constexpr uint16_t kBtnSwR1 = 1u << 10;
    constexpr uint16_t kBtnSwR2 = 1u << 11;

    const uint16_t buttons = state.buttons;
    const uint16_t prev_buttons = this->dog_usb_prev_buttons;

    if (dog_control_valid)
    {
        const bool r1_now = (buttons & kBtnSwR1) != 0;
        const bool r1_prev = (prev_buttons & kBtnSwR1) != 0;
        if (r1_now && !r1_prev)
        {
            this->TriggerDogUsbGamepad(Input::Gamepad::A, "R1_ON_GetUp");
        }
        else if (!r1_now && r1_prev)
        {
            this->TriggerDogUsbGamepad(Input::Gamepad::B, "R1_OFF_GetDown");
        }

        const bool l2_now = (buttons & kBtnSwL2) != 0;
        const bool l2_prev = (prev_buttons & kBtnSwL2) != 0;
        if (l2_now && !l2_prev)
        {
            this->TriggerDogUsbGamepad(Input::Gamepad::LB_X, "L2_ON_Passive");
        }

        const bool r2_now = (buttons & kBtnSwR2) != 0;
        const bool r2_prev = (prev_buttons & kBtnSwR2) != 0;
        if (r2_now && !r2_prev)
        {
            this->TriggerDogUsbGamepad(Input::Gamepad::RB_DPadUp, "R2_ON_RLLocomotion");
        }
    }

    this->dog_usb_prev_buttons = buttons;
}

void RL_Sim::GetKeyboard()
{
    if (this->DogUsbShouldBlockFallback())
    {
        return;
    }
    this->KeyboardInterface();
}


void RL_Sim::UpdateCsvAutoRecord(double t)
{
    if (!this->csv_ofs.is_open())
    {
        return;
    }
    if (!this->csv_auto_record)
    {
        this->csv_segment_name = "manual";
        return;
    }

    if (!this->csv_auto_getup_sent && t >= 0.5)
    {
        this->control.SetGamepad(Input::Gamepad::A);
        this->csv_auto_getup_sent = true;
        std::cout << LOGGER::INFO << "[CSV] auto get-up" << std::endl;
    }
    if (!this->csv_auto_locomotion_sent && t >= 4.0)
    {
        this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
        this->csv_auto_locomotion_sent = true;
        std::cout << LOGGER::INFO << "[CSV] auto RL locomotion" << std::endl;
    }

    struct Segment
    {
        double start;
        double end;
        const char *name;
        float x;
        float y;
        float yaw;
    };
    static constexpr Segment kSegments[] = {
        {0.0, 6.0, "stand_init", 0.0f, 0.0f, 0.0f},
        {6.0, 16.0, "back_0p3", -0.3f, 0.0f, 0.0f},
        {16.0, 26.0, "back_0p5", -0.5f, 0.0f, 0.0f},
        {26.0, 36.0, "back_0p7", -0.7f, 0.0f, 0.0f},
        {36.0, 46.0, "back_0p9", -0.9f, 0.0f, 0.0f},
        {46.0, 52.0, "stand_end", 0.0f, 0.0f, 0.0f},
    };

    const Segment *active = &kSegments[0];
    for (const auto &segment : kSegments)
    {
        if (t >= segment.start && t < segment.end)
        {
            active = &segment;
            break;
        }
    }
    this->csv_segment_name = active->name;
    this->control.x = active->x;
    this->control.y = active->y;
    this->control.yaw = active->yaw;

    const double stop_time = this->csv_auto_duration > 0.0 ? this->csv_auto_duration : 104.0;
    if (!this->csv_auto_exit_sent && t >= stop_time)
    {
        this->csv_auto_exit_sent = true;
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        if (this->csv_ofs.is_open())
        {
            this->csv_ofs.flush();
        }
        if (this->sim)
        {
            this->sim->exitrequest.store(1);
        }
        std::cout << LOGGER::INFO << "[CSV] auto record finished" << std::endl;
    }
}

void RL_Sim::RobotControl()
{
    ++this->robot_control_count_;
    // Lock the sim mutex once for the entire control cycle to prevent race conditions.
    std::unique_lock<std::recursive_mutex> lock;
    if (this->sim)
    {
        lock = std::unique_lock<std::recursive_mutex>(this->sim->mtx);
    }
    this->GetState(&this->robot_state);
    const double elapsed_t = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - this->csv_start_time
    ).count();
    this->UpdateCsvAutoRecord(elapsed_t);
    if (this->replay_qdes_enabled_)
    {
        this->ApplyReplayQdes(elapsed_t);
    }
    // std::cout << "关节位置：" << std::endl;
    // for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    // {
    //     std::cout << "关节 " << i << ": " << this->robot_state.motor_state.q[i] << std::endl;
    // }
    if (this->csv_ofs.is_open())
    {
        if (!this->csv_header_written)
        {
            this->csv_ofs << "t,segment,command_x,command_y,command_yaw";
            for (int i = 0; i < 12; ++i)
            {
                this->csv_ofs << ",q" << i << ",arm_q" << i;
            }
            for (int i = 0; i < 12; ++i)
            {
                this->csv_ofs << ",qd" << i
                              << ",tau_pd" << i
                              << ",ctrl" << i
                              << ",actuator_force" << i
                              << ",qfrc_actuator" << i
                              << ",qfrc_passive" << i
                              << ",qfrc_constraint" << i
                              << ",tau_limit" << i
                              << ",tau_clip" << i;
            }
            this->csv_ofs << ",base_roll,base_pitch,base_yaw,base_z,base_height,base_height_ground,stance_foot_z_mean"
                          << ",base_vel_x,base_vel_y,base_yaw_vel,base_ang_vel_x,base_ang_vel_y,base_ang_vel_z"
                          << ",foot_x_FL,foot_y_FL,foot_z_FL,foot_x_FR,foot_y_FR,foot_z_FR,foot_x_RL,foot_y_RL,foot_z_RL,foot_x_RR,foot_y_RR,foot_z_RR"
                          << ",foot_vx_FL,foot_vy_FL,foot_vz_FL,foot_vx_FR,foot_vy_FR,foot_vz_FR,foot_vx_RL,foot_vy_RL,foot_vz_RL,foot_vx_RR,foot_vy_RR,foot_vz_RR"
                          << ",foot_speed_xy_FL,foot_speed_xy_FR,foot_speed_xy_RL,foot_speed_xy_RR"
                          << ",contact_FL,contact_FR,contact_RL,contact_RR"
                          << ",target_foot_z_FL,target_foot_z_FR,target_foot_z_RL,target_foot_z_RR"
                          << ",target_swing_contact_FL,target_swing_contact_FR,target_swing_contact_RL,target_swing_contact_RR"
                          << ",contact_speed_xy_FL,contact_speed_xy_FR,contact_speed_xy_RL,contact_speed_xy_RR"
                          << ",air_time_FL,air_time_FR,air_time_RL,air_time_RR\n";
            this->csv_header_written = true;
        }

        double t = elapsed_t;

        const std::vector<float>& qv = this->robot_state.imu.quaternion;
        const float w = qv.size() > 0 ? qv[0] : 0.f;
        const float x = qv.size() > 1 ? qv[1] : 0.f;
        const float y = qv.size() > 2 ? qv[2] : 0.f;
        const float z = qv.size() > 3 ? qv[3] : 0.f;
        const float sinp = std::clamp(2.f * (w * y - z * x), -1.f, 1.f);
        const float base_pitch = std::asin(sinp);
        const float base_roll = std::atan2(2.f * (w * x + y * z), 1.f - 2.f * (x * x + y * y));
        const float base_yaw = std::atan2(2.f * (w * z + x * y), 1.f - 2.f * (y * y + z * z));

        const float base_z = this->mj_data ? static_cast<float>(this->mj_data->qpos[2]) : 0.f;
        std::array<std::array<float, 3>, 4> foot_pos = {{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}}};
        std::array<std::array<float, 3>, 4> foot_vel = {{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}}};
        std::array<float, 4> foot_z = {0.f, 0.f, 0.f, 0.f};
        std::array<int, 4> contact = {0, 0, 0, 0};
        std::array<float, 4> target_foot_z = {0.f, 0.f, 0.f, 0.f};
        if (this->mj_data)
        {
            for (size_t leg = 0; leg < this->foot_geom_ids_.size(); ++leg)
            {
                const int geom_id = this->foot_geom_ids_[leg];
                if (geom_id >= 0)
                {
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        foot_pos[leg][axis] = this->mj_data->geom_xpos[3 * geom_id + axis];
                    }
                    foot_z[leg] = foot_pos[leg][2];
                }
            }

            const float foot_dt = std::max(this->params.Get<float>("dt"), 1.0e-4f);
            if (this->foot_prev_valid_)
            {
                for (size_t leg = 0; leg < this->foot_geom_ids_.size(); ++leg)
                {
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        foot_vel[leg][axis] = (foot_pos[leg][axis] - this->prev_foot_pos_[leg][axis]) / foot_dt;
                    }
                }
            }
            this->prev_foot_pos_ = foot_pos;
            this->foot_prev_valid_ = true;

            for (int ci = 0; ci < this->mj_data->ncon; ++ci)
            {
                const int g1 = this->mj_data->contact[ci].geom1;
                const int g2 = this->mj_data->contact[ci].geom2;
                for (size_t leg = 0; leg < this->foot_geom_ids_.size(); ++leg)
                {
                    const int geom_id = this->foot_geom_ids_[leg];
                    if (geom_id >= 0 && (g1 == geom_id || g2 == geom_id))
                    {
                        contact[leg] = 1;
                    }
                }
            }
        }

        if (this->mj_model && this->mj_data && this->mj_diag_data_)
        {
            mj_copyData(this->mj_diag_data_, this->mj_model, this->mj_data);
            const auto target_joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
            const int target_dof_count = std::min(12, this->params.Get<int>("num_of_dofs"));
            for (int i = 0; i < target_dof_count && i < static_cast<int>(target_joint_mapping.size()); ++i)
            {
                const int actuator_id = target_joint_mapping[i];
                const int qpos_id = this->JointQposIdFromActuator(actuator_id);
                const int qvel_id = this->JointDofIdFromActuator(actuator_id);
                if (qpos_id >= 0 && qpos_id < this->mj_model->nq)
                {
                    this->mj_diag_data_->qpos[qpos_id] = this->CommandQWithDeployOffset(&this->robot_command, i);
                }
                if (qvel_id >= 0 && qvel_id < this->mj_model->nv)
                {
                    this->mj_diag_data_->qvel[qvel_id] = this->robot_command.motor_command.dq[i];
                }
            }
            mj_forward(this->mj_model, this->mj_diag_data_);
            for (size_t leg = 0; leg < this->foot_geom_ids_.size(); ++leg)
            {
                const int geom_id = this->foot_geom_ids_[leg];
                if (geom_id >= 0)
                {
                    target_foot_z[leg] = static_cast<float>(this->mj_diag_data_->geom_xpos[3 * geom_id + 2]);
                }
            }
        }

        const float log_dt = this->params.Get<float>("dt");
        for (size_t leg = 0; leg < this->foot_air_time_.size(); ++leg)
        {
            if (contact[leg])
            {
                this->foot_air_time_[leg] = 0.f;
            }
            else
            {
                this->foot_air_time_[leg] += log_dt;
            }
        }

        float stance_foot_z_sum = 0.f;
        int stance_foot_count = 0;
        for (size_t leg = 0; leg < foot_z.size(); ++leg)
        {
            if (contact[leg])
            {
                stance_foot_z_sum += foot_z[leg];
                ++stance_foot_count;
            }
        }
        if (stance_foot_count == 0)
        {
            for (float z_pos : foot_z)
            {
                stance_foot_z_sum += z_pos;
            }
            stance_foot_count = static_cast<int>(foot_z.size());
        }
        const float stance_foot_z_mean = stance_foot_z_sum / static_cast<float>(stance_foot_count);
        const float base_height = base_z - stance_foot_z_mean;
        // Keep the old foot-center-relative height, and also log the ground-relative root height used by Isaac-style rewards.
        const float base_height_ground = base_z;
        const float base_vel_x = (this->mj_data && this->mj_model && this->mj_model->nv > 0) ? static_cast<float>(this->mj_data->qvel[0]) : 0.f;
        const float base_vel_y = (this->mj_data && this->mj_model && this->mj_model->nv > 1) ? static_cast<float>(this->mj_data->qvel[1]) : 0.f;
        const float base_yaw_vel = (this->mj_data && this->mj_model && this->mj_model->nv > 5) ? static_cast<float>(this->mj_data->qvel[5]) : 0.f;
        const float base_ang_vel_x = this->robot_state.imu.gyroscope.size() > 0 ? this->robot_state.imu.gyroscope[0] : 0.f;
        const float base_ang_vel_y = this->robot_state.imu.gyroscope.size() > 1 ? this->robot_state.imu.gyroscope[1] : 0.f;
        const float base_ang_vel_z = this->robot_state.imu.gyroscope.size() > 2 ? this->robot_state.imu.gyroscope[2] : 0.f;

        this->csv_ofs << t << "," << this->csv_segment_name << "," << this->control.x << "," << this->control.y << "," << this->control.yaw;
        const auto joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
        const int dof_count = std::min(12, this->params.Get<int>("num_of_dofs"));
        for (int i = 0; i < 12; ++i)
        {
            this->csv_ofs
                << "," << this->robot_state.motor_state.q[i]
                << "," << this->CommandQWithDeployOffset(&this->robot_command, i);
        }
        for (int i = 0; i < 12; ++i)
        {
            float qd = 0.f;
            float tau_pd = 0.f;
            float ctrl = 0.f;
            float actuator_force = 0.f;
            float qfrc_actuator = 0.f;
            float qfrc_passive = 0.f;
            float qfrc_constraint = 0.f;
            float tau_limit = 0.f;
            float tau_clip = 0.f;
            if (i < dof_count && i < static_cast<int>(joint_mapping.size()))
            {
                qd = this->robot_state.motor_state.dq[i];
                const float q_des = this->CommandQWithDeployOffset(&this->robot_command, i);
                tau_pd = this->robot_command.motor_command.tau[i]
                       + this->robot_command.motor_command.kp[i] * (q_des - this->robot_state.motor_state.q[i])
                       + this->robot_command.motor_command.kd[i] * (this->robot_command.motor_command.dq[i] - this->robot_state.motor_state.dq[i]);
                tau_limit = this->TorqueLimit(i);
                tau_clip = std::abs(tau_pd) > tau_limit + 1.0e-4f ? 1.f : 0.f;
                const int actuator_id = joint_mapping[i];
                if (this->mj_data && this->mj_model && actuator_id >= 0 && actuator_id < this->mj_model->nu)
                {
                    ctrl = static_cast<float>(this->mj_data->ctrl[actuator_id]);
                    actuator_force = static_cast<float>(this->mj_data->actuator_force[actuator_id]);
                    const int joint_id = this->mj_model->actuator_trnid[2 * actuator_id];
                    if (joint_id >= 0 && joint_id < this->mj_model->njnt)
                    {
                        const int qfrc_id = this->mj_model->jnt_dofadr[joint_id];
                        if (qfrc_id >= 0 && qfrc_id < this->mj_model->nv)
                        {
                            qfrc_actuator = static_cast<float>(this->mj_data->qfrc_actuator[qfrc_id]);
                            qfrc_passive = static_cast<float>(this->mj_data->qfrc_passive[qfrc_id]);
                            qfrc_constraint = static_cast<float>(this->mj_data->qfrc_constraint[qfrc_id]);
                        }
                    }
                }
            }
            this->csv_ofs << "," << qd
                          << "," << tau_pd
                          << "," << ctrl
                          << "," << actuator_force
                          << "," << qfrc_actuator
                          << "," << qfrc_passive
                          << "," << qfrc_constraint
                          << "," << tau_limit
                          << "," << tau_clip;
        }
        this->csv_ofs << "," << base_roll << "," << base_pitch << "," << base_yaw << "," << base_z << "," << base_height
                      << "," << base_height_ground << "," << stance_foot_z_mean
                      << "," << base_vel_x << "," << base_vel_y << "," << base_yaw_vel
                      << "," << base_ang_vel_x << "," << base_ang_vel_y << "," << base_ang_vel_z;
        for (const auto &pos : foot_pos)
        {
            this->csv_ofs << "," << pos[0] << "," << pos[1] << "," << pos[2];
        }
        std::array<float, 4> foot_speed_xy = {0.f, 0.f, 0.f, 0.f};
        for (size_t leg = 0; leg < foot_vel.size(); ++leg)
        {
            foot_speed_xy[leg] = std::sqrt(foot_vel[leg][0] * foot_vel[leg][0] + foot_vel[leg][1] * foot_vel[leg][1]);
            this->csv_ofs << "," << foot_vel[leg][0] << "," << foot_vel[leg][1] << "," << foot_vel[leg][2];
        }
        for (float speed_xy : foot_speed_xy)
        {
            this->csv_ofs << "," << speed_xy;
        }
        for (int is_contact : contact)
        {
            this->csv_ofs << "," << is_contact;
        }
        for (float z_pos : target_foot_z)
        {
            this->csv_ofs << "," << z_pos;
        }
        for (size_t leg = 0; leg < contact.size(); ++leg)
        {
            const bool target_swing = target_foot_z[leg] > stance_foot_z_mean + 0.02f;
            this->csv_ofs << "," << (target_swing && contact[leg] ? 1 : 0);
        }
        for (size_t leg = 0; leg < contact.size(); ++leg)
        {
            this->csv_ofs << "," << (contact[leg] ? foot_speed_xy[leg] : 0.f);
        }
        for (float air_time : this->foot_air_time_)
        {
            this->csv_ofs << "," << air_time;
        }
        this->csv_ofs << "\n";
        if (this->replay_qdes_enabled_)
        {
            this->csv_ofs.flush();
        }
    }
    if (!this->replay_qdes_enabled_)
    {
        this->ApplyDogUsbControl(true);
        this->StateController(&this->robot_state, &this->robot_command);
        this->ApplyDogUsbControl(false);
    }

    if (this->control.current_keyboard == Input::Keyboard::R || this->control.current_gamepad == Input::Gamepad::RB_Y)
    {
        if (this->mj_model && this->mj_data)
        {
            if (this->mj_model->nkey > 0)
            {
                mj_resetDataKeyframe(this->mj_model, this->mj_data, 0);
            }
            else
            {
                mj_resetData(this->mj_model, this->mj_data);
            }
            mj_forward(this->mj_model, this->mj_data);
        }
    }
    if (this->control.current_keyboard == Input::Keyboard::Enter || this->control.current_gamepad == Input::Gamepad::RB_X)
    {
        if (simulation_running)
        {
            if (sim) { sim->run = 0; }
            std::cout << std::endl << LOGGER::INFO << "Simulation Stop" << std::endl;
        }
        else
        {
            if (sim) { sim->run = 1; }
            std::cout << std::endl << LOGGER::INFO << "Simulation Start" << std::endl;
        }
        simulation_running = !simulation_running;
    }

    this->control.ClearInput();

    this->SetCommand(&this->robot_command);
}

void RL_Sim::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }

    this->sys_js_max_value = (1 << (bits - 1));
}

void RL_Sim::GetSysJoystick()
{
    // Clear all button event states
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }

    // Check if joystick is valid before using
    if (!this->sys_js)
    {
        return;
    }
    if (this->DogUsbShouldBlockFallback())
    {
        return;
    }

    while (this->sys_js->sample(&this->sys_js_event))
    {
        if (this->sys_js_event.isButton())
        {
            this->sys_js_button[this->sys_js_event.number].update(this->sys_js_event.value);
        }
        else if (this->sys_js_event.isAxis())
        {
            double normalized = double(this->sys_js_event.value) / this->sys_js_max_value;
            if (std::abs(normalized) < this->axis_deadzone)
            {
                this->sys_js_axis[this->sys_js_event.number] = 0;
            }
            else
            {
                this->sys_js_axis[this->sys_js_event.number] = this->sys_js_event.value;
            }
        }
    }

    if (this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::A);
    if (this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::B);
    if (this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::X);
    if (this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::Y);
    if (this->sys_js_button[4].on_press) this->control.SetGamepad(Input::Gamepad::LB);
    if (this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::RB);
    if (this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LStick);
    if (this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RStick);
    if (this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::DPadUp);
    if (this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::DPadDown);
    if (this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::DPadLeft);
    if (this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::LB_A);
    if (this->sys_js_button[4].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::LB_B);
    if (this->sys_js_button[4].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::LB_X);
    if (this->sys_js_button[4].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::LB_Y);
    if (this->sys_js_button[4].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::LB_LStick);
    if (this->sys_js_button[4].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::LB_RStick);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadUp);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadDown);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::LB_DPadRight);
    if (this->sys_js_button[4].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::LB_DPadLeft);
    if (this->sys_js_button[5].pressed && this->sys_js_button[0].on_press) this->control.SetGamepad(Input::Gamepad::RB_A);
    if (this->sys_js_button[5].pressed && this->sys_js_button[1].on_press) this->control.SetGamepad(Input::Gamepad::RB_B);
    if (this->sys_js_button[5].pressed && this->sys_js_button[2].on_press) this->control.SetGamepad(Input::Gamepad::RB_X);
    if (this->sys_js_button[5].pressed && this->sys_js_button[3].on_press) this->control.SetGamepad(Input::Gamepad::RB_Y);
    if (this->sys_js_button[5].pressed && this->sys_js_button[9].on_press) this->control.SetGamepad(Input::Gamepad::RB_LStick);
    if (this->sys_js_button[5].pressed && this->sys_js_button[10].on_press) this->control.SetGamepad(Input::Gamepad::RB_RStick);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadUp);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[7] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadDown);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] > 0) this->control.SetGamepad(Input::Gamepad::RB_DPadRight);
    if (this->sys_js_button[5].pressed && this->sys_js_axis[6] < 0) this->control.SetGamepad(Input::Gamepad::RB_DPadLeft);
    if (this->sys_js_button[4].pressed && this->sys_js_button[5].on_press) this->control.SetGamepad(Input::Gamepad::LB_RB);

    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);

    if (has_input)
    {
        // 后退限到 -0.7: 后退 motion 只覆盖到 ~-0.77(0p70 档),训练 max_backward_curriculum=0.7。
        // 摇杆后退满偏(ly=-1.0)会超出训练/motion 范围,策略外推导致下蹲、膝盖贴地。
        this->control.x = (ly < -0.7f) ? -0.7f : ly;
        this->control.y = lx;
        this->control.yaw = rx;
        this->sys_js_active = true;
    }
    else if (this->sys_js_active)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        this->sys_js_active = false;
    }
}

void RL_Sim::RunModel()
{
    if (this->rl_init_done && simulation_running)
    {
        ++this->run_model_count_;
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->ApplyDogUsbControl(false);
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
        // 命令统一裁剪到 motion 覆盖范围: 键盘是累加式且无上限(rl_sdk.cpp 每按一次 ±0.1)、摇杆满偏也可能超范围,
        // 超出后 AMP policy 外推会退化(后退下蹲贴地、横移>1.2 变后退)。手柄/键盘/导航三条路径在此统一兜底。
        // vx: 后退 motion 仅到 -0.77、前进到 1.23; vy: 纯横移 motion 仅到 ±0.50; wz: 转向 motion 到 ±1.02
        this->obs.commands[0] = this->obs.commands[0] < -0.7f ? -0.7f : (this->obs.commands[0] > 1.2f ? 1.2f : this->obs.commands[0]);
        this->obs.commands[1] = this->obs.commands[1] < -0.6f ? -0.6f : (this->obs.commands[1] > 0.6f ? 0.6f : this->obs.commands[1]);
        this->obs.commands[2] = this->obs.commands[2] < -1.0f ? -1.0f : (this->obs.commands[2] > 1.0f ? 1.0f : this->obs.commands[2]);
        //not currently available for non-ros mujoco version
        // if (this->control.navigation_mode)
        // {
        //     this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        // }
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        this->obs.actions = this->Forward();
        this->ComputeOutput(this->obs.actions, this->output_dof_pos, this->output_dof_vel, this->output_dof_tau);

        if (!this->output_dof_pos.empty())
        {
            output_dof_pos_queue.push(this->output_dof_pos);
        }
        if (!this->output_dof_vel.empty())
        {
            output_dof_vel_queue.push(this->output_dof_vel);
        }
        if (!this->output_dof_tau.empty())
        {
            output_dof_tau_queue.push(this->output_dof_tau);
        }

        // this->TorqueProtect(this->output_dof_tau);
        // this->AttitudeProtect(this->robot_state.imu.quaternion, 75.0f, 75.0f);

#ifdef CSV_LOGGER
        std::vector<float> tau_est(this->params.Get<int>("num_of_dofs"), 0.0f);
        for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
        {
            tau_est[i] = this->joint_efforts[this->params.Get<std::vector<std::string>>("joint_controller_names")[i]];
        }
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

std::vector<float> RL_Sim::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // If model is being reinitialized, return previous actions to avoid blocking
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (this->params.Get<std::vector<int>>("observations_history").size() != 0)
    {
        this->history_obs_buf.insert(clamped_obs);
        this->history_obs = this->history_obs_buf.get_obs_vec(this->params.Get<std::vector<int>>("observations_history"));
        actions = this->model->forward({this->history_obs});
    }
    else
    {
        actions = this->model->forward({clamped_obs});
    }

    if (!this->params.Get<std::vector<float>>("clip_actions_upper").empty() && !this->params.Get<std::vector<float>>("clip_actions_lower").empty())
    {
        return clamp(actions, this->params.Get<std::vector<float>>("clip_actions_lower"), this->params.Get<std::vector<float>>("clip_actions_upper"));
    }
    else
    {
        return actions;
    }
}

void RL_Sim::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        this->plot_real_joint_pos[i].push_back(mj_data->sensordata[i]);
        // this->plot_target_joint_pos[i].push_back();  // TODO
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.01);
}

// Signal handler for Ctrl+C
void signalHandler(int signum)
{
    std::cout << LOGGER::INFO << "Received signal " << signum << ", exiting..." << std::endl;
    if (RL_Sim::instance && RL_Sim::instance->sim)
    {
        RL_Sim::instance->sim->exitrequest.store(1);
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, signalHandler);
    RL_Sim rl_sar(argc, argv);
    return 0;
}
