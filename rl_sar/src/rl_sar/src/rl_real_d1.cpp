/*
* Copyright (c) 2024-2025 Ziqi Fan
* SPDX-License-Identifier: Apache-2.0
*/
#include "rl_real_d1.hpp"

#if defined(USE_ROS1)
    #include "std_msgs/Float64MultiArray.h"
#elif defined(USE_ROS2)
    #include <std_msgs/msg/float64_multi_array.hpp>
#endif

// 构造实机 D1 控制器。
//
// 这里完成实机节点的主要初始化：创建 ROS 话题、读取 base.yaml、创建状态机、
// 打开 CAN 总线，并启动键盘、手柄、控制和 RL 推理等周期循环。
// 具体运动逻辑由 FSM 和 RL 模块在各自的运行周期中产生。
RL_Real::RL_Real(int argc, char **argv)
{
#if defined(USE_ROS1)
    ros::NodeHandle nh;
    // /cmd_vel 缓存外部速度命令；是否使用该命令由当前控制模式决定。
    this->cmd_vel_subscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &RL_Real::CmdvelCallback, this);
    // IMU 回调只负责缓存最新的姿态和角速度。真正给控制器使用的数据，
    // 会在每个控制周期由 GetState() 从缓存消息里复制出来。
    this->odom_sub_ = nh.subscribe("/imu/data", 1, &RL_Real::OdomCallBack, this);
    joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
#elif defined(USE_ROS2)
    ros2_node = std::make_shared<rclcpp::Node>("rl_real_node");
    // /cmd_vel 缓存外部速度命令；是否使用该命令由当前控制模式决定。
    cmd_vel_subscriber = ros2_node->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&RL_Real::CmdvelCallback, this, std::placeholders::_1));
    // IMU 回调只负责缓存最新的姿态和角速度。真正给控制器使用的数据，
    // 会在每个控制周期由 GetState() 从缓存消息里复制出来。
    odom_sub_ = ros2_node->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 10,
        std::bind(&RL_Real::OdomCallBack, this, std::placeholders::_1));
    joint_state_pub_ = ros2_node->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states", 10);
    dog_usb_enable = ros2_node->declare_parameter<bool>("dog_usb_enable", false);
    dog_usb_device = ros2_node->declare_parameter<std::string>("dog_usb_device", "");
    dog_usb_baud = ros2_node->declare_parameter<int>("dog_usb_baud", 115200);
    dog_usb_timeout_ms = ros2_node->declare_parameter<int>("dog_usb_timeout_ms", 300);
    dog_remote_timeout_ms = ros2_node->declare_parameter<int>("dog_remote_timeout_ms", 1500);
    dog_usb_allow_fallback = ros2_node->declare_parameter<bool>("dog_usb_allow_fallback", false);
    dog_usb_l1_off_exit = ros2_node->declare_parameter<bool>("dog_usb_l1_off_exit", false);
    dog_usb_l1_button_bit = ros2_node->declare_parameter<int>("dog_usb_l1_button_bit", 8);
    dog_usb_l1_exit_timeout_ms = ros2_node->declare_parameter<int>("dog_usb_l1_exit_timeout_ms", 8000);
    keyboard_enable = ros2_node->declare_parameter<bool>("keyboard_enable", true);
    sys_joystick_device = ros2_node->declare_parameter<std::string>("sys_joystick_device", "/dev/input/js0");
    if (dog_usb_timeout_ms < 1) {dog_usb_timeout_ms = 300;}
    if (dog_remote_timeout_ms < 1) {dog_remote_timeout_ms = 1500;}
    if (dog_usb_l1_button_bit < 0 || dog_usb_l1_button_bit > 15) {dog_usb_l1_button_bit = 8;}
    if (dog_usb_l1_exit_timeout_ms < 1) {dog_usb_l1_exit_timeout_ms = 8000;}
    if (this->dog_usb_enable)
    {
        if (this->dog_usb_device.empty())
        {
            std::cout << LOGGER::ERROR
                      << "[dog_usb] dog_usb_enable=true but dog_usb_device is empty; not starting dog USB receiver."
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
#endif
    // Linux 手柄设备会产生按键和摇杆事件。GetSysJoystick() 会把原始事件
    // 转换成统一的 Input::Gamepad 控制输入，供 FSM 和策略控制使用。
    this->SetupSysJoystick(this->sys_joystick_device, 16); //  ls -l /dev/input/js*

    // 创建状态机和控制循环之前，先读取 D1 基础配置。base.yaml 提供控制
    // 周期、固定增益、自由度数量、关节名称、默认关节位置等通用参数。
    this->ang_vel_axis = "body";
    this->robot_name = "d1";
    this->ReadYaml(this->robot_name, "base.yaml");//base.yaml

    // 根据 robot_name 创建机器人专属状态机。后续每个控制周期都会由 FSM
    // 根据当前状态和输入，生成对应的 RobotCommand。
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

    // 按 base.yaml 中的自由度数量分配状态、命令和观测缓存。这里的关节顺序
    // 必须和 joint_names、joint_mapping、default_dof_pos 保持一致。
    this->InitJointNum(this->params.Get<int>("num_of_dofs"));
    this->InitOutputs();
    this->InitControl();
    this->setupImu();
    int num_of_dofs = this->params.Get<int>("num_of_dofs");
    this->joint_mapping = this->params.Get<std::vector<int>>("joint_mapping");
    this->joint_names_ = this->params.Get<std::vector<std::string>>("joint_names");
    
    // 当前实机分支约定 CAN1 是第一组 6 个电机。电机 id 1..6 预期依次对应：
    // FL_hip、FL_thigh、FL_calf、FR_hip、FR_thigh、FR_calf。
    // 实物接线、电机 id 和这里的顺序必须一致。
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x01,.mst_id=0x11});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x02,.mst_id=0x12});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x03,.mst_id=0x13});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x04,.mst_id=0x14});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x05,.mst_id=0x15});
    this->CAN1.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x06,.mst_id=0x16});

    // CAN2 是第二组 6 个电机。CAN2 也使用 1..6 这组电机 id，但它和 CAN1
    // 是不同 SocketCAN 设备，所以同一个 id 会落到另一组物理电机上。
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x01,.mst_id=0x11});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x02,.mst_id=0x12});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x03,.mst_id=0x13});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x04,.mst_id=0x14});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x05,.mst_id=0x15});
    this->CAN2.push_back(damiao::DmActData{.motorType = damiao::DM6248P,.mode = damiao::MIT_MODE,.can_id=0x06,.mst_id=0x16});
    
    // Motor_Control 会打开 SocketCAN，并维护该总线下所有电机的收发状态。
    // 当前分支按 Jetson 实机部署方式使用 can1/can2，而不是 can0/can1。
    this->motorsInterface = std::make_shared<damiao::Motor_Control>("can1",&this->CAN1,damiao::canfd);
    this->motorsInterface2 = std::make_shared<damiao::Motor_Control>("can2",&this->CAN2,damiao::canfd);

    // 不要在主控制节点里自动写零点。零点标定是独立且有风险的硬件操作，
    // 应该使用 set_zero_all 或 set_zero_one 工具，在吊起机器人并确认姿态后执行。
    // for (int i = 1; i < 7; ++i) {
    // this->motorsInterface->set_zero_position(*this->motorsInterface->getMotor(i));
    // }
    // for (int i = 1; i < 7; ++i) {
    // this->motorsInterface2->set_zero_position(*this->motorsInterface2->getMotor(i));
    // }

    // 启动周期线程。loop_control 是真正的电机闭环控制线程；base.yaml 中
    // dt=0.005 时，RobotControl() 以 200 Hz 运行。
    this->loop_control = std::make_shared<LoopFunc>("loop_control", this->params.Get<float>("dt"), std::bind(&RL_Real::RobotControl, this));
    // loop_rl 只在 rl_init_done 为 true 时产生策略输出；是否置位由 FSM
    // 中的具体状态决定。
    this->loop_rl = std::make_shared<LoopFunc>("loop_rl", this->params.Get<float>("dt") * this->params.Get<int>("decimation"), std::bind(&RL_Real::RunModel, this));
    if (this->keyboard_enable)
    {
        this->loop_keyboard = std::make_shared<LoopFunc>("loop_keyboard", 0.05, std::bind(&RL_Real::KeyboardInterface, this));
        this->loop_keyboard->start();
    }
    else
    {
        std::cout << LOGGER::INFO << "Keyboard input disabled by parameter keyboard_enable=false" << std::endl;
    }
    this->loop_control->start();
    this->loop_rl->start();

    // 手柄轮询频率高于主控制频率，避免很短的按键动作被控制线程漏掉。
    this->loop_joystick = std::make_shared<LoopFunc>("loop_joystick", 0.01, std::bind(&RL_Real::GetSysJoystick, this));
    this->loop_joystick->start();
#ifdef PLOT
    this->plot_t = std::vector<int>(this->plot_size, 0);
    this->plot_real_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    this->plot_target_joint_pos.resize(this->params.Get<int>("num_of_dofs"));
    for (auto &vector : this->plot_real_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    for (auto &vector : this->plot_target_joint_pos) { vector = std::vector<float>(this->plot_size, 0); }
    this->loop_plot = std::make_shared<LoopFunc>("loop_plot", 0.002, std::bind(&RL_Real::Plot, this));
    this->loop_plot->start();
#endif
#ifdef CSV_LOGGER
    this->CSVInit(this->robot_name);
#endif
}

RL_Real::~RL_Real()
{
    if (this->loop_keyboard) {this->loop_keyboard->shutdown();}
    if (this->loop_control) {this->loop_control->shutdown();}
    if (this->loop_rl) {this->loop_rl->shutdown();}
    if (this->loop_joystick) {this->loop_joystick->shutdown();}
    if (this->dog_usb_receiver)
    {
        this->dog_usb_receiver->Stop();
    }
#ifdef PLOT
    this->loop_plot->shutdown();
#endif
    std::cout << LOGGER::INFO << "RL_Real exit------!!!!!!!!!" << std::endl;
}

// 将最新硬件状态复制到控制器状态结构体中。
//
// 状态机和策略网络使用的是 base.yaml 中定义的模型关节顺序，而不是电机原始
// 安装方向。每个电机读数都要乘以 directionMotor_* 中对应的符号，这样后续
// 控制逻辑看到的 q/dq/tau_est 才是模型坐标系下的关节量。
void RL_Real::GetState(RobotState<float> *state)
{
    // yesenceIMU_ 由 OdomCallBack() 异步更新。这里在每个控制周期采样一次
    // 缓存值，供 FSM 和策略观测使用。
    state->imu.quaternion[0] = this->yesenceIMU_.orientation.w; // w
    state->imu.quaternion[1] = this->yesenceIMU_.orientation.x; // x
    state->imu.quaternion[2] = this->yesenceIMU_.orientation.y; // y
    state->imu.quaternion[3] = this->yesenceIMU_.orientation.z; // z
    state->imu.gyroscope[0] = this->yesenceIMU_.angular_velocity.x;
    state->imu.gyroscope[1] = this->yesenceIMU_.angular_velocity.y;
    state->imu.gyroscope[2] = this->yesenceIMU_.angular_velocity.z;

    // 读取 CAN1/前腿电机。CAN1_MAP 把控制器数组下标映射到物理 CAN id；
    // directionMotor_Front 把电机坐标系转换成控制器使用的关节坐标系。
    for (int i = 0; i < 6; ++i) {
        int read_id = CAN1_MAP[i];
        state->motor_state.q[i] =this->motorsInterface->getMotor(read_id)->Get_Position() * directionMotor_Front[i];
        state->motor_state.dq[i]=this->motorsInterface->getMotor(read_id)->Get_Velocity() * directionMotor_Front[i];
        state->motor_state.tau_est[i]=this->motorsInterface->getMotor(read_id)->Get_tau() * directionMotor_Front[i];
    }

    // 读取 CAN2/后腿电机，并写入 12 自由度向量的下标 6..11。控制器看到的
    // 始终是一条连续的关节向量：FL、FR、RL、RR。
    for (int i = 0; i < 6; ++i) {
        int i_cout   = i + 6;
        int read_id = CAN2_MAP[i];
        state->motor_state.q[i_cout] =this->motorsInterface2->getMotor(read_id)->Get_Position() * directionMotor_Back[i];
        state->motor_state.dq[i_cout]=this->motorsInterface2->getMotor(read_id)->Get_Velocity() * directionMotor_Back[i];
        state->motor_state.tau_est[i_cout]=this->motorsInterface2->getMotor(read_id)->Get_tau() * directionMotor_Back[i];
    }
    #if defined(USE_ROS1)
        std_msgs::Float64MultiArray time_msg;
        std_msgs::Float64MultiArray time_msg2;
    #elif defined(USE_ROS2)
        std_msgs::msg::Float64MultiArray time_msg;
        std_msgs::msg::Float64MultiArray time_msg2;
    #endif
    time_msg.data.resize(6);
    time_msg2.data.resize(6);
    for (size_t i = 0; i < 6; ++i)
    {
        time_msg.data[i] = motorsInterface->getMotor(i+1)->getTimeInterval();
    }
    for (size_t i = 0; i < 6; ++i)
    {
        time_msg2.data[i] = motorsInterface2->getMotor(i+1)->getTimeInterval();
    }

    // 下面保留的打印语句适合实机 bring-up 时临时打开。它们打印的是乘过
    // directionMotor_* 之后的关节位置，也就是控制器和 FSM 真正看到的 q。
    // std::cerr<<"time_interval:   "<<time_msg.data[0]
    //         <<"  time_interval(s): "<<time_msg2.data[0]<<std::endl;
    // std::cerr<<"pos1: "<<state->motor_state.q[0]<<
    //     "     pos2: "<<state->motor_state.q[1]<<
    //     "     pos3: "<<state->motor_state.q[2]<<
    //     "     pos4: "<<state->motor_state.q[3]<<
    //     "     pos5: "<<state->motor_state.q[4]<<
    //     "     pos6: "<<state->motor_state.q[5]<<std::endl;
    // std::cerr<<"pos1: "<<state->motor_state.q[6]<<
    //     "     pos2: "<<state->motor_state.q[7]<<
    //     "     pos3: "<<state->motor_state.q[8]<<
    //     "     pos4: "<<state->motor_state.q[9]<<
    //     "     pos5: "<<state->motor_state.q[10]<<
    //     "     pos6: "<<state->motor_state.q[11]<<std::endl;
    // std::cerr<<"w2: "<<state->imu.quaternion[0]<<"  "
    //     <<"x2: "<<state->imu.quaternion[1]<<"  "
    //     <<"y2: "<<state->imu.quaternion[2]<<"  "
    //     <<"z2: "<<state->imu.quaternion[3]<<"  "<<std::endl;
    // std::cerr<<"x2: "<<state->imu.gyroscope[0]<<"  "
    //         <<"y2: "<<state->imu.gyroscope[1]<<"  "
    //         <<"z2: "<<state->imu.gyroscope[2]<<"  "<<std::endl;
}

// 将控制器命令发送到两条达妙 CAN 总线。
//
// RobotCommand 使用控制器/模型坐标系表达。真正发给电机之前，要按电机安装
// 方向乘回 directionMotor_*，这和 GetState() 中的方向转换互为对应。
// 命令来源由当前 FSM 状态决定，可能是固定位置控制，也可能是策略输出。
void RL_Real::SetCommand(const RobotCommand<float> *command)
{
    // 先缓存完整 12 自由度命令，再分别下发到 CAN1/CAN2。这样可以保持总线
    // 发送逻辑简单，也方便后续增加 target q/kp/kd 的调试打印。
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
       rl_date[i].pos_des_ = command->motor_command.q[i];
       rl_date[i].vel_des_ = command->motor_command.dq[i];
       rl_date[i].ff_ = command->motor_command.tau[i];
       rl_date[i].kp_ = command->motor_command.kp[i];
       rl_date[i].kd_ = command->motor_command.kd[i];
    //    std::cerr<<"pos: "<<i<<" "<<rl_date[i].pos_des_<<std::endl;
    }

    // 发送 CAN1/前腿命令。最终发给电机的 q/dq/tau 符号必须匹配实际电机
    // 安装方向，而不是模型坐标系的符号。
    for (int i = 0; i < 6; ++i) {
        int rl_idx = CAN1_MAP[i];
        dmSendcmd_[rl_idx].pos_des_ = rl_date[i].pos_des_ * directionMotor_Front[i];
        dmSendcmd_[rl_idx].vel_des_ = rl_date[i].vel_des_ * directionMotor_Front[i];
        dmSendcmd_[rl_idx].ff_ = rl_date[i].ff_ * directionMotor_Front[i];
        dmSendcmd_[rl_idx].kp_ = rl_date[i].kp_;
        dmSendcmd_[rl_idx].kd_ = rl_date[i].kd_;
        this->motorsInterface->control_mit(*this->motorsInterface->getMotor(rl_idx),dmSendcmd_[rl_idx].kp_,dmSendcmd_[rl_idx].kd_ ,
                                            dmSendcmd_[rl_idx].pos_des_,dmSendcmd_[rl_idx].vel_des_, dmSendcmd_[rl_idx].ff_);
        // this->motorsInterface->control_mit(*this->motorsInterface->getMotor(rl_idx), 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    // 发送 CAN2/后腿命令。i_cout 把第二条 6 电机总线的局部下标，转换成
    // 12 自由度命令向量中的全局下标。
    for (int i = 0; i < 6; ++i) {
        int i_cout   = i + 6;
        int rl_idx = CAN2_MAP[i];
        dmSendcmd2_[rl_idx].pos_des_ = rl_date[i_cout].pos_des_ * directionMotor_Back[i];
        dmSendcmd2_[rl_idx].vel_des_ = rl_date[i_cout].vel_des_ * directionMotor_Back[i];
        dmSendcmd2_[rl_idx].ff_ = rl_date[i_cout].ff_ * directionMotor_Back[i];
        dmSendcmd2_[rl_idx].kp_ = rl_date[i_cout].kp_;
        dmSendcmd2_[rl_idx].kd_ = rl_date[i_cout].kd_;
        this->motorsInterface2->control_mit(*this->motorsInterface2->getMotor(rl_idx),dmSendcmd2_[rl_idx].kp_,dmSendcmd2_[rl_idx].kd_ ,
                                            dmSendcmd2_[rl_idx].pos_des_,dmSendcmd2_[rl_idx].vel_des_, dmSendcmd2_[rl_idx].ff_);
        // this->motorsInterface2->control_mit(*this->motorsInterface2->getMotor(rl_idx), 0.0, 0.0, 0.0, 0.0, 0.0);
    }
}

// 打开 Linux 手柄设备，并记录摇杆原始数值范围。
//
// 如果没有检测到手柄，节点不会直接退出。这样在台架调试时仍然可以用键盘
// 或 ROS 侧命令继续测试。
void RL_Real::SetupSysJoystick(const std::string& device, int bits)
{
    this->sys_js = std::make_unique<Joystick>(device);
    if (!this->sys_js->isFound())
    {
        std::cout << LOGGER::ERROR << "Joystick [" << device << "] open failed." << std::endl;
        // exit(1);
    }
    this->sys_js_max_value = (1 << (bits - 1));
}

// 轮询所有待处理的手柄事件，并转换为控制器输入。
//
// Button::on_press 是边沿触发信号：只有按键从松开变成按下的那个轮询周期
// 才为 true。主控制循环运行完 FSM 后会清理这些一次性输入，所以按一次 A
// 只会请求一次状态切换，不会在后续周期里反复触发。
void RL_Real::GetSysJoystick()
{
    // 读取新事件前，先清空上一轮的一次性按下/松开事件。
    for (int i = 0; i < 20; ++i)
    {
        this->sys_js_button[i].on_press = false;
        this->sys_js_button[i].on_release = false;
    }
    // 手柄未打开时直接返回，避免空指针访问。
    if (!this->sys_js) {return;}
    if (this->DogUsbShouldBlockFallback()) {return;}
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

    // 将物理按键映射成统一的 Gamepad 枚举。FSM 只关心这些抽象输入，
    // 不直接依赖 Linux joystick 的原始按钮编号。
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

    // 摇杆轴值会被转换成期望机体速度命令。具体状态可以自行决定是否使用
    // control.x、control.y 和 control.yaw。
    float ly = -float(this->sys_js_axis[1]) / float(this->sys_js_max_value);
    float lx = -float(this->sys_js_axis[0]) / float(this->sys_js_max_value);
    float rx = -float(this->sys_js_axis[3]) / float(this->sys_js_max_value);

    bool has_input = (ly != 0.0f || lx != 0.0f || rx != 0.0f);
    if (has_input)
    {
        // 后退限到 -0.6(对齐训练 max_backward_curriculum): 后退 motion 只覆盖到 ~-0.77,
        // 摇杆后退满偏会超出范围,策略外推导致下蹲、膝盖贴地。
        this->control.x = (ly < -0.6f) ? -0.6f : ly;
        this->control.y = lx;
        this->control.yaw = rx*0.5;
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

// 执行一次实时控制周期。
//
// 这是实机机器人最核心的闭环路径：
//   1. GetState() 读取 IMU 和电机反馈。
//   2. StateController() 让 FSM 根据当前状态生成下一帧 RobotCommand。
//   3. ClearInput() 清理一次性按键，避免同一次按键被重复消费。
//   4. SetCommand() 把 RobotCommand 转换成 CAN MIT 命令并发给电机。
bool RL_Real::DogUsbShouldBlockFallback() const
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

void RL_Real::TriggerDogUsbGamepad(Input::Gamepad gamepad, const char* name)
{
    this->control.SetGamepad(gamepad);
    std::cout << LOGGER::INFO << "[dog_usb] event " << name << std::endl;
}

void RL_Real::RequestDogUsbExit(const char* reason)
{
    if (this->dog_usb_exit_requested)
    {
        return;
    }

    this->dog_usb_exit_requested = true;
    this->dog_usb_exit_shutdown_after_command = false;
    this->dog_usb_exit_started_at = std::chrono::steady_clock::now();
    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;

    std::cout << LOGGER::WARNING << "[dog_usb] " << reason
              << "; requesting getdown then rl_real_d1 shutdown" << std::endl;
}

std::string RL_Real::CurrentFsmStateName() const
{
    if (!this->fsm.current_state_)
    {
        return "";
    }
    return this->fsm.current_state_->GetStateName();
}

bool RL_Real::DogUsbExitTimedOut() const
{
    if (!this->dog_usb_exit_requested)
    {
        return false;
    }

    const auto elapsed = std::chrono::steady_clock::now() - this->dog_usb_exit_started_at;
    return elapsed >= std::chrono::milliseconds(this->dog_usb_l1_exit_timeout_ms);
}

void RL_Real::HoldDogUsbExitInputs()
{
    if (!this->dog_usb_exit_requested)
    {
        return;
    }

    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;
    this->control.current_keyboard = Input::Keyboard::None;
    this->control.last_keyboard = Input::Keyboard::None;
    this->control.current_gamepad = Input::Gamepad::None;

    const std::string state_name = this->CurrentFsmStateName();
    if (state_name == "RLFSMStateGetUp" || state_name == "RLFSMStateRLLocomotion")
    {
        this->control.SetGamepad(Input::Gamepad::B);
    }
}

void RL_Real::UpdateDogUsbExitShutdown()
{
    if (!this->dog_usb_exit_requested)
    {
        return;
    }

    this->control.x = 0.0f;
    this->control.y = 0.0f;
    this->control.yaw = 0.0f;

    const std::string state_name = this->CurrentFsmStateName();
    if (state_name == "RLFSMStatePassive")
    {
        this->dog_usb_exit_shutdown_after_command = true;
        std::cout << LOGGER::INFO
                  << "[dog_usb] L1 OFF exit reached passive; shutting down after this command"
                  << std::endl;
        return;
    }

    if (this->DogUsbExitTimedOut())
    {
        this->dog_usb_exit_shutdown_after_command = true;
        std::cout << LOGGER::WARNING
                  << "[dog_usb] L1 OFF exit timeout in state " << state_name
                  << "; forcing rl_real_d1 shutdown" << std::endl;
    }
}

void RL_Real::ShutdownDogUsbExitIfReady()
{
    if (!this->dog_usb_exit_shutdown_after_command)
    {
        return;
    }

    this->dog_usb_exit_shutdown_after_command = false;

#if defined(USE_ROS1)
    ros::shutdown();
#elif defined(USE_ROS2)
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
#endif
}

void RL_Real::LogDogUsbStatus(bool safe_state, bool usb_timeout, bool remote_timeout, bool serial_connected)
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

void RL_Real::ApplyDogUsbControl(bool emit_events)
{
    if (this->dog_usb_exit_requested)
    {
        this->control.x = 0.0f;
        this->control.y = 0.0f;
        this->control.yaw = 0.0f;
        return;
    }

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
        this->control.x = state.x < -0.6f ? -0.6f : static_cast<float>(state.x);
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

    const uint16_t kBtnSwL1 = static_cast<uint16_t>(1u << this->dog_usb_l1_button_bit);
    constexpr uint16_t kBtnSwL2 = 1u << 9;
    constexpr uint16_t kBtnSwR1 = 1u << 10;
    constexpr uint16_t kBtnSwR2 = 1u << 11;

    const uint16_t buttons = state.buttons;
    const uint16_t prev_buttons = this->dog_usb_prev_buttons;

    if (this->dog_usb_l1_off_exit)
    {
        const bool l1_now = (buttons & kBtnSwL1) != 0;
        const bool l1_prev = (prev_buttons & kBtnSwL1) != 0;
        if (l1_now)
        {
            if (!this->dog_usb_l1_seen_on)
            {
                std::cout << LOGGER::INFO
                          << "[dog_usb] L1 ON armed for OFF exit, buttons=0x"
                          << std::hex << buttons << std::dec
                          << " remote_age_ms=" << state.last_remote_age_ms
                          << std::endl;
            }
            this->dog_usb_l1_seen_on = true;
        }
        else if (this->dog_usb_l1_seen_on && l1_prev)
        {
            std::cout << LOGGER::WARNING
                      << "[dog_usb] L1 OFF edge detected, buttons=0x"
                      << std::hex << buttons
                      << " prev=0x" << prev_buttons << std::dec
                      << " remote_age_ms=" << state.last_remote_age_ms
                      << std::endl;
            this->RequestDogUsbExit("L1_OFF_Exit");
        }
    }

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

void RL_Real::RobotControl()
{
    this->GetState(&this->robot_state);
    this->ApplyDogUsbControl(true);
    this->HoldDogUsbExitInputs();
    this->StateController(&this->robot_state, &this->robot_command);
    this->ApplyDogUsbControl(false);
    this->UpdateDogUsbExitShutdown();
    this->control.ClearInput();
    this->SetCommand(&this->robot_command);
    this->ShutdownDogUsbExitIfReady();

    // joint_state_msg_.header.stamp = this->now();
    // joint_state_msg_.name = joint_names_;
    // joint_state_msg_.position.resize(12);
    // joint_state_msg_.velocity.resize(12);
    // for (int i = 0; i < 12; ++i)
    // {
    //     joint_state_msg_.position[i] = robot_state.motor_state.q[i];
    //     joint_state_msg_.velocity[i] = robot_command.motor_command.q[i];
    // }
    // joint_state_pub_->publish(joint_state_msg_);
}

// 执行神经网络策略推理循环。
//
// 这个循环和 200 Hz 电机控制循环是分开的。只有相关 FSM 状态完成策略
// 初始化并将 rl_init_done 置为 true 后，RunModel() 才会真正推理。
void RL_Real::RunModel()
{
    if (this->rl_init_done)
    {
        this->episode_length_buf += 1;
        this->obs.ang_vel = this->robot_state.imu.gyroscope;
        this->ApplyDogUsbControl(false);
        this->obs.commands = {this->control.x, this->control.y, this->control.yaw};
#if !defined(USE_CMAKE)
        if (this->control.navigation_mode)
        {
            this->obs.commands = {(float)this->cmd_vel.linear.x, (float)this->cmd_vel.linear.y, (float)this->cmd_vel.angular.z};
        }
#endif
        // 命令统一裁剪到 motion 覆盖范围(键盘累加无上限/摇杆满偏都会超范围,
        // 超出后 AMP policy 外推退化: 后退下蹲、横移>1.2变后退)。
        // vx 后退motion到-0.77,前进到1.0; vy 纯横移motion仅到0.50; wz 转向motion到1.02
        this->obs.commands[0] = this->obs.commands[0] < -0.6f ? -0.6f : (this->obs.commands[0] > 1.0f ? 1.0f : this->obs.commands[0]);
        this->obs.commands[1] = this->obs.commands[1] < -0.6f ? -0.6f : (this->obs.commands[1] > 0.6f ? 0.6f : this->obs.commands[1]);
        this->obs.commands[2] = this->obs.commands[2] < -1.0f ? -1.0f : (this->obs.commands[2] > 1.0f ? 1.0f : this->obs.commands[2]);
        this->obs.base_quat = this->robot_state.imu.quaternion;
        this->obs.dof_pos = this->robot_state.motor_state.q;
        this->obs.dof_vel = this->robot_state.motor_state.dq;

        // Forward() 返回策略网络的原始 action。ComputeOutput() 再根据
        // config.yaml 中的 action_scale、default_dof_pos、rl_kp/rl_kd 等参数，
        // 转换成目标关节位置、目标关节速度和估算力矩。
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

#ifdef CSV_LOGGER
        std::vector<float> tau_est = this->robot_state.motor_state.tau_est;
        this->CSVLogger(this->output_dof_tau, tau_est, this->obs.dof_pos, this->output_dof_pos, this->obs.dof_vel);
#endif
    }
}

// 执行当前已加载策略模型的前向推理。
//
// 部署策略可能只需要当前观测，也可能需要一段历史观测；具体由
// config.yaml/observations_history 决定。model_mutex 用来避免推理过程和
// 模型重新初始化过程并发访问同一个模型对象。
std::vector<float> RL_Real::Forward()
{
    std::unique_lock<std::mutex> lock(this->model_mutex, std::try_to_lock);

    // 如果模型正在重新初始化，本轮不阻塞控制线程，直接复用上一帧 action。
    if (!lock.owns_lock())
    {
        std::cout << LOGGER::WARNING << "Model is being reinitialized, using previous actions" << std::endl;
        return this->obs.actions;
    }

    std::vector<float> clamped_obs = this->ComputeObservation();

    std::vector<float> actions;
    if (!this->params.Get<std::vector<int>>("observations_history").empty())
    {
        // HIMLoco 风格策略使用短时间历史观测，让模型从最近几帧机器人响应中
        // 推断速度和隐含动力学状态。
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

// 更新可选的关节曲线窗口。
//
// 只有定义 PLOT 宏时这段逻辑才会编译进来。matplotlib 调用不适合实时控制，
// 所以正常实机运行时应保持关闭。
void RL_Real::Plot()
{
    this->plot_t.erase(this->plot_t.begin());
    this->plot_t.push_back(this->motiontime);
    plt::cla();
    plt::clf();
    for (int i = 0; i < this->params.Get<int>("num_of_dofs"); ++i)
    {
        this->plot_real_joint_pos[i].erase(this->plot_real_joint_pos[i].begin());
        this->plot_target_joint_pos[i].erase(this->plot_target_joint_pos[i].begin());
        // this->plot_real_joint_pos[i].push_back(this->unitree_low_state.motorState[i].q);
        // this->plot_target_joint_pos[i].push_back(this->unitree_low_command.motorCmd[i].q);
        plt::subplot(this->params.Get<int>("num_of_dofs"), 1, i + 1);
        plt::named_plot("_real_joint_pos", this->plot_t, this->plot_real_joint_pos[i], "r");
        plt::named_plot("_target_joint_pos", this->plot_t, this->plot_target_joint_pos[i], "b");
        plt::xlim(this->plot_t.front(), this->plot_t.back());
    }
    // plt::legend();
    plt::pause(0.0001);
}

// 初始化缓存 IMU 数据结构中的固定协方差数值。
bool RL_Real::setupImu() {
  this->imuData_.ori_cov[0] = 0.0012;
  this->imuData_.ori_cov[4] = 0.0012;
  this->imuData_.ori_cov[8] = 0.0012;
  this->imuData_.angular_vel_cov[0] = 0.0004;
  this->imuData_.angular_vel_cov[4] = 0.0004;
  this->imuData_.angular_vel_cov[8] = 0.0004;
  return true;
}
#if !defined(USE_CMAKE)

// 缓存最新的导航速度命令。
//
// 该命令只有在 navigation_mode 打开时才会被 RunModel() 使用。手柄输入
// 和其他 FSM 状态不会直接依赖这个回调。
void RL_Real::CmdvelCallback(
#if defined(USE_ROS1)
    const geometry_msgs::Twist::ConstPtr &msg
#elif defined(USE_ROS2)
    const geometry_msgs::msg::Twist::SharedPtr msg
#endif
)
{
    this->cmd_vel = *msg;
}
#endif

#if defined(USE_ROS1)

// 在 ROS1 构建中处理 Ctrl-C，并触发 ros::shutdown()。
void sigintHandler(int)
{
  ros::shutdown();
    exit(0);
}
#endif

// ROS1、ROS2 和非 ROS 构建共用的程序入口。
int main(int argc, char **argv)
{
#if defined(USE_ROS1)
    signal(SIGINT, sigintHandler);
    ros::init(argc, argv, "rl_sar");
    RL_Real rl_sar(argc, argv);
    ros::spin();
#elif defined(USE_ROS2)
    rclcpp::init(argc, argv);
    auto rl_sar = std::make_shared<RL_Real>(argc, argv);
    rclcpp::spin(rl_sar->ros2_node);
    rclcpp::shutdown();
    return 0;
#else
    RL_Real rl_sar(argc, argv);
    while (true) {sleep(10);}
    return 0;
#endif
}
