// D1 12 电机零点标定工具（双 CAN 总线，一次标全部）
//
// 硬件假设：
//   * can1 = 前 6 电机，CAN ID 0x01..0x06（DM6248P，MIT 模式）
//   * can2 = 后 6 电机，CAN ID 0x01..0x06（DM6248P，MIT 模式）
//
// 全程不调用 control_mit()，只在用户明确按 y 之后才会发
// set_zero_position() 和 save_motor_param()。

#include "dmbot_serial/protocol/damiao.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

std::atomic<bool> g_running(true);

void signalHandler(int signum)
{
    g_running = false;
    std::cerr << "\n[中断] 收到信号 " << signum
              << "，析构器将自动失能所有电机..." << std::endl;
}

const char* CAN1_JOINT_NAMES[6] = {
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
};
const char* CAN2_JOINT_NAMES[6] = {
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
};

bool ask_yes(const std::string& prompt)
{
    std::cout << prompt << "  [y/N]: " << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) return false;
    return !input.empty() && (input[0] == 'y' || input[0] == 'Y');
}

void refresh_all(damiao::Motor_Control& bus)
{
    for (int i = 1; i <= 6; ++i) {
        auto m = bus.getMotor(static_cast<uint16_t>(i));
        if (m) bus.refresh_motor_status(*m);
        usleep(2000);
    }
}

void warmup_read(damiao::Motor_Control& can1,
                 damiao::Motor_Control& can2,
                 int rounds = 5,
                 int per_round_ms = 150)
{
    for (int k = 0; k < rounds && g_running; ++k) {
        refresh_all(can1);
        refresh_all(can2);
        std::this_thread::sleep_for(std::chrono::milliseconds(per_round_ms));
    }
}

void print_positions_table(const std::string& title,
                           damiao::Motor_Control& can1,
                           damiao::Motor_Control& can2)
{
    std::cout << "\n--- " << title << " ---\n";
    std::cout << "     CAN1（前腿）                  |   CAN2（后腿）\n";
    std::cout << "     ---------------------------   |   ---------------------------\n";
    std::cout << std::fixed << std::setprecision(5);
    for (int i = 1; i <= 6; ++i) {
        auto m1 = can1.getMotor(static_cast<uint16_t>(i));
        auto m2 = can2.getMotor(static_cast<uint16_t>(i));
        float p1 = m1 ? m1->Get_Position() : 0.0f;
        float p2 = m2 ? m2->Get_Position() : 0.0f;
        std::cout << "  id=" << i << "  "
                  << std::left << std::setw(9) << CAN1_JOINT_NAMES[i - 1]
                  << " = " << std::right << std::setw(10) << p1
                  << "  |  id=" << i << "  "
                  << std::left << std::setw(9) << CAN2_JOINT_NAMES[i - 1]
                  << " = " << std::right << std::setw(10) << p2
                  << "\n";
    }
    std::cout << std::flush;
}

}  // namespace

int main()
{
    std::signal(SIGINT, signalHandler);

    std::cout << "\n############################################\n";
    std::cout <<   "#  D1 12 电机零点标定工具\n";
    std::cout <<   "############################################\n";
    std::cout << "\n!! 安全检查清单（任意一条不满足请立即 Ctrl+C）!!\n"
              << "  [1] 机器已吊离地面，所有腿无负载\n"
              << "  [2] 12 个关节都已摆到 URDF 机械零位\n"
              << "  [3] can1 和 can2 都处于 UP 状态（ip -brief link 检查）\n"
              << "  [4] rl_real_d1 / test_motor 没有在运行\n";
    std::cout << "\n本工具全程不会调用 control_mit()，流程如下：\n"
              << "  [1/6] 打开 can1 + can2（自动使能电机，但不输出力矩）\n"
              << "  [2/6] 读取当前位置 → 请你确认\n"
              << "  [3/6] 对 12 个电机发送 set_zero_position（仅写入 RAM）\n"
              << "  [4/6] 再次读取位置 → 应全部 ≈ 0\n"
              << "  [5/6] 写入 FLASH（需你确认，掉电保持）\n"
              << "  [6/6] 重新使能并最终验证 → 仍应 ≈ 0\n\n";

    if (!ask_yes("是否继续？")) {
        std::cout << "[已取消] 用户放弃。\n";
        return 0;
    }

    try {
        std::vector<damiao::DmActData> can1_init;
        std::vector<damiao::DmActData> can2_init;
        for (uint16_t id = 1; id <= 6; ++id) {
            can1_init.push_back(damiao::DmActData{
                .motorType = damiao::DM6248P,
                .mode      = damiao::MIT_MODE,
                .can_id    = id,
                .mst_id    = static_cast<uint16_t>(0x10 + id),
            });
            can2_init.push_back(damiao::DmActData{
                .motorType = damiao::DM6248P,
                .mode      = damiao::MIT_MODE,
                .can_id    = id,
                .mst_id    = static_cast<uint16_t>(0x10 + id),
            });
        }

        std::cout << "\n[1/6] 打开 can1，自动使能前 6 个电机...\n" << std::flush;
        auto can1 = std::make_shared<damiao::Motor_Control>("can1", &can1_init, damiao::canfd);
        std::cout << "[1/6] 打开 can2，自动使能后 6 个电机...\n" << std::flush;
        auto can2 = std::make_shared<damiao::Motor_Control>("can2", &can2_init, damiao::canfd);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        warmup_read(*can1, *can2);
        if (!g_running) return 0;

        std::cout << "\n[2/6] 标定前的位置：" << std::endl;
        print_positions_table("标定前（BEFORE）", *can1, *can2);
        std::cout << "\n  说明：这些值是电机在【当前物理姿态】下、相对【之前存的零点】的读数，\n"
                  << "  非零属于正常现象。我们下一步就是把这个姿态重新设为零点。\n"
                  << "  请先确认：12 个值都有正常打印（没有全 0 或全一样），说明 CAN 通信正常。\n";

        if (!ask_yes("\n姿态正确，现在对 12 个电机设置零点？")) {
            std::cout << "[已取消] 用户在零点写入前放弃。\n";
            return 0;
        }

        std::cout << "\n[3/6] 向 12 个电机发送 set_zero_position...\n" << std::flush;
        for (int i = 1; i <= 6; ++i) {
            if (auto m = can1->getMotor(static_cast<uint16_t>(i))) can1->set_zero_position(*m);
            usleep(5000);
        }
        for (int i = 1; i <= 6; ++i) {
            if (auto m = can2->getMotor(static_cast<uint16_t>(i))) can2->set_zero_position(*m);
            usleep(5000);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        warmup_read(*can1, *can2, 3, 150);

        std::cout << "\n[4/6] 零点写入后的位置（仅 RAM 有效，断电会丢）：\n";
        print_positions_table("零点写入后（RAM）", *can1, *can2);

        if (!ask_yes("\n12 个值是否都 ≈ 0？  是否写入 FLASH（掉电保持）？")) {
            std::cout << "[已取消] 用户跳过 FLASH 写入。\n"
                      << "          当前零点仅保存在 RAM，电机下次上电会恢复到旧零点。\n";
            return 0;
        }

        std::cout << "\n[5/6] 正在写入 FLASH（每个电机短暂失能约 110ms × 12 ≈ 1.4s）...\n" << std::flush;
        for (int i = 1; i <= 6; ++i) {
            if (auto m = can1->getMotor(static_cast<uint16_t>(i))) can1->save_motor_param(*m);
        }
        for (int i = 1; i <= 6; ++i) {
            if (auto m = can2->getMotor(static_cast<uint16_t>(i))) can2->save_motor_param(*m);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "\n[6/6] 重新使能并最终验证...\n" << std::flush;
        can1->enable_all(damiao::MIT_MODE);
        can2->enable_all(damiao::MIT_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        warmup_read(*can1, *can2, 3, 150);

        print_positions_table("FLASH 写入后（已验证）", *can1, *can2);

        std::cout << "\n[完成] 如果上面 12 个值都 ≈ 0，零点已成功写入 FLASH。\n"
                  << "       建议稍后给电机断电重启一次，再运行本工具查看 [2/6] 的打印，\n"
                  << "       如果那时候读数仍然 ≈ 0，说明 FLASH 持久化验证通过。\n"
                  << "       程序退出时会自动失能所有电机。\n";

    } catch (const std::exception& e) {
        std::cerr << "\n[错误] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
