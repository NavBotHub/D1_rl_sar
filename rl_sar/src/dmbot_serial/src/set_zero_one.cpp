// D1 单电机零点标定工具
//
// 用法：
//   set_zero_one <CAN 接口> <CAN ID>
// 示例：
//   set_zero_one can1 0x02
//   set_zero_one can2 5
//
// 最安全做法：只把目标电机物理接在该总线上，或确保总线上其他电机已经标定
// 完毕、处于失能 / 未动作状态。

#include "dmbot_serial/protocol/damiao.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
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
              << "，析构器将自动失能电机..." << std::endl;
}

bool ask_yes(const std::string& prompt)
{
    std::cout << prompt << "  [y/N]: " << std::flush;
    std::string input;
    if (!std::getline(std::cin, input)) return false;
    return !input.empty() && (input[0] == 'y' || input[0] == 'Y');
}

void print_usage(const char* argv0)
{
    std::cerr << "用法：" << argv0 << " <CAN 接口> <CAN ID>\n"
              << "  CAN 接口   例如 can1 或 can2\n"
              << "  CAN ID     1..6（十进制）或 0x01..0x06（十六进制）\n"
              << "\n示例：\n"
              << "  " << argv0 << " can1 0x02\n"
              << "  " << argv0 << " can2 5\n";
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signalHandler);

    if (argc != 3) {
        print_usage(argv[0]);
        return 1;
    }
    std::string bus_name = argv[1];
    uint16_t can_id = 0;
    try {
        long v = std::stol(argv[2], nullptr, 0);  // base=0 同时支持 0x 前缀和十进制
        if (v < 1 || v > 6) throw std::out_of_range("CAN ID 必须在 1..6 之间");
        can_id = static_cast<uint16_t>(v);
    } catch (const std::exception& e) {
        std::cerr << "[错误] CAN ID 非法：" << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }
    uint16_t mst_id = static_cast<uint16_t>(0x10 + can_id);

    std::cout << "\n############################################\n"
              <<   "#  D1 单电机零点标定工具\n"
              <<   "############################################\n"
              << "  目标总线：  " << bus_name << "\n"
              << "  目标 CAN ID：0x" << std::hex << std::setw(2) << std::setfill('0')
              << can_id << std::dec << std::setfill(' ')
              << "  （十进制 " << can_id << "）\n"
              << "  目标 MST ID：0x" << std::hex << std::setw(2) << std::setfill('0')
              << mst_id << std::dec << std::setfill(' ')
              << "  （十进制 " << mst_id << "）\n"
              << "  电机型号：  DM6248P（MIT 模式）\n";

    std::cout << "\n!! 安全检查清单（任意一条不满足请立即 Ctrl+C）!!\n"
              << "  [1] 该关节自由（或整腿吊起），没有负载\n"
              << "  [2] 该关节已摆到 URDF 机械零位\n"
              << "  [3] " << bus_name << " 处于 UP 状态\n"
              << "  [4] rl_real_d1 / test_motor 没有在运行\n\n";

    if (!ask_yes("是否继续？")) {
        std::cout << "[已取消] 用户放弃。\n";
        return 0;
    }

    try {
        std::vector<damiao::DmActData> init_data;
        init_data.push_back(damiao::DmActData{
            .motorType = damiao::DM6248P,
            .mode      = damiao::MIT_MODE,
            .can_id    = can_id,
            .mst_id    = mst_id,
        });

        std::cout << "\n[1/6] 打开 " << bus_name << "，自动使能电机 " << can_id
                  << "...\n" << std::flush;
        auto bus = std::make_shared<damiao::Motor_Control>(bus_name, &init_data, damiao::canfd);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto motor = bus->getMotor(can_id);
        if (!motor) {
            std::cerr << "[错误] CAN ID " << can_id << " 的电机未能注册成功。\n";
            return 1;
        }

        for (int k = 0; k < 5 && g_running; ++k) {
            bus->refresh_motor_status(*motor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running) return 0;

        std::cout << "\n[2/6] 标定前的位置：  "
                  << std::fixed << std::setprecision(6)
                  << motor->Get_Position() << " rad\n";

        if (!ask_yes("\n姿态正确？现在设置零点？")) {
            std::cout << "[已取消] 用户在零点写入前放弃。\n";
            return 0;
        }

        std::cout << "\n[3/6] 发送 set_zero_position...\n";
        bus->set_zero_position(*motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        for (int k = 0; k < 3; ++k) {
            bus->refresh_motor_status(*motor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[4/6] 零点写入后的位置（RAM）：  "
                  << std::fixed << std::setprecision(6)
                  << motor->Get_Position() << " rad\n";

        if (!ask_yes("\n读数 ≈ 0？  是否写入 FLASH（掉电保持）？")) {
            std::cout << "[已取消] 用户跳过 FLASH 写入，零点仅保存在 RAM。\n";
            return 0;
        }

        std::cout << "\n[5/6] 正在写入 FLASH...\n";
        bus->save_motor_param(*motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::cout << "[6/6] 重新使能并最终验证...\n";
        bus->enable_all(damiao::MIT_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        for (int k = 0; k < 3; ++k) {
            bus->refresh_motor_status(*motor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "FLASH 写入后的位置：  "
                  << std::fixed << std::setprecision(6)
                  << motor->Get_Position() << " rad\n";

        std::cout << "\n[完成] 如果上面读数 ≈ 0，FLASH 写入成功。\n"
                  << "       程序退出时会自动失能电机。\n";

    } catch (const std::exception& e) {
        std::cerr << "\n[错误] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
