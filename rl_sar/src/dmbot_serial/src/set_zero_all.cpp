// D1 12-motor zero-position calibration tool (dual CAN buses, calibrates all at once)
//
// Hardware assumptions:
//   * can1 = front 6 motors, CAN ID 0x01..0x06 (DM6248P, MIT mode)
//   * can2 = rear 6 motors, CAN ID 0x01..0x06 (DM6248P, MIT mode)
//
// The motors remain disabled throughout calibration. Only after the user explicitly
// presses y will the tool send set_zero_position() and save_motor_param().

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
    std::cerr << "\n[Interrupted] received signal " << signum
              << ", the destructor will automatically disable all motors..." << std::endl;
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
    std::cout << "     CAN1 (front legs)                  |   CAN2 (rear legs)\n";
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
    std::cout <<   "#  D1 12-motor zero-position calibration tool\n";
    std::cout <<   "############################################\n";
    std::cout << "\n!! Safety checklist (press Ctrl+C immediately if any item is not satisfied) !!\n"
              << "  [1] The robot is suspended off the ground and all legs are unloaded\n"
              << "  [2] All 12 joints have been moved to the URDF mechanical zero position\n"
              << "  [3] can1 and can2 are both UP (check with ip -brief link)\n"
              << "  [4] rl_real_d1 / test_motor is not running\n";
    std::cout << "\nThis tool keeps all motors disabled; the procedure is:\n"
              << "  [1/6] Open can1 + can2 and explicitly disable all motors\n"
              << "  [2/6] Read current positions -> user confirmation\n"
              << "  [3/6] Send set_zero_position while disabled (RAM only)\n"
              << "  [4/6] Read positions again -> all should be approximately 0\n"
              << "  [5/6] Write to FLASH (requires confirmation; persists across power cycles)\n"
              << "  [6/6] Final verify while disabled -> values should still be approximately 0\n\n";

    if (!ask_yes("Continue?")) {
        std::cout << "[Canceled] User aborted.\n";
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

        std::cout << "\n[1/6] Opening can1 with automatic enable disabled...\n" << std::flush;
        auto can1 = std::make_shared<damiao::Motor_Control>("can1", &can1_init, damiao::canfd, false);
        can1->disable_all();

        std::cout << "[1/6] Opening can2 with automatic enable disabled...\n" << std::flush;
        auto can2 = std::make_shared<damiao::Motor_Control>("can2", &can2_init, damiao::canfd, false);
        can2->disable_all();

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        warmup_read(*can1, *can2);
        if (!g_running) return 0;

        std::cout << "\n[2/6] Positions before calibration:" << std::endl;
        print_positions_table("Before calibration", *can1, *can2);
        std::cout << "\n  Note: these values are motor readings in the current physical pose relative to the previously stored zero positions,\n"
                  << "  so nonzero values are normal. The next step sets this pose as the new zero position.\n"
                  << "  First confirm that all 12 values print normally (not all 0 and not all identical), which indicates CAN communication is working.\n";

        if (!ask_yes("\nIs the pose correct? Set zero positions for all 12 motors now?")) {
            std::cout << "[Canceled] User aborted before writing zero positions.\n";
            return 0;
        }

        std::cout << "\n[3/6] Sending set_zero_position to all 12 disabled motors...\n" << std::flush;
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

        std::cout << "\n[4/6] Positions after zero write (RAM only; lost after power-off):\n";
        print_positions_table("After zero write (RAM)", *can1, *can2);

        if (!ask_yes("\nAre all 12 values approximately 0? Write to FLASH for power-off persistence?")) {
            std::cout << "[Canceled] User skipped FLASH write.\n"
                      << "          The current zero positions are only stored in RAM; the motors will restore the old zero positions on the next power-on.\n";
            return 0;
        }

        std::cout << "\n[5/6] Writing to FLASH while all motors remain disabled (about 110 ms x 12 ~= 1.4 s)...\n" << std::flush;
        for (int i = 1; i <= 6; ++i) {
            if (auto m = can1->getMotor(static_cast<uint16_t>(i))) can1->save_motor_param(*m);
        }
        for (int i = 1; i <= 6; ++i) {
            if (auto m = can2->getMotor(static_cast<uint16_t>(i))) can2->save_motor_param(*m);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        std::cout << "\n[6/6] Performing final verification while motors remain disabled...\n" << std::flush;
        warmup_read(*can1, *can2, 3, 150);

        print_positions_table("After FLASH write while disabled (verified)", *can1, *can2);

        std::cout << "\n[Done] If all 12 values above are approximately 0, the zero positions were written to FLASH successfully.\n"
                  << "       Later, power-cycle the motors and run this tool again to check the [2/6] output.\n"
                  << "       If the readings are still approximately 0, FLASH persistence verification passed.\n"
                  << "       The program will automatically disable all motors when it exits.\n";

    } catch (const std::exception& e) {
        std::cerr << "\n[Error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
