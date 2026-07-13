// D1 single-motor zero-position calibration tool
//
// Usage:
//   set_zero_one <CAN interface> <CAN ID>
// Example:
//   set_zero_one can1 0x02
//   set_zero_one can2 5
//
// Safest practice: connect only the target motor physically to this bus, or ensure all other motors on the bus have already been calibrated
// and are disabled / not moving.

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
    std::cerr << "\n[Interrupted] received signal " << signum
              << ", the destructor will automatically disable the motor..." << std::endl;
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
    std::cerr << "Usage:" << argv0 << " <CAN interface> <CAN ID>\n"
              << "  CAN interface   for example can1 or can2\n"
              << "  CAN ID     1..6(decimal) or 0x01..0x06(hexadecimal)\n"
              << "\nExample:\n"
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
        long v = std::stol(argv[2], nullptr, 0);  // base=0 supports both the 0x prefix and decimal
        if (v < 1 || v > 6) throw std::out_of_range("CAN ID must be between 1 and 6");
        can_id = static_cast<uint16_t>(v);
    } catch (const std::exception& e) {
        std::cerr << "[Error] Invalid CAN ID: " << e.what() << "\n";
        print_usage(argv[0]);
        return 1;
    }
    uint16_t mst_id = static_cast<uint16_t>(0x10 + can_id);

    std::cout << "\n############################################\n"
              <<   "#  D1 single-motor zero-position calibration tool\n"
              <<   "############################################\n"
              << "  Target bus:  " << bus_name << "\n"
              << "  Target CAN ID:0x" << std::hex << std::setw(2) << std::setfill('0')
              << can_id << std::dec << std::setfill(' ')
              << "  (decimal " << can_id << ")\n"
              << "  Target MST ID:0x" << std::hex << std::setw(2) << std::setfill('0')
              << mst_id << std::dec << std::setfill(' ')
              << "  (decimal " << mst_id << ")\n"
              << "  Motor model:  DM6248P (MIT mode)\n";

    std::cout << "\n!! Safety checklist (press Ctrl+C immediately if any item is not satisfied) !!\n"
              << "  [1] The joint is free (or the whole leg is suspended) with no load\n"
              << "  [2] The joint has been moved to the URDF mechanical zero position\n"
              << "  [3] " << bus_name << " is UP\n"
              << "  [4] rl_real_d1 / test_motor is not running\n\n";

    if (!ask_yes("Continue?")) {
        std::cout << "[Canceled] User aborted.\n";
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

        std::cout << "\n[1/6] Opening " << bus_name << " and automatically enabling motor " << can_id
                  << "...\n" << std::flush;
        auto bus = std::make_shared<damiao::Motor_Control>(bus_name, &init_data, damiao::canfd);

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        auto motor = bus->getMotor(can_id);
        if (!motor) {
            std::cerr << "[Error] CAN ID " << can_id << "  failed to register successfully.\n";
            return 1;
        }

        for (int k = 0; k < 5 && g_running; ++k) {
            bus->refresh_motor_status(*motor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!g_running) return 0;

        std::cout << "\n[2/6] Positions before calibration:  "
                  << std::fixed << std::setprecision(6)
                  << motor->Get_Position() << " rad\n";

        if (!ask_yes("\nIs the pose correct? Set the zero position now?")) {
            std::cout << "[Canceled] User aborted before writing zero positions.\n";
            return 0;
        }

        std::cout << "\n[3/6] Sending set_zero_position...\n";
        bus->set_zero_position(*motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        for (int k = 0; k < 3; ++k) {
            bus->refresh_motor_status(*motor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "[4/6] Position after zero write (RAM):  "
                  << std::fixed << std::setprecision(6)
                  << motor->Get_Position() << " rad\n";

        if (!ask_yes("\nIs the reading approximately 0? Write to FLASH for power-off persistence?")) {
            std::cout << "[Canceled] User skipped FLASH write; the zero position is only stored in RAM.\n";
            return 0;
        }

        std::cout << "\n[5/6] Writing to FLASH...\n";
        bus->save_motor_param(*motor);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));

        std::cout << "[6/6] Re-enabling and performing final verification...\n";
        bus->enable_all(damiao::MIT_MODE);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        for (int k = 0; k < 3; ++k) {
            bus->refresh_motor_status(*motor);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Position after FLASH write:  "
                  << std::fixed << std::setprecision(6)
                  << motor->Get_Position() << " rad\n";

        std::cout << "\n[Done] If the reading above is approximately 0, the FLASH write succeeded.\n"
                  << "       The program will automatically disable the motor when it exits.\n";

    } catch (const std::exception& e) {
        std::cerr << "\n[Error] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
