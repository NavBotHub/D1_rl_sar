/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dog_usb_receiver.hpp"
#include "joystick.hh"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

namespace
{

std::atomic<bool> g_running{true};

struct Config
{
    std::string joy_device = "/dev/input/js0";
    int joy_button = 3;
    std::string dog_usb_device = "/dev/ttyTHS1";
    int dog_usb_baud = 115200;
    int dog_usb_l1_button_bit = 8;
    int dog_usb_timeout_ms = 300;
    int dog_remote_timeout_ms = 1500;
    int monitor_interval_ms = 10;
    std::string main_service = "rl-sar-main.service";
    bool dry_run = false;
};

void HandleSignal(int)
{
    g_running.store(false);
}

void PrintUsage(const char* argv0)
{
    std::cout
        << "Usage: " << argv0 << " [OPTIONS]\n"
        << "\n"
        << "Options:\n"
        << "  --dry-run                         Log triggers without starting systemd service\n"
        << "  --joy-device PATH                 Linux joystick device (default: /dev/input/js0)\n"
        << "  --joy-button INDEX                Button index that starts the main service (default: 3)\n"
        << "  --dog-usb-device PATH             DOG_CTRL serial device (default: /dev/ttyTHS1)\n"
        << "  --dog-usb-baud BAUD               DOG_CTRL serial baud rate (default: 115200)\n"
        << "  --dog-usb-l1-button-bit BIT       DOG_CTRL L1 bit in buttons field, 0..15 (default: 8)\n"
        << "  --dog-usb-timeout-ms MS           Fresh USB frame timeout (default: 300)\n"
        << "  --dog-remote-timeout-ms MS        Fresh LoRa remote packet timeout (default: 1500)\n"
        << "  --monitor-interval-ms MS          Poll interval while waiting for triggers (default: 10)\n"
        << "  --main-service NAME               systemd unit to start (default: rl-sar-main.service)\n"
        << "  -h, --help                        Show this help\n";
}

bool ParseInt(const char* text, int* value)
{
    if (!text || !value)
    {
        return false;
    }
    char* end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0')
    {
        return false;
    }
    *value = static_cast<int>(parsed);
    return true;
}

bool NeedValue(int argc, char** argv, int index)
{
    if (index + 1 < argc)
    {
        return true;
    }
    std::cerr << "Missing value for " << argv[index] << std::endl;
    return false;
}

bool ParseArgs(int argc, char** argv, Config* config)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            PrintUsage(argv[0]);
            return false;
        }
        if (arg == "--dry-run")
        {
            config->dry_run = true;
            continue;
        }
        if (arg == "--joy-device")
        {
            if (!NeedValue(argc, argv, i)) {return false;}
            config->joy_device = argv[++i];
            continue;
        }
        if (arg == "--joy-button")
        {
            if (!NeedValue(argc, argv, i) || !ParseInt(argv[++i], &config->joy_button)) {return false;}
            continue;
        }
        if (arg == "--dog-usb-device")
        {
            if (!NeedValue(argc, argv, i)) {return false;}
            config->dog_usb_device = argv[++i];
            continue;
        }
        if (arg == "--dog-usb-baud")
        {
            if (!NeedValue(argc, argv, i) || !ParseInt(argv[++i], &config->dog_usb_baud)) {return false;}
            continue;
        }
        if (arg == "--dog-usb-l1-button-bit")
        {
            if (!NeedValue(argc, argv, i) || !ParseInt(argv[++i], &config->dog_usb_l1_button_bit)) {return false;}
            continue;
        }
        if (arg == "--dog-usb-timeout-ms")
        {
            if (!NeedValue(argc, argv, i) || !ParseInt(argv[++i], &config->dog_usb_timeout_ms)) {return false;}
            continue;
        }
        if (arg == "--dog-remote-timeout-ms")
        {
            if (!NeedValue(argc, argv, i) || !ParseInt(argv[++i], &config->dog_remote_timeout_ms)) {return false;}
            continue;
        }
        if (arg == "--monitor-interval-ms")
        {
            if (!NeedValue(argc, argv, i) || !ParseInt(argv[++i], &config->monitor_interval_ms)) {return false;}
            continue;
        }
        if (arg == "--main-service")
        {
            if (!NeedValue(argc, argv, i)) {return false;}
            config->main_service = argv[++i];
            continue;
        }

        std::cerr << "Unknown argument: " << arg << std::endl;
        return false;
    }

    if (config->joy_button < 0)
    {
        std::cerr << "--joy-button must be >= 0" << std::endl;
        return false;
    }
    if (config->dog_usb_l1_button_bit < 0 || config->dog_usb_l1_button_bit > 15)
    {
        std::cerr << "--dog-usb-l1-button-bit must be in range 0..15" << std::endl;
        return false;
    }
    if (config->dog_usb_baud <= 0 || config->dog_usb_timeout_ms <= 0 ||
        config->dog_remote_timeout_ms <= 0 || config->monitor_interval_ms <= 0)
    {
        std::cerr << "Numeric timeout/baud/interval arguments must be positive" << std::endl;
        return false;
    }
    if (config->main_service.empty())
    {
        std::cerr << "--main-service must not be empty" << std::endl;
        return false;
    }
    return true;
}

int RunCommand(const std::vector<std::string>& args)
{
    if (args.empty())
    {
        return -1;
    }

    pid_t pid = fork();
    if (pid < 0)
    {
        std::cerr << "fork failed: " << std::strerror(errno) << std::endl;
        return -1;
    }
    if (pid == 0)
    {
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (const auto& arg : args)
        {
            argv.push_back(const_cast<char*>(arg.c_str()));
        }
        argv.push_back(nullptr);
        execvp(argv[0], argv.data());
        _exit(127);
    }

    int status = 0;
    while (waitpid(pid, &status, 0) < 0)
    {
        if (errno == EINTR)
        {
            if (!g_running.load())
            {
                return -1;
            }
            continue;
        }
        std::cerr << "waitpid failed: " << std::strerror(errno) << std::endl;
        return -1;
    }

    if (WIFEXITED(status))
    {
        return WEXITSTATUS(status);
    }
    if (WIFSIGNALED(status))
    {
        return 128 + WTERMSIG(status);
    }
    return -1;
}

bool IsMainServiceActive(const Config& config)
{
    if (config.dry_run)
    {
        return false;
    }
    return RunCommand({"systemctl", "is-active", "--quiet", config.main_service}) == 0;
}

bool StartMainService(const Config& config)
{
    if (config.dry_run)
    {
        std::cout << "[trigger] dry-run: would start " << config.main_service << std::endl;
        return true;
    }

    const int ret = RunCommand({"systemctl", "start", config.main_service});
    if (ret == 0)
    {
        std::cout << "[trigger] started " << config.main_service << std::endl;
        return true;
    }

    std::cerr << "[trigger] failed to start " << config.main_service
              << ", systemctl exit code=" << ret << std::endl;
    return false;
}

void WaitForMainServiceToStop(const Config& config)
{
    if (config.dry_run)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        return;
    }

    for (int i = 0; i < 40 && g_running.load(); ++i)
    {
        if (IsMainServiceActive(config))
        {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    while (g_running.load() && IsMainServiceActive(config))
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

bool DogFrameIsFresh(const DogUsbState& state, const Config& config)
{
    if (!state.valid)
    {
        return false;
    }

    const auto now = std::chrono::steady_clock::now();
    const bool usb_fresh =
        (now - state.last_valid_frame_time <= std::chrono::milliseconds(config.dog_usb_timeout_ms));
    const bool remote_fresh =
        state.last_remote_age_ms <= static_cast<uint32_t>(config.dog_remote_timeout_ms);
    return usb_fresh && remote_fresh;
}

bool WaitForTrigger(const Config& config)
{
    std::unique_ptr<Joystick> joystick;
    if (!config.joy_device.empty())
    {
        joystick = std::make_unique<Joystick>(config.joy_device);
        if (joystick->isFound())
        {
            std::cout << "[trigger] listening joystick " << config.joy_device
                      << " button[" << config.joy_button << "]" << std::endl;
        }
        else
        {
            std::cerr << "[trigger] failed to open joystick " << config.joy_device << std::endl;
            joystick.reset();
        }
    }

    DogUsbReceiver dog_usb;
    bool dog_usb_running = false;
    if (!config.dog_usb_device.empty())
    {
        dog_usb_running = dog_usb.Start(config.dog_usb_device, config.dog_usb_baud);
        if (dog_usb_running)
        {
            std::cout << "[trigger] listening DOG_CTRL " << config.dog_usb_device
                      << " L1 bit " << config.dog_usb_l1_button_bit << std::endl;
        }
    }

    if (!joystick && !dog_usb_running)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        return false;
    }

    const uint16_t l1_mask = static_cast<uint16_t>(1u << config.dog_usb_l1_button_bit);
    uint16_t prev_dog_buttons = 0;
    bool have_prev_dog_buttons = false;
    auto last_service_check = std::chrono::steady_clock::now();

    while (g_running.load())
    {
        if (!config.dry_run)
        {
            const auto now = std::chrono::steady_clock::now();
            if (now - last_service_check >= std::chrono::seconds(1))
            {
                if (IsMainServiceActive(config))
                {
                    std::cout << "[trigger] " << config.main_service
                              << " is already active; releasing input devices" << std::endl;
                    dog_usb.Stop();
                    return false;
                }
                last_service_check = now;
            }
        }

        if (joystick)
        {
            JoystickEvent event;
            while (joystick->sample(&event))
            {
                if (event.isButton() && !event.isInitialState() &&
                    static_cast<int>(event.number) == config.joy_button && event.value != 0)
                {
                    std::cout << "[trigger] joystick button[" << config.joy_button
                              << "] pressed" << std::endl;
                    dog_usb.Stop();
                    joystick.reset();
                    return true;
                }
            }
        }

        if (dog_usb_running)
        {
            const DogUsbState state = dog_usb.GetLatestState();
            if (DogFrameIsFresh(state, config))
            {
                const bool l1_now = (state.buttons & l1_mask) != 0;
                const bool l1_prev = (prev_dog_buttons & l1_mask) != 0;
                if (have_prev_dog_buttons && l1_now && !l1_prev)
                {
                    std::cout << "[trigger] DOG_CTRL L1 ON, buttons=0x"
                              << std::hex << state.buttons << std::dec << std::endl;
                    dog_usb.Stop();
                    joystick.reset();
                    return true;
                }
                prev_dog_buttons = state.buttons;
                have_prev_dog_buttons = true;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(config.monitor_interval_ms));
    }

    dog_usb.Stop();
    return false;
}

} // namespace

int main(int argc, char** argv)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help")
        {
            PrintUsage(argv[0]);
            return 0;
        }
    }

    Config config;
    if (!ParseArgs(argc, argv, &config))
    {
        return 2;
    }

    std::signal(SIGINT, HandleSignal);
    std::signal(SIGTERM, HandleSignal);

    std::cout << "[trigger] rl_real_d1_trigger started"
              << (config.dry_run ? " (dry-run)" : "") << std::endl;

    while (g_running.load())
    {
        if (IsMainServiceActive(config))
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }

        if (!WaitForTrigger(config))
        {
            continue;
        }

        if (StartMainService(config))
        {
            WaitForMainServiceToStop(config);
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    std::cout << "[trigger] exiting" << std::endl;
    return 0;
}
