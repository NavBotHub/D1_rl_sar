/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef DOG_USB_RECEIVER_HPP
#define DOG_USB_RECEIVER_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>
#include <vector>

struct DogUsbFrame
{
    int16_t x_milli = 0;
    int16_t y_milli = 0;
    int16_t yaw_milli = 0;
    uint16_t buttons = 0;
    uint32_t last_remote_age_ms = 0;
    uint8_t event = 0;
    uint8_t remote_seq = 0;
    uint8_t safe_state = 0;
    uint8_t reserved = 0;
    uint8_t usb_seq = 0;
};

struct DogUsbState
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    uint16_t buttons = 0;
    uint32_t last_remote_age_ms = 0;
    uint8_t event = 0;
    uint8_t remote_seq = 0;
    bool safe_state = false;
    bool valid = false;
    std::chrono::steady_clock::time_point last_valid_frame_time =
        std::chrono::steady_clock::time_point::min();
};

struct DogUsbCounters
{
    uint64_t valid_frame_count = 0;
    uint64_t crc_error_count = 0;
    uint64_t resync_drop_count = 0;
    uint64_t read_error_count = 0;
};

class DogUsbReceiver
{
public:
    DogUsbReceiver() = default;
    ~DogUsbReceiver();

    DogUsbReceiver(const DogUsbReceiver&) = delete;
    DogUsbReceiver& operator=(const DogUsbReceiver&) = delete;

    bool Start(const std::string& device, int baud);
    void Stop();

    DogUsbState GetLatestState() const;
    DogUsbCounters GetCounters() const;
    bool IsConnected() const;
    bool IsRunning() const;

private:
    void ThreadMain();
    int OpenSerial(const std::string& device);
    bool ConfigureSerial(int fd, int baud);
    speed_t BaudToTermios(int baud) const;

    void ParseBytes(const uint8_t* data, size_t size);
    void PublishFrame(const DogUsbFrame& frame);
    static uint16_t Crc16Ccitt(const uint8_t* data, size_t size);
    static int16_t ReadLeI16(const uint8_t* data);
    static uint16_t ReadLeU16(const uint8_t* data);
    static uint32_t ReadLeU32(const uint8_t* data);

    std::string device_;
    int baud_ = 115200;
    int fd_ = -1;
    std::thread rx_thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};

    mutable std::mutex mutex_;
    DogUsbState latest_state_;

    std::vector<uint8_t> rx_buffer_;
    std::atomic<uint64_t> valid_frame_count_{0};
    std::atomic<uint64_t> crc_error_count_{0};
    std::atomic<uint64_t> resync_drop_count_{0};
    std::atomic<uint64_t> read_error_count_{0};
};

#endif // DOG_USB_RECEIVER_HPP
