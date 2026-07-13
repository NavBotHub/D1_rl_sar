/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dog_usb_receiver.hpp"
#include "logger.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <iterator>
#include <poll.h>
#include <unistd.h>

namespace
{
constexpr uint8_t kHeader0 = 'D';
constexpr uint8_t kHeader1 = 'T';
constexpr uint8_t kVersion = 1;
constexpr uint8_t kTypeDogCtrl = 0x20;
constexpr uint8_t kPayloadLen = 16;
constexpr size_t kHeaderLen = 6;
constexpr size_t kPayloadOffset = kHeaderLen;
constexpr size_t kCrcOffset = kHeaderLen + kPayloadLen;
constexpr size_t kFrameLen = kCrcOffset + 2;
constexpr size_t kCrcDataLen = kCrcOffset;
constexpr size_t kMaxBufferSize = 4096;
constexpr size_t kKeepTailSize = kFrameLen - 1;
}

DogUsbReceiver::~DogUsbReceiver()
{
    Stop();
}

bool DogUsbReceiver::Start(const std::string& device, int baud)
{
    if (running_.load())
    {
        return true;
    }

    device_ = device;
    baud_ = baud;
    fd_ = OpenSerial(device_);
    if (fd_ < 0)
    {
        connected_.store(false);
        return false;
    }

    if (!ConfigureSerial(fd_, baud_))
    {
        close(fd_);
        fd_ = -1;
        connected_.store(false);
        return false;
    }

    running_.store(true);
    connected_.store(true);
    rx_thread_ = std::thread(&DogUsbReceiver::ThreadMain, this);

    std::cout << LOGGER::INFO << "[dog_usb] opened " << device_ << " at " << baud_ << " baud" << std::endl;
    return true;
}

void DogUsbReceiver::Stop()
{
    running_.store(false);
    if (rx_thread_.joinable())
    {
        rx_thread_.join();
    }
}

DogUsbState DogUsbReceiver::GetLatestState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_state_;
}

DogUsbCounters DogUsbReceiver::GetCounters() const
{
    DogUsbCounters counters;
    counters.valid_frame_count = valid_frame_count_.load();
    counters.crc_error_count = crc_error_count_.load();
    counters.resync_drop_count = resync_drop_count_.load();
    counters.read_error_count = read_error_count_.load();
    return counters;
}

bool DogUsbReceiver::IsConnected() const
{
    return connected_.load();
}

bool DogUsbReceiver::IsRunning() const
{
    return running_.load();
}

int DogUsbReceiver::OpenSerial(const std::string& device)
{
    int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cout << LOGGER::ERROR << "[dog_usb] failed to open " << device
                  << ": " << std::strerror(errno) << std::endl;
    }
    return fd;
}

bool DogUsbReceiver::ConfigureSerial(int fd, int baud)
{
    termios tty {};
    if (tcgetattr(fd, &tty) != 0)
    {
        std::cout << LOGGER::ERROR << "[dog_usb] tcgetattr failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const speed_t speed = BaudToTermios(baud);
    if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0)
    {
        std::cout << LOGGER::ERROR << "[dog_usb] failed to set baud " << baud
                  << ": " << std::strerror(errno) << std::endl;
        return false;
    }

    if (tcsetattr(fd, TCSANOW, &tty) != 0)
    {
        std::cout << LOGGER::ERROR << "[dog_usb] tcsetattr failed: "
                  << std::strerror(errno) << std::endl;
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

speed_t DogUsbReceiver::BaudToTermios(int baud) const
{
    switch (baud)
    {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 500000: return B500000;
    case 921600: return B921600;
    default:
        std::cout << LOGGER::WARNING << "[dog_usb] unsupported baud " << baud
                  << ", using 115200" << std::endl;
        return B115200;
    }
}

void DogUsbReceiver::ThreadMain()
{
    uint8_t read_buffer[256];
    auto last_counter_log = std::chrono::steady_clock::now();
    uint64_t last_valid_log = 0;
    uint64_t last_crc_log = 0;

    while (running_.load())
    {
        pollfd pfd {};
        pfd.fd = fd_;
        pfd.events = POLLIN | POLLERR | POLLHUP | POLLNVAL;

        const int poll_ret = poll(&pfd, 1, 100);
        if (!running_.load())
        {
            break;
        }
        if (poll_ret < 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            read_error_count_.fetch_add(1);
            connected_.store(false);
            std::cout << LOGGER::ERROR << "[dog_usb] poll failed: "
                      << std::strerror(errno) << std::endl;
            break;
        }
        if (poll_ret > 0)
        {
            if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
            {
                read_error_count_.fetch_add(1);
                connected_.store(false);
                std::cout << LOGGER::ERROR << "[dog_usb] serial disconnected or poll error, revents=0x"
                          << std::hex << pfd.revents << std::dec << std::endl;
                break;
            }

            while (running_.load())
            {
                const ssize_t n = read(fd_, read_buffer, sizeof(read_buffer));
                if (n > 0)
                {
                    ParseBytes(read_buffer, static_cast<size_t>(n));
                    continue;
                }
                if (n == 0)
                {
                    break;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                {
                    break;
                }

                read_error_count_.fetch_add(1);
                connected_.store(false);
                std::cout << LOGGER::ERROR << "[dog_usb] read failed: "
                          << std::strerror(errno) << std::endl;
                running_.store(false);
                break;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_counter_log >= std::chrono::seconds(1))
        {
            const uint64_t valid = valid_frame_count_.load();
            const uint64_t crc = crc_error_count_.load();
            if (valid != last_valid_log || crc != last_crc_log)
            {
                std::cout << LOGGER::INFO << "[dog_usb] frames=" << valid
                          << " crc_errors=" << crc
                          << " resync_drops=" << resync_drop_count_.load()
                          << " read_errors=" << read_error_count_.load() << std::endl;
                last_valid_log = valid;
                last_crc_log = crc;
            }
            last_counter_log = now;
        }
    }

    running_.store(false);
    if (fd_ >= 0)
    {
        close(fd_);
        fd_ = -1;
    }
    connected_.store(false);
}

void DogUsbReceiver::ParseBytes(const uint8_t* data, size_t size)
{
    rx_buffer_.insert(rx_buffer_.end(), data, data + size);

    while (true)
    {
        if (rx_buffer_.size() > kMaxBufferSize)
        {
            const size_t drop_count = rx_buffer_.size() - kKeepTailSize;
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(drop_count));
            resync_drop_count_.fetch_add(drop_count);
        }

        const uint8_t magic[] = {kHeader0, kHeader1};
        auto it = std::search(rx_buffer_.begin(), rx_buffer_.end(), std::begin(magic), std::end(magic));
        if (it == rx_buffer_.end())
        {
            if (rx_buffer_.size() > 1)
            {
                const bool keep_last_d = (rx_buffer_.back() == kHeader0);
                const size_t keep = keep_last_d ? 1 : 0;
                const size_t drop_count = rx_buffer_.size() - keep;
                rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(drop_count));
                resync_drop_count_.fetch_add(drop_count);
            }
            return;
        }

        const size_t offset = static_cast<size_t>(std::distance(rx_buffer_.begin(), it));
        if (offset > 0)
        {
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(offset));
            resync_drop_count_.fetch_add(offset);
        }

        if (rx_buffer_.size() < kFrameLen)
        {
            return;
        }

        if (rx_buffer_[2] != kVersion || rx_buffer_[3] != kTypeDogCtrl || rx_buffer_[5] != kPayloadLen)
        {
            rx_buffer_.erase(rx_buffer_.begin());
            resync_drop_count_.fetch_add(1);
            continue;
        }

        const uint16_t computed_crc = Crc16Ccitt(rx_buffer_.data(), kCrcDataLen);
        const uint16_t received_crc = ReadLeU16(rx_buffer_.data() + kCrcOffset);
        if (computed_crc != received_crc)
        {
            crc_error_count_.fetch_add(1);
            rx_buffer_.erase(rx_buffer_.begin());
            continue;
        }

        const uint8_t* payload = rx_buffer_.data() + kPayloadOffset;
        DogUsbFrame frame;
        frame.usb_seq = rx_buffer_[4];
        frame.x_milli = ReadLeI16(payload + 0);
        frame.y_milli = ReadLeI16(payload + 2);
        frame.yaw_milli = ReadLeI16(payload + 4);
        frame.buttons = ReadLeU16(payload + 6);
        frame.last_remote_age_ms = ReadLeU32(payload + 8);
        frame.event = payload[12];
        frame.remote_seq = payload[13];
        frame.safe_state = payload[14];
        frame.reserved = payload[15];
        PublishFrame(frame);

        rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + static_cast<std::ptrdiff_t>(kFrameLen));
    }
}

void DogUsbReceiver::PublishFrame(const DogUsbFrame& frame)
{
    DogUsbState state;
    state.x = static_cast<double>(frame.x_milli) / 1000.0;
    state.y = static_cast<double>(frame.y_milli) / 1000.0;
    state.yaw = static_cast<double>(frame.yaw_milli) / 1000.0;
    state.buttons = frame.buttons;
    state.last_remote_age_ms = frame.last_remote_age_ms;
    state.event = frame.event;
    state.remote_seq = frame.remote_seq;
    state.safe_state = frame.safe_state != 0;
    state.valid = true;
    state.last_valid_frame_time = std::chrono::steady_clock::now();

    {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_state_ = state;
    }
    valid_frame_count_.fetch_add(1);
}

uint16_t DogUsbReceiver::Crc16Ccitt(const uint8_t* data, size_t size)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < size; ++i)
    {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
        {
            if ((crc & 0x8000) != 0)
            {
                crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
            }
            else
            {
                crc = static_cast<uint16_t>(crc << 1);
            }
        }
    }
    return crc;
}

int16_t DogUsbReceiver::ReadLeI16(const uint8_t* data)
{
    return static_cast<int16_t>(ReadLeU16(data));
}

uint16_t DogUsbReceiver::ReadLeU16(const uint8_t* data)
{
    return static_cast<uint16_t>(data[0]) |
           static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
}

uint32_t DogUsbReceiver::ReadLeU32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}
