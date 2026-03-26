#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdexcept>
#include <string>

namespace safemrc::can {

class CANSocketException : public std::runtime_error {
public:
    explicit CANSocketException(const std::string& message)
        : std::runtime_error("CAN socket error: " + message) {}
};

class CANSocket {
public:
    explicit CANSocket(const std::string& interface, bool enable_fd = false);
    ~CANSocket();

    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;
    CANSocket(CANSocket&& other) noexcept;
    CANSocket& operator=(CANSocket&& other) noexcept;

    bool write_can_frame(const can_frame& frame);
    bool write_canfd_frame(const canfd_frame& frame);
    bool read_can_frame(can_frame& frame);
    bool read_canfd_frame(canfd_frame& frame);

    bool is_data_available(int timeout_us = 0);
    bool is_fd_enabled() const { return fd_enabled_; }
    int fd() const { return socket_fd_; }
    const std::string& interface_name() const { return interface_; }

private:
    bool initialize(const std::string& interface);
    void cleanup();

    int socket_fd_ = -1;
    std::string interface_;
    bool fd_enabled_ = false;
};

}  // namespace safemrc::can
