#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>

namespace safemrc::can {

class CANDevice {
public:
    explicit CANDevice(canid_t send_id, canid_t recv_id,
                       canid_t recv_mask = CAN_SFF_MASK, bool use_fd = false)
        : send_id_(send_id), recv_id_(recv_id), recv_mask_(recv_mask), use_fd_(use_fd) {}
    virtual ~CANDevice() = default;

    virtual void on_frame(const can_frame& frame) = 0;
    virtual void on_frame(const canfd_frame& frame) = 0;

    canid_t send_id() const { return send_id_; }
    canid_t recv_id() const { return recv_id_; }
    canid_t recv_mask() const { return recv_mask_; }
    bool is_fd() const { return use_fd_; }

protected:
    canid_t send_id_;
    canid_t recv_id_;
    canid_t recv_mask_;
    bool use_fd_;
};

}  // namespace safemrc::can
