#pragma once

#include <safemrc/can/can_device.hpp>
#include <safemrc/mrc/mrc_protocol.hpp>
#include <safemrc/mrc/mrc_state.hpp>

namespace safemrc::mrc {

class MRCDevice : public can::CANDevice {
public:
    explicit MRCDevice(uint8_t device_id, bool use_fd = false);

    void on_frame(const can_frame& frame) override;
    void on_frame(const canfd_frame& frame) override;

    const MRCState& state() const { return state_; }
    uint8_t device_id() const { return device_id_; }

private:
    uint8_t device_id_;
    MRCState state_;
};

}  // namespace safemrc::mrc
