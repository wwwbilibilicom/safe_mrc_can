#pragma once

#include <map>
#include <memory>

#include <safemrc/can/can_device.hpp>

namespace safemrc::can {

class CANDeviceCollection {
public:
    void add_device(std::shared_ptr<CANDevice> device);
    void remove_device(canid_t recv_id);
    void dispatch(const can_frame& frame);
    void dispatch(const canfd_frame& frame);

    const std::map<canid_t, std::shared_ptr<CANDevice>>& devices() const { return devices_; }

private:
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
};

}  // namespace safemrc::can
