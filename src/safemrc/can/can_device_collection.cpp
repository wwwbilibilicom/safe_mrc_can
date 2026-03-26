#include <safemrc/can/can_device_collection.hpp>

namespace safemrc::can {

void CANDeviceCollection::add_device(std::shared_ptr<CANDevice> device) {
    if (!device) return;
    devices_[device->recv_id()] = std::move(device);
}

void CANDeviceCollection::remove_device(canid_t recv_id) {
    devices_.erase(recv_id);
}

void CANDeviceCollection::dispatch(const can_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->on_frame(frame);
    }
}

void CANDeviceCollection::dispatch(const canfd_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->on_frame(frame);
    }
}

}  // namespace safemrc::can
