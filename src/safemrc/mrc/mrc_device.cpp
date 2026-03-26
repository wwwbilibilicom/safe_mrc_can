#include <safemrc/mrc/mrc_device.hpp>

namespace safemrc::mrc {

MRCDevice::MRCDevice(uint8_t device_id, bool use_fd)
    : CANDevice(MRCProtocol::cmd_id(device_id),
                MRCProtocol::fbk_id(device_id),
                CAN_SFF_MASK, use_fd),
      device_id_(device_id),
      state_(device_id) {}

void MRCDevice::on_frame(const can_frame& frame) {
    state_.update(MRCProtocol::decode_fbk(frame));
}

void MRCDevice::on_frame(const canfd_frame& frame) {
    state_.update(MRCProtocol::decode_fbk(frame));
}

}  // namespace safemrc::mrc
