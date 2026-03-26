#include <safemrc/mrc/mrc_protocol.hpp>

#include <cstring>

namespace safemrc::mrc {

static void fill_cmd_data(uint8_t* data, const MRCCommand& cmd) {
    std::memset(data, 0, 8);
    data[0] = static_cast<uint8_t>(cmd.mode);
    data[1] = 0x00;
    data[2] = static_cast<uint8_t>(cmd.des_coil_current & 0xFF);
    data[3] = static_cast<uint8_t>((cmd.des_coil_current >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>((cmd.des_coil_current >> 16) & 0xFF);
    data[5] = static_cast<uint8_t>((cmd.des_coil_current >> 24) & 0xFF);
    data[6] = 0x00;
    data[7] = 0x00;
}

static MRCFeedback parse_fbk_data(const uint8_t* data) {
    MRCFeedback fbk;
    fbk.mode = static_cast<MRCMode>(data[0]);
    fbk.collision = (data[1] != 0);
    fbk.encoder_value = static_cast<int32_t>(
        static_cast<uint32_t>(data[2]) |
        (static_cast<uint32_t>(data[3]) << 8) |
        (static_cast<uint32_t>(data[4]) << 16) |
        (static_cast<uint32_t>(data[5]) << 24));
    fbk.present_current = static_cast<int16_t>(
        static_cast<uint16_t>(data[6]) |
        (static_cast<uint16_t>(data[7]) << 8));
    return fbk;
}

can_frame MRCProtocol::encode_cmd(uint8_t device_id, const MRCCommand& cmd) {
    can_frame frame{};
    frame.can_id = cmd_id(device_id);
    frame.can_dlc = 8;
    fill_cmd_data(frame.data, cmd);
    return frame;
}

canfd_frame MRCProtocol::encode_cmd_fd(uint8_t device_id, const MRCCommand& cmd) {
    canfd_frame frame{};
    frame.can_id = cmd_id(device_id);
    frame.len = 8;
    fill_cmd_data(frame.data, cmd);
    return frame;
}

MRCFeedback MRCProtocol::decode_fbk(const can_frame& frame) {
    return parse_fbk_data(frame.data);
}

MRCFeedback MRCProtocol::decode_fbk(const canfd_frame& frame) {
    return parse_fbk_data(frame.data);
}

MRCCommand MRCProtocol::free_cmd() { return {MRCMode::FREE, 0}; }
MRCCommand MRCProtocol::fix_limit_cmd(double current_amp) {
    return {MRCMode::FIX_LIMIT, static_cast<int32_t>(current_amp * 1000.0)};
}
MRCCommand MRCProtocol::adaptation_cmd(double current_amp) {
    return {MRCMode::ADAPTATION, static_cast<int32_t>(current_amp * 1000.0)};
}
MRCCommand MRCProtocol::reset_cmd() { return {MRCMode::MRC_RESET, 0}; }
MRCCommand MRCProtocol::zero_cmd() { return {MRCMode::ZERO, 0}; }
MRCCommand MRCProtocol::refresh_cmd() { return {MRCMode::REFRESH, 0}; }

}  // namespace safemrc::mrc
