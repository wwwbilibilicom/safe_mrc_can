#pragma once

#include <linux/can.h>

#include <cstdint>

namespace safemrc::mrc {

enum class MRCMode : uint8_t {
    FREE = 0,
    FIX_LIMIT = 1,
    ADAPTATION = 2,
    DEBUG = 3,
    MRC_RESET = 4,
    ZERO = 5,
    REFRESH = 6
};

struct MRCCommand {
    MRCMode mode = MRCMode::FREE;
    int32_t des_coil_current = 0;  // mA, x1000 scaling
};

struct MRCFeedback {
    MRCMode mode = MRCMode::FREE;
    bool collision = false;
    int32_t encoder_value = 0;     // raw, = rad * 65535
    int16_t present_current = 0;   // mA, x1000 scaling

    double position_rad() const { return static_cast<double>(encoder_value) / 65535.0; }
    double current_amp() const { return static_cast<double>(present_current) / 1000.0; }
};

class MRCProtocol {
public:
    static constexpr canid_t CMD_BASE = 0x100;
    static constexpr canid_t FBK_BASE = 0x200;

    static canid_t cmd_id(uint8_t device_id) { return CMD_BASE | device_id; }
    static canid_t fbk_id(uint8_t device_id) { return FBK_BASE | device_id; }

    static can_frame encode_cmd(uint8_t device_id, const MRCCommand& cmd);
    static canfd_frame encode_cmd_fd(uint8_t device_id, const MRCCommand& cmd);
    static MRCFeedback decode_fbk(const can_frame& frame);
    static MRCFeedback decode_fbk(const canfd_frame& frame);

    static MRCCommand free_cmd();
    static MRCCommand fix_limit_cmd(double current_amp);
    static MRCCommand adaptation_cmd(double current_amp);
    static MRCCommand reset_cmd();
    static MRCCommand zero_cmd();
    static MRCCommand refresh_cmd();
};

}  // namespace safemrc::mrc
