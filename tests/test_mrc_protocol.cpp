#include <gtest/gtest.h>
#include <safemrc/mrc/mrc_protocol.hpp>
#include <cmath>

using namespace safemrc::mrc;

TEST(MRCProtocol, CmdIdEncoding) {
    EXPECT_EQ(MRCProtocol::cmd_id(1), 0x101u);
    EXPECT_EQ(MRCProtocol::cmd_id(254), 0x1FEu);
}

TEST(MRCProtocol, FbkIdEncoding) {
    EXPECT_EQ(MRCProtocol::fbk_id(1), 0x201u);
    EXPECT_EQ(MRCProtocol::fbk_id(254), 0x2FEu);
}

TEST(MRCProtocol, EncodeDecodeRoundTrip) {
    auto cmd = MRCProtocol::fix_limit_cmd(3.0);
    auto frame = MRCProtocol::encode_cmd(1, cmd);
    EXPECT_EQ(frame.can_id, 0x101u);
    EXPECT_EQ(frame.can_dlc, 8);
    EXPECT_EQ(frame.data[0], static_cast<uint8_t>(MRCMode::FIX_LIMIT));
    EXPECT_EQ(frame.data[1], 0x00);
    EXPECT_EQ(frame.data[2], 0xB8); EXPECT_EQ(frame.data[3], 0x0B);
    EXPECT_EQ(frame.data[4], 0x00); EXPECT_EQ(frame.data[5], 0x00);
}

TEST(MRCProtocol, EncodeCmdFd) {
    auto cmd = MRCProtocol::adaptation_cmd(2.5);
    auto frame = MRCProtocol::encode_cmd_fd(3, cmd);
    EXPECT_EQ(frame.can_id, 0x103u); EXPECT_EQ(frame.len, 8);
    EXPECT_EQ(frame.data[0], static_cast<uint8_t>(MRCMode::ADAPTATION));
    EXPECT_EQ(frame.data[2], 0xC4); EXPECT_EQ(frame.data[3], 0x09);
}

TEST(MRCProtocol, DecodeFeedback) {
    can_frame frame{};
    frame.can_id = 0x201; frame.can_dlc = 8;
    frame.data[0] = 2; frame.data[1] = 0x01;
    frame.data[2] = 0xFF; frame.data[3] = 0xFF; frame.data[4] = 0x00; frame.data[5] = 0x00;
    frame.data[6] = 0xAC; frame.data[7] = 0x0D;
    auto fbk = MRCProtocol::decode_fbk(frame);
    EXPECT_EQ(fbk.mode, MRCMode::ADAPTATION); EXPECT_TRUE(fbk.collision);
    EXPECT_EQ(fbk.encoder_value, 65535);
    EXPECT_NEAR(fbk.position_rad(), 1.0, 1e-4);
    EXPECT_EQ(fbk.present_current, 3500); EXPECT_NEAR(fbk.current_amp(), 3.5, 1e-6);
}

TEST(MRCProtocol, DecodeFeedbackNegativeValues) {
    can_frame frame{};
    frame.can_id = 0x201; frame.can_dlc = 8;
    frame.data[0] = 0; frame.data[1] = 0x00;
    frame.data[2] = 0x00; frame.data[3] = 0x80; frame.data[4] = 0xFF; frame.data[5] = 0xFF;
    frame.data[6] = 0x18; frame.data[7] = 0xFC;
    auto fbk = MRCProtocol::decode_fbk(frame);
    EXPECT_EQ(fbk.encoder_value, -32768);
    EXPECT_NEAR(fbk.position_rad(), -32768.0 / 65535.0, 1e-6);
    EXPECT_EQ(fbk.present_current, -1000); EXPECT_NEAR(fbk.current_amp(), -1.0, 1e-6);
}

TEST(MRCProtocol, ConvenienceFactories) {
    auto free = MRCProtocol::free_cmd();
    EXPECT_EQ(free.mode, MRCMode::FREE); EXPECT_EQ(free.des_coil_current, 0);
    auto fix = MRCProtocol::fix_limit_cmd(5.0);
    EXPECT_EQ(fix.mode, MRCMode::FIX_LIMIT); EXPECT_EQ(fix.des_coil_current, 5000);
    auto adapt = MRCProtocol::adaptation_cmd(-2.0);
    EXPECT_EQ(adapt.mode, MRCMode::ADAPTATION); EXPECT_EQ(adapt.des_coil_current, -2000);
    EXPECT_EQ(MRCProtocol::reset_cmd().mode, MRCMode::MRC_RESET);
    EXPECT_EQ(MRCProtocol::zero_cmd().mode, MRCMode::ZERO);
    EXPECT_EQ(MRCProtocol::refresh_cmd().mode, MRCMode::REFRESH);
}

TEST(MRCProtocol, EncodeNegativeCurrent) {
    auto cmd = MRCProtocol::fix_limit_cmd(-3.0);
    auto frame = MRCProtocol::encode_cmd(1, cmd);
    EXPECT_EQ(frame.data[2], 0x48); EXPECT_EQ(frame.data[3], 0xF4);
    EXPECT_EQ(frame.data[4], 0xFF); EXPECT_EQ(frame.data[5], 0xFF);
}
