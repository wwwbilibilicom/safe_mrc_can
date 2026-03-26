#include <gtest/gtest.h>
#include <safemrc/safemrc.hpp>
#include <safemrc/can/can_socket.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <chrono>

static bool vcan_available() {
    struct ifreq ifr;
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) return false;
    strncpy(ifr.ifr_name, "vcan0", IFNAMSIZ - 1);
    bool ok = ioctl(s, SIOCGIFINDEX, &ifr) >= 0;
    close(s);
    return ok;
}

class IntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!vcan_available()) {
            GTEST_SKIP() << "vcan0 not available. Run: sudo modprobe vcan && "
                            "sudo ip link add dev vcan0 type vcan && "
                            "sudo ip link set up vcan0";
        }
    }
};

TEST_F(IntegrationTest, SendAndReceiveViaLoopback) {
    safemrc::SafeMRC mrc("vcan0");
    mrc.add_device(1);
    mrc.start();

    safemrc::can::CANSocket sim_socket("vcan0");

    mrc.set_fix_limit(1, 2.0);

    if (sim_socket.is_data_available(10000)) {
        can_frame cmd{};
        ASSERT_TRUE(sim_socket.read_can_frame(cmd));
        EXPECT_EQ(cmd.can_id, 0x101u);
        EXPECT_EQ(cmd.data[0], static_cast<uint8_t>(safemrc::mrc::MRCMode::FIX_LIMIT));
    }

    can_frame fbk{};
    fbk.can_id = 0x201; fbk.can_dlc = 8;
    fbk.data[0] = 1; fbk.data[1] = 0x00;
    fbk.data[2] = 0x00; fbk.data[3] = 0x80; fbk.data[4] = 0x00; fbk.data[5] = 0x00;
    fbk.data[6] = 0xD0; fbk.data[7] = 0x07;
    sim_socket.write_can_frame(fbk);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    auto& state = mrc.device_state(1);
    EXPECT_EQ(state.mode(), safemrc::mrc::MRCMode::FIX_LIMIT);
    EXPECT_FALSE(state.collision());
    EXPECT_EQ(state.feedback().encoder_value, 32768);
    EXPECT_EQ(state.feedback().present_current, 2000);
    EXPECT_FALSE(state.is_stale(std::chrono::milliseconds(100)));

    mrc.stop();
}

TEST_F(IntegrationTest, MultipleDevices) {
    safemrc::SafeMRC mrc("vcan0");
    mrc.add_device(1); mrc.add_device(2);
    mrc.start();

    safemrc::can::CANSocket sim("vcan0");

    can_frame fbk1{};
    fbk1.can_id = 0x201; fbk1.can_dlc = 8; fbk1.data[0] = 0;
    sim.write_can_frame(fbk1);

    can_frame fbk2{};
    fbk2.can_id = 0x202; fbk2.can_dlc = 8; fbk2.data[0] = 2; fbk2.data[1] = 0x01;
    sim.write_can_frame(fbk2);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_EQ(mrc.device_state(1).mode(), safemrc::mrc::MRCMode::FREE);
    EXPECT_EQ(mrc.device_state(2).mode(), safemrc::mrc::MRCMode::ADAPTATION);
    EXPECT_TRUE(mrc.device_state(2).collision());

    mrc.stop();
}
