#include <gtest/gtest.h>
#include <safemrc/can/can_device.hpp>
#include <safemrc/can/can_device_collection.hpp>
#include <cstring>

namespace {
class MockDevice : public safemrc::can::CANDevice {
public:
    MockDevice(canid_t s, canid_t r) : CANDevice(s, r), frame_count(0), last_byte0(0) {}
    void on_frame(const can_frame& f) override { ++frame_count; last_byte0 = f.data[0]; }
    void on_frame(const canfd_frame& f) override { ++frame_count; last_byte0 = f.data[0]; }
    int frame_count; uint8_t last_byte0;
};

can_frame make_frame(canid_t id, uint8_t b0) {
    can_frame f{}; f.can_id = id; f.can_dlc = 8; f.data[0] = b0; return f;
}
}

TEST(CANDeviceCollection, DispatchRoutesToCorrectDevice) {
    safemrc::can::CANDeviceCollection col;
    auto d1 = std::make_shared<MockDevice>(0x101, 0x201);
    auto d2 = std::make_shared<MockDevice>(0x102, 0x202);
    col.add_device(d1); col.add_device(d2);
    col.dispatch(make_frame(0x201, 0xAA));
    EXPECT_EQ(d1->frame_count, 1); EXPECT_EQ(d1->last_byte0, 0xAA);
    EXPECT_EQ(d2->frame_count, 0);
}

TEST(CANDeviceCollection, DispatchIgnoresUnknownId) {
    safemrc::can::CANDeviceCollection col;
    auto d = std::make_shared<MockDevice>(0x101, 0x201);
    col.add_device(d);
    col.dispatch(make_frame(0x999, 0xFF));
    EXPECT_EQ(d->frame_count, 0);
}

TEST(CANDeviceCollection, RemoveDevice) {
    safemrc::can::CANDeviceCollection col;
    auto d = std::make_shared<MockDevice>(0x101, 0x201);
    col.add_device(d); col.remove_device(0x201);
    col.dispatch(make_frame(0x201, 0x01));
    EXPECT_EQ(d->frame_count, 0); EXPECT_TRUE(col.devices().empty());
}

TEST(CANDeviceCollection, DispatchFdFrame) {
    safemrc::can::CANDeviceCollection col;
    auto d = std::make_shared<MockDevice>(0x101, 0x201);
    col.add_device(d);
    canfd_frame f{}; f.can_id = 0x201; f.len = 8; f.data[0] = 0xBB;
    col.dispatch(f);
    EXPECT_EQ(d->frame_count, 1); EXPECT_EQ(d->last_byte0, 0xBB);
}
