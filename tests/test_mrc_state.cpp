#include <gtest/gtest.h>
#include <safemrc/mrc/mrc_state.hpp>
#include <thread>
#include <vector>

using namespace safemrc::mrc;

TEST(MRCState, InitialState) {
    MRCState state(1);
    EXPECT_EQ(state.device_id(), 1);
    EXPECT_EQ(state.mode(), MRCMode::FREE);
    EXPECT_FALSE(state.collision());
    EXPECT_DOUBLE_EQ(state.position_rad(), 0.0);
    EXPECT_DOUBLE_EQ(state.current_amp(), 0.0);
    EXPECT_TRUE(state.is_stale());
}

TEST(MRCState, UpdateAndRead) {
    MRCState state(2);
    MRCFeedback fbk;
    fbk.mode = MRCMode::ADAPTATION; fbk.collision = true;
    fbk.encoder_value = 65535; fbk.present_current = 3500;
    state.update(fbk);
    EXPECT_EQ(state.mode(), MRCMode::ADAPTATION); EXPECT_TRUE(state.collision());
    EXPECT_NEAR(state.position_rad(), 1.0, 1e-4);
    EXPECT_NEAR(state.current_amp(), 3.5, 1e-6);
    EXPECT_FALSE(state.is_stale());
}

TEST(MRCState, FeedbackCopy) {
    MRCState state(3);
    MRCFeedback fbk; fbk.mode = MRCMode::FIX_LIMIT; fbk.encoder_value = 1000; fbk.present_current = -500;
    state.update(fbk);
    auto copy = state.feedback();
    EXPECT_EQ(copy.mode, MRCMode::FIX_LIMIT); EXPECT_EQ(copy.encoder_value, 1000);
}

TEST(MRCState, StalenessAfterTimeout) {
    MRCState state(4); MRCFeedback fbk{}; state.update(fbk);
    EXPECT_FALSE(state.is_stale(std::chrono::milliseconds(100)));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(state.is_stale(std::chrono::milliseconds(10)));
}

TEST(MRCState, ConcurrentReadWrite) {
    MRCState state(5);
    constexpr int iterations = 10000;
    std::thread writer([&] {
        for (int i = 0; i < iterations; ++i) {
            MRCFeedback fbk; fbk.mode = MRCMode::FIX_LIMIT;
            fbk.encoder_value = i; fbk.present_current = static_cast<int16_t>(i % 1000);
            state.update(fbk);
        }
    });
    std::thread reader([&] {
        for (int i = 0; i < iterations; ++i) {
            auto fbk = state.feedback();
            (void)fbk.position_rad(); (void)fbk.current_amp();
        }
    });
    writer.join(); reader.join();
    EXPECT_EQ(state.feedback().mode, MRCMode::FIX_LIMIT);
}
