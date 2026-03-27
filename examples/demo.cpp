#include <safemrc/safemrc.hpp>

#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char* argv[]) {
    const char* interface = (argc > 1) ? argv[1] : "can0";
    bool use_fd = false;

    try {
        safemrc::SafeMRC mrc(interface, use_fd);
        mrc.add_device(1);
        printf("Added device 1 on %s (FD=%s)\n", interface, use_fd ? "true" : "false");

        mrc.start();

        mrc.set_zero(1);
        printf("Sent ZERO command\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        printf("Setting FIX_LIMIT mode at 2.0 A\n");
        for (int i = 0; i < 100; ++i) {
            mrc.set_fix_limit(1, 2.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            auto& s = mrc.device_state(1);
            if (!s.is_stale()) {
                printf("[%3d] mode=%d collision=%d pos=%.4f rad cur=%.3f A\n",
                       i, static_cast<int>(s.mode()), s.collision(),
                       s.position_rad(), s.current_amp());
            }
        }

        if (mrc.device_state(1).collision()) {
            printf("Collision detected! Resetting...\n");
            mrc.reset_collision(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        mrc.set_free(1);
        printf("Released (FREE mode)\n");
        mrc.stop();

    } catch (const safemrc::can::CANSocketException& e) {
        fprintf(stderr, "CAN error: %s\n", e.what());
        return 1;
    }

    return 0;
}
