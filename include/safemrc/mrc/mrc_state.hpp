#pragma once

#include <safemrc/mrc/mrc_protocol.hpp>

#include <chrono>
#include <mutex>

namespace safemrc::mrc {

class MRCState {
public:
    explicit MRCState(uint8_t device_id);

    MRCFeedback feedback() const;
    uint8_t device_id() const { return device_id_; }
    MRCMode mode() const;
    bool collision() const;
    double position_rad() const;
    double current_amp() const;

    std::chrono::steady_clock::time_point last_update_time() const;
    bool is_stale(std::chrono::milliseconds timeout = std::chrono::milliseconds(5)) const;

    void update(const MRCFeedback& fbk);

private:
    uint8_t device_id_;
    mutable std::mutex mutex_;
    MRCFeedback latest_{};
    std::chrono::steady_clock::time_point last_update_{};
};

}  // namespace safemrc::mrc
