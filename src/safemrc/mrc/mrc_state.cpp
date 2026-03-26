#include <safemrc/mrc/mrc_state.hpp>

namespace safemrc::mrc {

MRCState::MRCState(uint8_t device_id) : device_id_(device_id) {}

MRCFeedback MRCState::feedback() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_;
}

MRCMode MRCState::mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_.mode;
}

bool MRCState::collision() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_.collision;
}

double MRCState::position_rad() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_.position_rad();
}

double MRCState::current_amp() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_.current_amp();
}

std::chrono::steady_clock::time_point MRCState::last_update_time() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_update_;
}

bool MRCState::is_stale(std::chrono::milliseconds timeout) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (last_update_ == std::chrono::steady_clock::time_point{}) return true;
    return (std::chrono::steady_clock::now() - last_update_) > timeout;
}

void MRCState::update(const MRCFeedback& fbk) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = fbk;
    last_update_ = std::chrono::steady_clock::now();
}

}  // namespace safemrc::mrc
