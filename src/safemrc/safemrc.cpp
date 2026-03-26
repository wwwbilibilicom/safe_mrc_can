#include <safemrc/safemrc.hpp>

#include <stdexcept>

namespace safemrc {

SafeMRC::SafeMRC(const std::string& interface, bool use_fd)
    : socket_(interface, use_fd) {}

SafeMRC::~SafeMRC() { stop(); }

void SafeMRC::add_device(uint8_t device_id) {
    if (mrc_devices_.count(device_id)) return;
    auto dev = std::make_shared<mrc::MRCDevice>(device_id, socket_.is_fd_enabled());
    mrc_devices_[device_id] = dev;
    devices_.add_device(dev);
}

void SafeMRC::remove_device(uint8_t device_id) {
    auto it = mrc_devices_.find(device_id);
    if (it == mrc_devices_.end()) return;
    devices_.remove_device(it->second->recv_id());
    mrc_devices_.erase(it);
}

void SafeMRC::send_frame(uint8_t device_id, const mrc::MRCCommand& cmd) {
    if (socket_.is_fd_enabled()) {
        auto frame = mrc::MRCProtocol::encode_cmd_fd(device_id, cmd);
        socket_.write_canfd_frame(frame);
    } else {
        auto frame = mrc::MRCProtocol::encode_cmd(device_id, cmd);
        socket_.write_can_frame(frame);
    }
}

void SafeMRC::send_command(uint8_t device_id, const mrc::MRCCommand& cmd) {
    if (!mrc_devices_.count(device_id)) {
        throw std::out_of_range("Device ID " + std::to_string(device_id) + " not found");
    }
    send_frame(device_id, cmd);
}

void SafeMRC::send_command_all(const mrc::MRCCommand& cmd) {
    for (auto& [id, _] : mrc_devices_) {
        send_frame(id, cmd);
    }
}

void SafeMRC::set_free(uint8_t device_id) {
    send_command(device_id, mrc::MRCProtocol::free_cmd());
}

void SafeMRC::set_free_all() {
    send_command_all(mrc::MRCProtocol::free_cmd());
}

void SafeMRC::set_fix_limit(uint8_t device_id, double current_amp) {
    send_command(device_id, mrc::MRCProtocol::fix_limit_cmd(current_amp));
}

void SafeMRC::set_adaptation(uint8_t device_id, double current_amp) {
    send_command(device_id, mrc::MRCProtocol::adaptation_cmd(current_amp));
}

void SafeMRC::reset_collision(uint8_t device_id) {
    send_command(device_id, mrc::MRCProtocol::reset_cmd());
}

void SafeMRC::set_zero(uint8_t device_id) {
    send_command(device_id, mrc::MRCProtocol::zero_cmd());
}

const mrc::MRCState& SafeMRC::device_state(uint8_t device_id) const {
    auto it = mrc_devices_.find(device_id);
    if (it == mrc_devices_.end()) {
        throw std::out_of_range("Device ID " + std::to_string(device_id) + " not found");
    }
    return it->second->state();
}

std::vector<uint8_t> SafeMRC::device_ids() const {
    std::vector<uint8_t> ids;
    ids.reserve(mrc_devices_.size());
    for (auto& [id, _] : mrc_devices_) ids.push_back(id);
    return ids;
}

size_t SafeMRC::device_count() const { return mrc_devices_.size(); }

void SafeMRC::start() {
    if (running_.load()) return;
    running_.store(true);
    recv_thread_ = std::thread(&SafeMRC::recv_loop, this);
}

void SafeMRC::stop() {
    running_.store(false);
    if (recv_thread_.joinable()) recv_thread_.join();
}

void SafeMRC::recv_loop() {
    while (running_.load()) {
        if (!socket_.is_data_available(1000)) continue;
        if (socket_.is_fd_enabled()) {
            canfd_frame frame{};
            if (socket_.read_canfd_frame(frame)) devices_.dispatch(frame);
        } else {
            can_frame frame{};
            if (socket_.read_can_frame(frame)) devices_.dispatch(frame);
        }
    }
}

}  // namespace safemrc
