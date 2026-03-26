#pragma once

#include <safemrc/can/can_device_collection.hpp>
#include <safemrc/can/can_socket.hpp>
#include <safemrc/mrc/mrc_device.hpp>
#include <safemrc/mrc/mrc_protocol.hpp>
#include <safemrc/mrc/mrc_state.hpp>

#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace safemrc {

class SafeMRC {
public:
    explicit SafeMRC(const std::string& interface, bool use_fd = false);
    ~SafeMRC();

    SafeMRC(const SafeMRC&) = delete;
    SafeMRC& operator=(const SafeMRC&) = delete;

    void add_device(uint8_t device_id);
    void remove_device(uint8_t device_id);

    void send_command(uint8_t device_id, const mrc::MRCCommand& cmd);
    void send_command_all(const mrc::MRCCommand& cmd);

    void set_free(uint8_t device_id);
    void set_free_all();
    void set_fix_limit(uint8_t device_id, double current_amp);
    void set_adaptation(uint8_t device_id, double current_amp);
    void reset_collision(uint8_t device_id);
    void set_zero(uint8_t device_id);

    const mrc::MRCState& device_state(uint8_t device_id) const;
    std::vector<uint8_t> device_ids() const;
    size_t device_count() const;

    void start();
    void stop();
    bool is_running() const { return running_.load(); }

private:
    can::CANSocket socket_;
    can::CANDeviceCollection devices_;
    std::map<uint8_t, std::shared_ptr<mrc::MRCDevice>> mrc_devices_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    void recv_loop();

    void send_frame(uint8_t device_id, const mrc::MRCCommand& cmd);
};

}  // namespace safemrc
