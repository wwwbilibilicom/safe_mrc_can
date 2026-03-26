# SafeMRC CAN Driver Library — Design Spec

> Date: 2026-03-26
> Status: Approved

## Overview

A C++17 CAN driver library for SafeMRC bearing lock controllers, with Python bindings via nanobind. Communicates over Linux SocketCAN (Classic CAN 2.0B and CAN FD). Supports multi-device management with a background receive thread.

Target platforms: Linux x86_64 (development) and ARM64 (Raspberry Pi deployment).

## Protocol Summary

Source: `docs/CAN_Protocol_Handoff.md`

- **Bus**: Classic CAN 2.0B / CAN FD, 1 Mbps, 11-bit standard ID, 8-byte fixed-length frames
- **Pattern**: Request-response at 1 kHz. Host sends command, device replies immediately from ISR.
- **CAN ID scheme**: Command `0x100 | device_id`, Feedback `0x200 | device_id`. device_id range: 1–254.
- **Byte order**: Little-endian for all multi-byte fields.

### Command Frame (Host -> Device)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | mode | uint8_t | MRCMode enum |
| 1 | reserved | uint8_t | 0x00 |
| 2–5 | des_coil_current | int32_t LE | Target coil current in mA (×1000 scaling) |
| 6–7 | reserved2 | uint8_t[2] | 0x00 |

### Feedback Frame (Device -> Host)

| Offset | Field | Type | Description |
|--------|-------|------|-------------|
| 0 | mode | uint8_t | Current MRCMode |
| 1 | collision_flag | uint8_t | 0x00=safe, 0x01=collision |
| 2–5 | encoder_value | int32_t LE | Position = value / 65535.0 (rad) |
| 6–7 | present_current | int16_t LE | Current = value / 1000.0 (A) |

### Work Modes

| Mode | Value | des_coil_current | Behavior |
|------|-------|------------------|----------|
| FREE | 0 | ignored | Coil de-energized |
| FIX_LIMIT | 1 | **active** | Fixed braking torque |
| ADAPTATION | 2 | **active** | Adaptive control + collision detection |
| DEBUG | 3 | ignored | CLI debug, CAN commands ignored |
| MRC_RESET | 4 | ignored | Clear collision flag, demagnetize |
| ZERO | 5 | ignored | Set encoder zero point |
| REFRESH | 6 | ignored | Reserved |

### Collision Protection

In ADAPTATION mode, when collision is detected: `collision_flag` set to 0x01, device auto-demagnetizes, only responds to MRC_RESET until cleared.

## Architecture

Four-layer design, referencing openarm_can's architecture with adaptations for SafeMRC's simpler protocol and background-thread receive model.

```
┌─────────────────────────────────────────┐
│  SafeMRC  (Facade + background thread)  │
├─────────────────────────────────────────┤
│  MRCDevice / MRCState / MRCProtocol     │  Protocol Layer
├─────────────────────────────────────────┤
│  CANDevice / CANDeviceCollection        │  Device Abstraction
├─────────────────────────────────────────┤
│  CANSocket                              │  Transport Layer
└─────────────────────────────────────────┘
```

### Layer 1: Transport — `CANSocket`

RAII wrapper over Linux SocketCAN. Move-only, no copy.

```cpp
class CANSocket {
public:
    CANSocket(const std::string& interface, bool use_fd = false);
    ~CANSocket();

    CANSocket(CANSocket&&) noexcept;
    CANSocket& operator=(CANSocket&&) noexcept;
    CANSocket(const CANSocket&) = delete;

    int write_frame(const can_frame& frame);
    int write_frame(const canfd_frame& frame);
    int read_frame(can_frame& frame);
    int read_frame(canfd_frame& frame);

    bool is_data_available(int timeout_us = 0);
    bool is_fd() const;
    int fd() const;
};
```

Throws `CANSocketException` on initialization failure (interface not found, bind error).

### Layer 2: Device Abstraction — `CANDevice` + `CANDeviceCollection`

```cpp
class CANDevice {
public:
    CANDevice(canid_t send_id, canid_t recv_id, canid_t recv_mask, bool use_fd);
    virtual ~CANDevice() = default;

    virtual void on_frame(const can_frame& frame) = 0;
    virtual void on_frame(const canfd_frame& frame) = 0;

    canid_t send_id() const;
    canid_t recv_id() const;
    canid_t recv_mask() const;
    bool is_fd() const;
};

class CANDeviceCollection {
public:
    void add_device(std::shared_ptr<CANDevice> device);
    void remove_device(canid_t recv_id);
    void dispatch(const can_frame& frame);
    void dispatch(const canfd_frame& frame);
private:
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
};
```

Dispatch silently ignores frames with no matching device (normal on shared CAN bus).

### Layer 3: SafeMRC Protocol

#### `MRCProtocol` — Static encoder/decoder

```cpp
enum class MRCMode : uint8_t {
    FREE = 0, FIX_LIMIT = 1, ADAPTATION = 2,
    DEBUG = 3, MRC_RESET = 4, ZERO = 5, REFRESH = 6
};

struct MRCCommand {
    MRCMode mode;
    int32_t des_coil_current;  // mA
};

struct MRCFeedback {
    MRCMode mode;
    bool collision;
    int32_t encoder_value;    // raw
    int16_t present_current;  // mA
    double position_rad() const;  // encoder_value / 65535.0
    double current_amp() const;   // present_current / 1000.0
};

class MRCProtocol {
public:
    static canid_t cmd_id(uint8_t device_id);
    static canid_t fbk_id(uint8_t device_id);

    static can_frame   encode_cmd(uint8_t device_id, const MRCCommand& cmd);
    static canfd_frame encode_cmd_fd(uint8_t device_id, const MRCCommand& cmd);
    static MRCFeedback decode_fbk(const can_frame& frame);
    static MRCFeedback decode_fbk(const canfd_frame& frame);

    // Convenience factories
    static MRCCommand free_cmd();
    static MRCCommand fix_limit_cmd(double current_amp);
    static MRCCommand adaptation_cmd(double current_amp);
    static MRCCommand reset_cmd();
    static MRCCommand zero_cmd();
};
```

#### `MRCState` — Thread-safe state container

```cpp
class MRCState {
public:
    MRCState(uint8_t device_id);

    MRCFeedback feedback() const;       // mutex-protected read
    uint8_t device_id() const;
    MRCMode mode() const;
    bool collision() const;
    double position_rad() const;
    double current_amp() const;

    std::chrono::steady_clock::time_point last_update_time() const;
    bool is_stale(std::chrono::milliseconds timeout = std::chrono::milliseconds(5)) const;

    void update(const MRCFeedback& fbk);  // called by MRCDevice callback

private:
    mutable std::mutex mutex_;
    MRCFeedback latest_;
    std::chrono::steady_clock::time_point last_update_;
};
```

#### `MRCDevice` — Concrete CANDevice

```cpp
class MRCDevice : public CANDevice {
public:
    MRCDevice(uint8_t device_id, bool use_fd);

    void on_frame(const can_frame& frame) override;
    void on_frame(const canfd_frame& frame) override;

    const MRCState& state() const;
    uint8_t device_id() const;

private:
    uint8_t device_id_;
    MRCState state_;
};
```

### Layer 4: Facade — `SafeMRC`

```cpp
class SafeMRC {
public:
    SafeMRC(const std::string& interface, bool use_fd = false);
    ~SafeMRC();

    // Device management
    void add_device(uint8_t device_id);
    void remove_device(uint8_t device_id);

    // Commands
    void send_command(uint8_t device_id, const MRCCommand& cmd);
    void send_command_all(const MRCCommand& cmd);

    // Convenience
    void set_free(uint8_t device_id);
    void set_free_all();
    void set_fix_limit(uint8_t device_id, double current_amp);
    void set_adaptation(uint8_t device_id, double current_amp);
    void reset_collision(uint8_t device_id);
    void set_zero(uint8_t device_id);

    // State queries (thread-safe)
    const MRCState& device_state(uint8_t device_id) const;
    std::vector<uint8_t> device_ids() const;
    size_t device_count() const;

    // Background thread
    void start();
    void stop();
    bool is_running() const;

private:
    CANSocket socket_;
    CANDeviceCollection devices_;
    std::map<uint8_t, std::shared_ptr<MRCDevice>> mrc_devices_;

    std::thread recv_thread_;
    std::atomic<bool> running_{false};
    void recv_loop();
};
```

**Background receive thread** (`recv_loop`): Loops while `running_`, calls `socket_.is_data_available(1000)` (1ms timeout), reads frame, dispatches via `devices_.dispatch()`. Each device's `on_frame` decodes the feedback and updates its `MRCState` under mutex.

**Send thread safety**: `write_frame()` is called from the user's thread. Linux SocketCAN `write()` is atomic for single frames. No mutex needed at 1kHz single-thread sending. If future multi-thread sending is needed, a mutex can be added around socket writes.

## Python Bindings

Package name: `safemrc_can`. Built with scikit-build-core + nanobind.

Exposed surface:
- `MRCMode` enum
- `MRCCommand`, `MRCFeedback` (with `position_rad`, `current_amp` as properties)
- `MRCState` (read-only properties)
- `SafeMRC` class (all public methods)

Transport layer (`CANSocket`, `CANDevice`, etc.) is NOT exposed — Python users only interact with the Facade.

```python
import safemrc_can as mrc

sm = mrc.SafeMRC("can0", use_fd=False)
sm.add_device(1)
sm.add_device(2)
sm.start()

sm.set_fix_limit(1, 3.0)
state = sm.device_state(1)
print(f"pos={state.position_rad:.4f} collision={state.collision}")

sm.set_free_all()
sm.stop()
```

## Build System

CMake 3.22+, C++17 required.

```
safe-mrc-can/
├── include/safemrc/
│   ├── can/
│   │   ├── can_socket.hpp
│   │   ├── can_device.hpp
│   │   └── can_device_collection.hpp
│   ├── mrc/
│   │   ├── mrc_protocol.hpp
│   │   ├── mrc_device.hpp
│   │   └── mrc_state.hpp
│   └── safemrc.hpp
├── src/safemrc/
│   ├── can/
│   │   ├── can_socket.cpp
│   │   └── can_device_collection.cpp
│   ├── mrc/
│   │   ├── mrc_protocol.cpp
│   │   ├── mrc_device.cpp
│   │   └── mrc_state.cpp
│   └── safemrc.cpp
├── python/
│   ├── src/safemrc_can.cpp
│   ├── examples/demo.py
│   └── pyproject.toml
├── examples/demo.cpp
├── docs/
├── CMakeLists.txt
└── LICENSE
```

**Targets:**
- `safemrc_can` — shared library, links pthread
- `safemrc_demo` — example executable
- Install: headers, lib, CMake config, pkg-config

**Compiler flags:** `-Wall -Wextra -Wpedantic`

**Dependencies:** Linux kernel headers (`<linux/can.h>`), pthread. No third-party libraries.

## Error Handling

- `CANSocketException` on socket init failure (interface not found, bind error, FD mode unsupported)
- `std::out_of_range` when accessing non-existent device_id
- `MRCState::is_stale()` for detecting device timeout/disconnection
- Unknown CAN IDs silently ignored in dispatch (normal on shared bus)
- Collision state exposed via `MRCState::collision()` — upper layer decides recovery logic

## Testing Strategy

- Unit tests for `MRCProtocol` encode/decode (no hardware needed)
- Unit tests for `MRCState` thread safety (concurrent read/write)
- Integration tests with virtual CAN (`vcan0`) for full send/receive cycle
- Python binding smoke tests
