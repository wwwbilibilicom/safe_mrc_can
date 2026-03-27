<div align="center">

# SafeMRC CAN Driver

**High-performance C++17 library for SafeMRC bearing lock controllers over CAN bus**

[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://isocpp.org/std/the-standard)
[![Platform](https://img.shields.io/badge/platform-Linux-lightgrey.svg)](https://www.kernel.org/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-yellow.svg)](https://www.python.org/)

[Features](#features) · [Quick Start](#quick-start) · [Documentation](#documentation) · [Examples](#examples) · [Contributing](#contributing)

</div>

---

## Overview

SafeMRC CAN Driver provides a clean, thread-safe C++ interface for communicating with SafeMRC magnetic rotor controller (MRC) bearing lock devices over Linux SocketCAN. It supports multi-device orchestration, real-time feedback collection via a background receive loop, and built-in collision detection — all at 1 kHz communication frequency.

```cpp
#include <safemrc/safemrc.hpp>

safemrc::SafeMRC mrc("can0");
mrc.add_device(1);
mrc.start();                    // background receive thread
mrc.set_fix_limit(1, 2.0);     // brake at 2.0 A
auto state = mrc.device_state(1);
std::cout << "Position: " << state.position_rad() << " rad\n";
mrc.stop();
```

## Features

- **Multi-device support** — manage up to 254 devices on a single CAN bus
- **Thread-safe** — background receive loop with mutex-protected state
- **Real-time feedback** — position (rad), current (A), collision flag at 1 kHz
- **Collision detection** — automatic demagnetization and safety lockout in adaptation mode
- **CAN FD ready** — supports both Classic CAN 2.0B and CAN FD frames
- **Python bindings** — full API exposed via nanobind
- **Zero dependencies** — pure Linux kernel SocketCAN, no third-party C++ libraries

## Quick Start

### Prerequisites

- Linux with SocketCAN support
- CMake 3.22+
- C++17 compatible compiler (GCC 9+ / Clang 10+)
- (Optional) Google Test for running tests
- (Optional) Python 3.10+ and nanobind for Python bindings

### Build

```bash
git clone https://github.com/your-org/safe-mrc-can.git
cd safe-mrc-can
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### Install

```bash
sudo make install
```

This installs headers, the shared library, CMake config files, and pkg-config support.

### Run Tests

```bash
# Set up virtual CAN interface for integration tests
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Build and run
cmake -DBUILD_TESTING=ON ..
make -j$(nproc)
ctest --output-on-failure
```

## Documentation

### Architecture

The library follows a clean four-layer architecture:

```
┌─────────────────────────────────┐
│          SafeMRC (Facade)       │  ← User-facing API, background thread
├─────────────────────────────────┤
│   MRCDevice / MRCState          │  ← Device logic, thread-safe state
├─────────────────────────────────┤
│   MRCProtocol                   │  ← Frame encode/decode, mode enums
├─────────────────────────────────┤
│   CANSocket / CANDeviceCollection│ ← Transport, CAN ID routing
└─────────────────────────────────┘
```

### Work Modes

| Mode | Value | Description |
|------|-------|-------------|
| `FREE` | 0 | Coil de-energized, free rotation |
| `FIX_LIMIT` | 1 | Fixed braking torque at specified current |
| `ADAPTATION` | 2 | Adaptive control with collision detection |
| `DEBUG` | 3 | CLI debug mode (CAN commands ignored) |
| `MRC_RESET` | 4 | Clear collision flag, demagnetize |
| `ZERO` | 5 | Set current position as zero |

### CAN Frame Format

- **Command frames** (Host → Device): CAN ID `0x100 | device_id`
- **Feedback frames** (Device → Host): CAN ID `0x200 | device_id`
- 8-byte fixed-length, little-endian

For the full protocol specification, see [`docs/CAN_Protocol_Handoff.md`](docs/CAN_Protocol_Handoff.md).

## Examples

### C++

```cpp
#include <safemrc/safemrc.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    safemrc::SafeMRC mrc("can0");
    mrc.add_device(1);
    mrc.start();

    // Set zero position
    mrc.set_zero(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Apply braking torque
    mrc.set_fix_limit(1, 2.0);  // 2.0 Amps

    // Monitor state
    for (int i = 0; i < 100; ++i) {
        auto state = mrc.device_state(1);
        std::cout << "pos=" << state.position_rad() << " rad, "
                  << "cur=" << state.current_amp() << " A, "
                  << "collision=" << state.collision() << "\n";

        if (state.collision()) {
            mrc.reset_collision(1);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    mrc.set_free(1);
    mrc.stop();
}
```

### Python

```bash
pip install ./python
```

```python
import safemrc_can as mrc
import time

sm = mrc.SafeMRC("can0", use_fd=False)
sm.add_device(1)
sm.start()

sm.set_fix_limit(1, 3.0)  # 3.0 Amps

for _ in range(100):
    state = sm.device_state(1)
    print(f"Position: {state.position_rad:.4f} rad, "
          f"Current: {state.current_amp:.3f} A, "
          f"Collision: {state.collision}")
    time.sleep(0.01)

sm.set_free_all()
sm.stop()
```

### CMake Integration

```cmake
find_package(SafeMRCCAN REQUIRED)
target_link_libraries(your_target PRIVATE safemrc::safemrc_can)
```

### pkg-config

```bash
g++ main.cpp $(pkg-config --cflags --libs safemrc-can) -o main
```

## Project Structure

```
safe-mrc-can/
├── include/safemrc/          # Public headers
│   ├── can/                  #   Transport layer
│   ├── mrc/                  #   Protocol & device layer
│   └── safemrc.hpp           #   Facade (main entry point)
├── src/safemrc/              # Implementation
├── tests/                    # Google Test suite
├── examples/                 # C++ demo
├── python/                   # Python bindings (nanobind)
│   ├── src/                  #   Binding source
│   └── examples/             #   Python demo
├── docs/                     # Protocol spec & design docs
└── CMakeLists.txt            # Build configuration
```

## API Reference

### `SafeMRC` (Facade)

| Method | Description |
|--------|-------------|
| `SafeMRC(iface, use_fd)` | Create instance on CAN interface |
| `add_device(id)` | Register a device by ID (1–254) |
| `remove_device(id)` | Unregister a device |
| `start()` / `stop()` | Start/stop background receive thread |
| `send_command(id, cmd)` | Send raw command to device |
| `set_free(id)` | Release device (de-energize coil) |
| `set_fix_limit(id, amps)` | Set fixed braking torque |
| `set_adaptation(id, amps)` | Enable adaptive mode with collision detection |
| `reset_collision(id)` | Clear collision flag |
| `set_zero(id)` | Set current position as zero reference |
| `device_state(id)` | Get thread-safe snapshot of device state |
| `device_ids()` | List all registered device IDs |

### `MRCState`

| Method | Description |
|--------|-------------|
| `position_rad()` | Current position in radians |
| `current_amp()` | Present coil current in amps |
| `collision()` | Collision flag (true/false) |
| `mode()` | Current work mode |
| `is_stale(timeout)` | Check if feedback is outdated |

## Contributing

Contributions are welcome! Here's how to get started:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/your-feature`)
3. Make your changes and add tests
4. Ensure all tests pass (`ctest --output-on-failure`)
5. Commit your changes (`git commit -m 'feat: add your feature'`)
6. Push to the branch (`git push origin feature/your-feature`)
7. Open a Pull Request

### Development Setup

```bash
# Set up virtual CAN for testing without hardware
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

# Build with tests
mkdir build && cd build
cmake -DBUILD_TESTING=ON ..
make -j$(nproc)
ctest --output-on-failure
```

## License

This project is licensed under the Apache License 2.0 — see the [LICENSE](LICENSE) file for details.

---

<div align="center">

**[Report Bug](../../issues) · [Request Feature](../../issues)**

</div>
