# SafeMRC CAN Driver Library Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a C++17 CAN driver library for SafeMRC bearing lock controllers with Python bindings, supporting CAN 2.0B and CAN FD over Linux SocketCAN.

**Architecture:** Four-layer design — Transport (CANSocket), Device Abstraction (CANDevice/CANDeviceCollection), Protocol (MRCProtocol/MRCState/MRCDevice), and Facade (SafeMRC with background receive thread). Python bindings via nanobind expose only the Facade layer.

**Tech Stack:** C++17, CMake 3.22+, Linux SocketCAN, pthread, nanobind, scikit-build-core

**Reference project:** `/home/wenbo-li/lab_dev/openarm_can` — follow its patterns for CANSocket, CANDevice, CANDeviceCollection, CMake structure, and nanobind bindings.

---

## File Structure

```
safe-mrc-can/
├── CMakeLists.txt                              # Main build config
├── safemrc-can.pc.in                           # pkg-config template
├── SafeMRCCANConfig.cmake.in                   # CMake package config template
├── include/safemrc/
│   ├── can/
│   │   ├── can_socket.hpp                      # RAII SocketCAN wrapper
│   │   ├── can_device.hpp                      # Abstract CAN device base class
│   │   └── can_device_collection.hpp           # CAN ID routing/dispatch
│   ├── mrc/
│   │   ├── mrc_protocol.hpp                    # Frame encode/decode + mode enum + structs
│   │   ├── mrc_state.hpp                       # Thread-safe device state container
│   │   └── mrc_device.hpp                      # Concrete CANDevice for SafeMRC
│   └── safemrc.hpp                             # Top-level facade with background thread
├── src/safemrc/
│   ├── can/
│   │   ├── can_socket.cpp
│   │   └── can_device_collection.cpp
│   ├── mrc/
│   │   ├── mrc_protocol.cpp
│   │   ├── mrc_state.cpp
│   │   └── mrc_device.cpp
│   └── safemrc.cpp
├── tests/
│   ├── CMakeLists.txt                          # Test build config
│   ├── test_mrc_protocol.cpp                   # Protocol encode/decode tests
│   ├── test_mrc_state.cpp                      # Thread-safe state tests
│   ├── test_can_device_collection.cpp          # Device dispatch tests
│   └── test_integration.cpp                    # Full vcan integration test
├── examples/
│   └── demo.cpp                                # C++ usage example
├── python/
│   ├── CMakeLists.txt                          # Python binding build
│   ├── pyproject.toml                          # Python package config
│   └── src/
│       └── safemrc_can.cpp                     # nanobind bindings
└── docs/
```

---

### Task 1: Project Scaffold — CMake + Directory Structure

**Files:**
- Create: `CMakeLists.txt`
- Create: `safemrc-can.pc.in`
- Create: `SafeMRCCANConfig.cmake.in`
- Create: `tests/CMakeLists.txt`

- [ ] **Step 1: Create root CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.22)
project(safemrc_can VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(CTest)
include(GNUInstallDirs)

add_library(
  safemrc_can
  src/safemrc/can/can_socket.cpp
  src/safemrc/can/can_device_collection.cpp
  src/safemrc/mrc/mrc_protocol.cpp
  src/safemrc/mrc/mrc_state.cpp
  src/safemrc/mrc/mrc_device.cpp
  src/safemrc/safemrc.cpp)
set_target_properties(
  safemrc_can
  PROPERTIES POSITION_INDEPENDENT_CODE ON
             VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR})
target_include_directories(
  safemrc_can PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_link_libraries(safemrc_can PRIVATE pthread)

# Install
install(TARGETS safemrc_can EXPORT safemrc_can_export)
install(DIRECTORY include/safemrc TYPE INCLUDE)

# CMake package
set(INSTALL_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/SafeMRCCAN)
install(
  EXPORT safemrc_can_export
  DESTINATION ${INSTALL_CMAKE_DIR}
  NAMESPACE SafeMRCCAN::
  FILE SafeMRCCANTargets.cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  SafeMRCCANConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/SafeMRCCANConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/SafeMRCCANConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/SafeMRCCANConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/SafeMRCCANConfigVersion.cmake
        DESTINATION ${INSTALL_CMAKE_DIR})

# pkg-config
if(IS_ABSOLUTE "${CMAKE_INSTALL_INCLUDEDIR}")
  set(PKG_CONFIG_INCLUDEDIR "${CMAKE_INSTALL_INCLUDEDIR}")
else()
  set(PKG_CONFIG_INCLUDEDIR "\${prefix}/${CMAKE_INSTALL_INCLUDEDIR}")
endif()
if(IS_ABSOLUTE "${CMAKE_INSTALL_LIBDIR}")
  set(PKG_CONFIG_LIBDIR "${CMAKE_INSTALL_LIBDIR}")
else()
  set(PKG_CONFIG_LIBDIR "\${prefix}/${CMAKE_INSTALL_LIBDIR}")
endif()
configure_file(safemrc-can.pc.in ${CMAKE_CURRENT_BINARY_DIR}/safemrc-can.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/safemrc-can.pc
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)

# Example
add_executable(safemrc-demo examples/demo.cpp)
target_link_libraries(safemrc-demo safemrc_can)

# Tests
if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
```

- [ ] **Step 2: Create SafeMRCCANConfig.cmake.in**

```cmake
@PACKAGE_INIT@
include("${CMAKE_CURRENT_LIST_DIR}/SafeMRCCANTargets.cmake")
check_required_components(SafeMRCCAN)
```

- [ ] **Step 3: Create safemrc-can.pc.in**

```
prefix=@CMAKE_INSTALL_PREFIX@
libdir=@PKG_CONFIG_LIBDIR@
includedir=@PKG_CONFIG_INCLUDEDIR@

Name: safemrc-can
Description: SafeMRC CAN driver library
Version: @PROJECT_VERSION@
Libs: -L${libdir} -lsafemrc_can
Cflags: -I${includedir}
```

- [ ] **Step 4: Create tests/CMakeLists.txt**

```cmake
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0)
FetchContent_MakeAvailable(googletest)

function(safemrc_add_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} safemrc_can GTest::gtest_main)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

safemrc_add_test(test_mrc_protocol)
safemrc_add_test(test_mrc_state)
safemrc_add_test(test_can_device_collection)
safemrc_add_test(test_integration)
```

- [ ] **Step 5: Create all source directories and stub files**

Create empty directories and minimal stub files so CMake can configure. Every `.cpp` file listed in `CMakeLists.txt` must exist (can be empty or have just a comment). Every `.hpp` file must exist with just `#pragma once`.

Stub headers — each contains only `#pragma once`:
- `include/safemrc/can/can_socket.hpp`
- `include/safemrc/can/can_device.hpp`
- `include/safemrc/can/can_device_collection.hpp`
- `include/safemrc/mrc/mrc_protocol.hpp`
- `include/safemrc/mrc/mrc_state.hpp`
- `include/safemrc/mrc/mrc_device.hpp`
- `include/safemrc/safemrc.hpp`

Stub sources — each contains only `// stub`:
- `src/safemrc/can/can_socket.cpp`
- `src/safemrc/can/can_device_collection.cpp`
- `src/safemrc/mrc/mrc_protocol.cpp`
- `src/safemrc/mrc/mrc_state.cpp`
- `src/safemrc/mrc/mrc_device.cpp`
- `src/safemrc/safemrc.cpp`

Stub test files — each contains:
```cpp
#include <gtest/gtest.h>
TEST(Stub, Placeholder) { EXPECT_TRUE(true); }
```

Stub example:
```cpp
int main() { return 0; }
```

- [ ] **Step 6: Verify build**

Run:
```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can
mkdir -p build && cd build
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)
ctest --output-on-failure
```

Expected: All targets build, 4 stub tests pass.

- [ ] **Step 7: Commit**

```bash
git add -A
git commit -m "feat: project scaffold with CMake, test infrastructure, and stub files"
```

---

### Task 2: CANSocket — Transport Layer

**Files:**
- Modify: `include/safemrc/can/can_socket.hpp`
- Modify: `src/safemrc/can/can_socket.cpp`

- [ ] **Step 1: Write can_socket.hpp**

```cpp
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdexcept>
#include <string>

namespace safemrc::can {

class CANSocketException : public std::runtime_error {
public:
    explicit CANSocketException(const std::string& message)
        : std::runtime_error("CAN socket error: " + message) {}
};

class CANSocket {
public:
    explicit CANSocket(const std::string& interface, bool enable_fd = false);
    ~CANSocket();

    CANSocket(const CANSocket&) = delete;
    CANSocket& operator=(const CANSocket&) = delete;
    CANSocket(CANSocket&& other) noexcept;
    CANSocket& operator=(CANSocket&& other) noexcept;

    bool write_can_frame(const can_frame& frame);
    bool write_canfd_frame(const canfd_frame& frame);
    bool read_can_frame(can_frame& frame);
    bool read_canfd_frame(canfd_frame& frame);

    bool is_data_available(int timeout_us = 0);
    bool is_fd_enabled() const { return fd_enabled_; }
    int fd() const { return socket_fd_; }
    const std::string& interface_name() const { return interface_; }

private:
    bool initialize(const std::string& interface);
    void cleanup();

    int socket_fd_ = -1;
    std::string interface_;
    bool fd_enabled_ = false;
};

}  // namespace safemrc::can
```

- [ ] **Step 2: Write can_socket.cpp**

```cpp
#include <safemrc/can/can_socket.hpp>

#include <net/if.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

namespace safemrc::can {

CANSocket::CANSocket(const std::string& interface, bool enable_fd)
    : interface_(interface), fd_enabled_(enable_fd) {
    if (!initialize(interface)) {
        throw CANSocketException("Failed to initialize socket for interface: " + interface);
    }
}

CANSocket::~CANSocket() { cleanup(); }

CANSocket::CANSocket(CANSocket&& other) noexcept
    : socket_fd_(other.socket_fd_),
      interface_(std::move(other.interface_)),
      fd_enabled_(other.fd_enabled_) {
    other.socket_fd_ = -1;
}

CANSocket& CANSocket::operator=(CANSocket&& other) noexcept {
    if (this != &other) {
        cleanup();
        socket_fd_ = other.socket_fd_;
        interface_ = std::move(other.interface_);
        fd_enabled_ = other.fd_enabled_;
        other.socket_fd_ = -1;
    }
    return *this;
}

bool CANSocket::initialize(const std::string& interface) {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) return false;

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';

    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        cleanup();
        return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (fd_enabled_) {
        int enable = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                       sizeof(enable)) < 0) {
            cleanup();
            return false;
        }
    }

    if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        cleanup();
        return false;
    }

    struct timeval timeout{};
    timeout.tv_sec = 0;
    timeout.tv_usec = 100;
    if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) < 0) {
        cleanup();
        return false;
    }

    return true;
}

void CANSocket::cleanup() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CANSocket::write_can_frame(const can_frame& frame) {
    return write(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::write_canfd_frame(const canfd_frame& frame) {
    return write(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::read_can_frame(can_frame& frame) {
    if (socket_fd_ < 0) return false;
    return read(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::read_canfd_frame(canfd_frame& frame) {
    if (socket_fd_ < 0) return false;
    return read(socket_fd_, &frame, sizeof(frame)) == sizeof(frame);
}

bool CANSocket::is_data_available(int timeout_us) {
    if (socket_fd_ < 0) return false;

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd_, &read_fds);

    struct timeval timeout{};
    timeout.tv_sec = timeout_us / 1000000;
    timeout.tv_usec = timeout_us % 1000000;

    int result = select(socket_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    return result > 0 && FD_ISSET(socket_fd_, &read_fds);
}

}  // namespace safemrc::can
```

- [ ] **Step 3: Verify build**

Run:
```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
```

Expected: Builds without error.

- [ ] **Step 4: Commit**

```bash
git add include/safemrc/can/can_socket.hpp src/safemrc/can/can_socket.cpp
git commit -m "feat: implement CANSocket transport layer with CAN FD support"
```

---

### Task 3: CANDevice + CANDeviceCollection — Device Abstraction

**Files:**
- Modify: `include/safemrc/can/can_device.hpp`
- Modify: `include/safemrc/can/can_device_collection.hpp`
- Modify: `src/safemrc/can/can_device_collection.cpp`
- Modify: `tests/test_can_device_collection.cpp`

- [ ] **Step 1: Write can_device.hpp**

```cpp
#pragma once

#include <linux/can.h>
#include <linux/can/raw.h>

#include <cstdint>

namespace safemrc::can {

class CANDevice {
public:
    explicit CANDevice(canid_t send_id, canid_t recv_id,
                       canid_t recv_mask = CAN_SFF_MASK, bool use_fd = false)
        : send_id_(send_id), recv_id_(recv_id), recv_mask_(recv_mask), use_fd_(use_fd) {}
    virtual ~CANDevice() = default;

    virtual void on_frame(const can_frame& frame) = 0;
    virtual void on_frame(const canfd_frame& frame) = 0;

    canid_t send_id() const { return send_id_; }
    canid_t recv_id() const { return recv_id_; }
    canid_t recv_mask() const { return recv_mask_; }
    bool is_fd() const { return use_fd_; }

protected:
    canid_t send_id_;
    canid_t recv_id_;
    canid_t recv_mask_;
    bool use_fd_;
};

}  // namespace safemrc::can
```

- [ ] **Step 2: Write can_device_collection.hpp**

```cpp
#pragma once

#include <map>
#include <memory>

#include <safemrc/can/can_device.hpp>

namespace safemrc::can {

class CANDeviceCollection {
public:
    void add_device(std::shared_ptr<CANDevice> device);
    void remove_device(canid_t recv_id);
    void dispatch(const can_frame& frame);
    void dispatch(const canfd_frame& frame);

    const std::map<canid_t, std::shared_ptr<CANDevice>>& devices() const { return devices_; }

private:
    std::map<canid_t, std::shared_ptr<CANDevice>> devices_;
};

}  // namespace safemrc::can
```

- [ ] **Step 3: Write can_device_collection.cpp**

```cpp
#include <safemrc/can/can_device_collection.hpp>

namespace safemrc::can {

void CANDeviceCollection::add_device(std::shared_ptr<CANDevice> device) {
    if (!device) return;
    devices_[device->recv_id()] = std::move(device);
}

void CANDeviceCollection::remove_device(canid_t recv_id) {
    devices_.erase(recv_id);
}

void CANDeviceCollection::dispatch(const can_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->on_frame(frame);
    }
}

void CANDeviceCollection::dispatch(const canfd_frame& frame) {
    auto it = devices_.find(frame.can_id);
    if (it != devices_.end()) {
        it->second->on_frame(frame);
    }
}

}  // namespace safemrc::can
```

- [ ] **Step 4: Write test_can_device_collection.cpp**

```cpp
#include <gtest/gtest.h>
#include <safemrc/can/can_device.hpp>
#include <safemrc/can/can_device_collection.hpp>

#include <cstring>

namespace {

class MockDevice : public safemrc::can::CANDevice {
public:
    MockDevice(canid_t send_id, canid_t recv_id)
        : CANDevice(send_id, recv_id), frame_count(0), last_byte0(0) {}

    void on_frame(const can_frame& frame) override {
        ++frame_count;
        last_byte0 = frame.data[0];
    }
    void on_frame(const canfd_frame& frame) override {
        ++frame_count;
        last_byte0 = frame.data[0];
    }

    int frame_count;
    uint8_t last_byte0;
};

can_frame make_frame(canid_t id, uint8_t byte0) {
    can_frame f{};
    f.can_id = id;
    f.can_dlc = 8;
    f.data[0] = byte0;
    return f;
}

}  // namespace

TEST(CANDeviceCollection, DispatchRoutesToCorrectDevice) {
    safemrc::can::CANDeviceCollection col;
    auto dev1 = std::make_shared<MockDevice>(0x101, 0x201);
    auto dev2 = std::make_shared<MockDevice>(0x102, 0x202);
    col.add_device(dev1);
    col.add_device(dev2);

    auto f = make_frame(0x201, 0xAA);
    col.dispatch(f);

    EXPECT_EQ(dev1->frame_count, 1);
    EXPECT_EQ(dev1->last_byte0, 0xAA);
    EXPECT_EQ(dev2->frame_count, 0);
}

TEST(CANDeviceCollection, DispatchIgnoresUnknownId) {
    safemrc::can::CANDeviceCollection col;
    auto dev = std::make_shared<MockDevice>(0x101, 0x201);
    col.add_device(dev);

    auto f = make_frame(0x999, 0xFF);
    col.dispatch(f);

    EXPECT_EQ(dev->frame_count, 0);
}

TEST(CANDeviceCollection, RemoveDevice) {
    safemrc::can::CANDeviceCollection col;
    auto dev = std::make_shared<MockDevice>(0x101, 0x201);
    col.add_device(dev);
    col.remove_device(0x201);

    auto f = make_frame(0x201, 0x01);
    col.dispatch(f);

    EXPECT_EQ(dev->frame_count, 0);
    EXPECT_TRUE(col.devices().empty());
}

TEST(CANDeviceCollection, DispatchFdFrame) {
    safemrc::can::CANDeviceCollection col;
    auto dev = std::make_shared<MockDevice>(0x101, 0x201);
    col.add_device(dev);

    canfd_frame f{};
    f.can_id = 0x201;
    f.len = 8;
    f.data[0] = 0xBB;
    col.dispatch(f);

    EXPECT_EQ(dev->frame_count, 1);
    EXPECT_EQ(dev->last_byte0, 0xBB);
}
```

- [ ] **Step 5: Run tests**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
ctest --output-on-failure -R test_can_device_collection
```

Expected: 4 tests pass.

- [ ] **Step 6: Commit**

```bash
git add include/safemrc/can/ src/safemrc/can/ tests/test_can_device_collection.cpp
git commit -m "feat: implement CANDevice base class and CANDeviceCollection dispatcher"
```

---

### Task 4: MRCProtocol — Protocol Encode/Decode

**Files:**
- Modify: `include/safemrc/mrc/mrc_protocol.hpp`
- Modify: `src/safemrc/mrc/mrc_protocol.cpp`
- Modify: `tests/test_mrc_protocol.cpp`

- [ ] **Step 1: Write mrc_protocol.hpp**

```cpp
#pragma once

#include <linux/can.h>

#include <cstdint>

namespace safemrc::mrc {

enum class MRCMode : uint8_t {
    FREE = 0,
    FIX_LIMIT = 1,
    ADAPTATION = 2,
    DEBUG = 3,
    MRC_RESET = 4,
    ZERO = 5,
    REFRESH = 6
};

struct MRCCommand {
    MRCMode mode = MRCMode::FREE;
    int32_t des_coil_current = 0;  // mA, x1000 scaling
};

struct MRCFeedback {
    MRCMode mode = MRCMode::FREE;
    bool collision = false;
    int32_t encoder_value = 0;     // raw, = rad * 65535
    int16_t present_current = 0;   // mA, x1000 scaling

    double position_rad() const { return static_cast<double>(encoder_value) / 65535.0; }
    double current_amp() const { return static_cast<double>(present_current) / 1000.0; }
};

class MRCProtocol {
public:
    static constexpr canid_t CMD_BASE = 0x100;
    static constexpr canid_t FBK_BASE = 0x200;

    static canid_t cmd_id(uint8_t device_id) { return CMD_BASE | device_id; }
    static canid_t fbk_id(uint8_t device_id) { return FBK_BASE | device_id; }

    // Encode command into CAN frames
    static can_frame encode_cmd(uint8_t device_id, const MRCCommand& cmd);
    static canfd_frame encode_cmd_fd(uint8_t device_id, const MRCCommand& cmd);

    // Decode feedback from CAN frames
    static MRCFeedback decode_fbk(const can_frame& frame);
    static MRCFeedback decode_fbk(const canfd_frame& frame);

    // Convenience command factories
    static MRCCommand free_cmd();
    static MRCCommand fix_limit_cmd(double current_amp);
    static MRCCommand adaptation_cmd(double current_amp);
    static MRCCommand reset_cmd();
    static MRCCommand zero_cmd();
    static MRCCommand refresh_cmd();
};

}  // namespace safemrc::mrc
```

- [ ] **Step 2: Write mrc_protocol.cpp**

```cpp
#include <safemrc/mrc/mrc_protocol.hpp>

#include <cstring>

namespace safemrc::mrc {

static void fill_cmd_data(uint8_t* data, const MRCCommand& cmd) {
    std::memset(data, 0, 8);
    data[0] = static_cast<uint8_t>(cmd.mode);
    data[1] = 0x00;  // reserved
    // des_coil_current: int32_t, little-endian, bytes 2-5
    data[2] = static_cast<uint8_t>(cmd.des_coil_current & 0xFF);
    data[3] = static_cast<uint8_t>((cmd.des_coil_current >> 8) & 0xFF);
    data[4] = static_cast<uint8_t>((cmd.des_coil_current >> 16) & 0xFF);
    data[5] = static_cast<uint8_t>((cmd.des_coil_current >> 24) & 0xFF);
    data[6] = 0x00;  // reserved
    data[7] = 0x00;  // reserved
}

static MRCFeedback parse_fbk_data(const uint8_t* data) {
    MRCFeedback fbk;
    fbk.mode = static_cast<MRCMode>(data[0]);
    fbk.collision = (data[1] != 0);
    // encoder_value: int32_t, little-endian, bytes 2-5
    fbk.encoder_value = static_cast<int32_t>(
        static_cast<uint32_t>(data[2]) |
        (static_cast<uint32_t>(data[3]) << 8) |
        (static_cast<uint32_t>(data[4]) << 16) |
        (static_cast<uint32_t>(data[5]) << 24));
    // present_current: int16_t, little-endian, bytes 6-7
    fbk.present_current = static_cast<int16_t>(
        static_cast<uint16_t>(data[6]) |
        (static_cast<uint16_t>(data[7]) << 8));
    return fbk;
}

can_frame MRCProtocol::encode_cmd(uint8_t device_id, const MRCCommand& cmd) {
    can_frame frame{};
    frame.can_id = cmd_id(device_id);
    frame.can_dlc = 8;
    fill_cmd_data(frame.data, cmd);
    return frame;
}

canfd_frame MRCProtocol::encode_cmd_fd(uint8_t device_id, const MRCCommand& cmd) {
    canfd_frame frame{};
    frame.can_id = cmd_id(device_id);
    frame.len = 8;
    fill_cmd_data(frame.data, cmd);
    return frame;
}

MRCFeedback MRCProtocol::decode_fbk(const can_frame& frame) {
    return parse_fbk_data(frame.data);
}

MRCFeedback MRCProtocol::decode_fbk(const canfd_frame& frame) {
    return parse_fbk_data(frame.data);
}

MRCCommand MRCProtocol::free_cmd() {
    return {MRCMode::FREE, 0};
}

MRCCommand MRCProtocol::fix_limit_cmd(double current_amp) {
    return {MRCMode::FIX_LIMIT, static_cast<int32_t>(current_amp * 1000.0)};
}

MRCCommand MRCProtocol::adaptation_cmd(double current_amp) {
    return {MRCMode::ADAPTATION, static_cast<int32_t>(current_amp * 1000.0)};
}

MRCCommand MRCProtocol::reset_cmd() {
    return {MRCMode::MRC_RESET, 0};
}

MRCCommand MRCProtocol::zero_cmd() {
    return {MRCMode::ZERO, 0};
}

MRCCommand MRCProtocol::refresh_cmd() {
    return {MRCMode::REFRESH, 0};
}

}  // namespace safemrc::mrc
```

- [ ] **Step 3: Write test_mrc_protocol.cpp**

```cpp
#include <gtest/gtest.h>
#include <safemrc/mrc/mrc_protocol.hpp>

#include <cmath>

using namespace safemrc::mrc;

TEST(MRCProtocol, CmdIdEncoding) {
    EXPECT_EQ(MRCProtocol::cmd_id(1), 0x101u);
    EXPECT_EQ(MRCProtocol::cmd_id(254), 0x1FEu);
}

TEST(MRCProtocol, FbkIdEncoding) {
    EXPECT_EQ(MRCProtocol::fbk_id(1), 0x201u);
    EXPECT_EQ(MRCProtocol::fbk_id(254), 0x2FEu);
}

TEST(MRCProtocol, EncodeDecodeRoundTrip) {
    auto cmd = MRCProtocol::fix_limit_cmd(3.0);
    auto frame = MRCProtocol::encode_cmd(1, cmd);

    EXPECT_EQ(frame.can_id, 0x101u);
    EXPECT_EQ(frame.can_dlc, 8);
    EXPECT_EQ(frame.data[0], static_cast<uint8_t>(MRCMode::FIX_LIMIT));
    EXPECT_EQ(frame.data[1], 0x00);

    // 3000 = 0x00000BB8 in LE: B8 0B 00 00
    EXPECT_EQ(frame.data[2], 0xB8);
    EXPECT_EQ(frame.data[3], 0x0B);
    EXPECT_EQ(frame.data[4], 0x00);
    EXPECT_EQ(frame.data[5], 0x00);
}

TEST(MRCProtocol, EncodeCmdFd) {
    auto cmd = MRCProtocol::adaptation_cmd(2.5);
    auto frame = MRCProtocol::encode_cmd_fd(3, cmd);

    EXPECT_EQ(frame.can_id, 0x103u);
    EXPECT_EQ(frame.len, 8);
    EXPECT_EQ(frame.data[0], static_cast<uint8_t>(MRCMode::ADAPTATION));
    // 2500 = 0x000009C4 in LE: C4 09 00 00
    EXPECT_EQ(frame.data[2], 0xC4);
    EXPECT_EQ(frame.data[3], 0x09);
}

TEST(MRCProtocol, DecodeFeedback) {
    can_frame frame{};
    frame.can_id = 0x201;
    frame.can_dlc = 8;
    frame.data[0] = 2;     // ADAPTATION
    frame.data[1] = 0x01;  // collision
    // encoder_value = 65535 (0x0000FFFF LE)
    frame.data[2] = 0xFF;
    frame.data[3] = 0xFF;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    // present_current = 3500 (0x0DAC LE)
    frame.data[6] = 0xAC;
    frame.data[7] = 0x0D;

    auto fbk = MRCProtocol::decode_fbk(frame);

    EXPECT_EQ(fbk.mode, MRCMode::ADAPTATION);
    EXPECT_TRUE(fbk.collision);
    EXPECT_EQ(fbk.encoder_value, 65535);
    EXPECT_NEAR(fbk.position_rad(), 1.0, 1e-4);
    EXPECT_EQ(fbk.present_current, 3500);
    EXPECT_NEAR(fbk.current_amp(), 3.5, 1e-6);
}

TEST(MRCProtocol, DecodeFeedbackNegativeValues) {
    can_frame frame{};
    frame.can_id = 0x201;
    frame.can_dlc = 8;
    frame.data[0] = 0;     // FREE
    frame.data[1] = 0x00;  // no collision
    // encoder_value = -32768 (0xFFFF8000 LE)
    frame.data[2] = 0x00;
    frame.data[3] = 0x80;
    frame.data[4] = 0xFF;
    frame.data[5] = 0xFF;
    // present_current = -1000 (0xFC18 LE)
    frame.data[6] = 0x18;
    frame.data[7] = 0xFC;

    auto fbk = MRCProtocol::decode_fbk(frame);

    EXPECT_EQ(fbk.encoder_value, -32768);
    EXPECT_NEAR(fbk.position_rad(), -32768.0 / 65535.0, 1e-6);
    EXPECT_EQ(fbk.present_current, -1000);
    EXPECT_NEAR(fbk.current_amp(), -1.0, 1e-6);
}

TEST(MRCProtocol, DecodeFeedbackFdFrame) {
    canfd_frame frame{};
    frame.can_id = 0x202;
    frame.len = 8;
    frame.data[0] = 1;     // FIX_LIMIT
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0xE8;  // 1000 LE
    frame.data[7] = 0x03;

    auto fbk = MRCProtocol::decode_fbk(frame);

    EXPECT_EQ(fbk.mode, MRCMode::FIX_LIMIT);
    EXPECT_FALSE(fbk.collision);
    EXPECT_EQ(fbk.encoder_value, 0);
    EXPECT_EQ(fbk.present_current, 1000);
    EXPECT_NEAR(fbk.current_amp(), 1.0, 1e-6);
}

TEST(MRCProtocol, ConvenienceFactories) {
    auto free = MRCProtocol::free_cmd();
    EXPECT_EQ(free.mode, MRCMode::FREE);
    EXPECT_EQ(free.des_coil_current, 0);

    auto fix = MRCProtocol::fix_limit_cmd(5.0);
    EXPECT_EQ(fix.mode, MRCMode::FIX_LIMIT);
    EXPECT_EQ(fix.des_coil_current, 5000);

    auto adapt = MRCProtocol::adaptation_cmd(-2.0);
    EXPECT_EQ(adapt.mode, MRCMode::ADAPTATION);
    EXPECT_EQ(adapt.des_coil_current, -2000);

    auto reset = MRCProtocol::reset_cmd();
    EXPECT_EQ(reset.mode, MRCMode::MRC_RESET);

    auto zero = MRCProtocol::zero_cmd();
    EXPECT_EQ(zero.mode, MRCMode::ZERO);

    auto refresh = MRCProtocol::refresh_cmd();
    EXPECT_EQ(refresh.mode, MRCMode::REFRESH);
}

TEST(MRCProtocol, EncodeNegativeCurrent) {
    auto cmd = MRCProtocol::fix_limit_cmd(-3.0);
    auto frame = MRCProtocol::encode_cmd(1, cmd);

    // -3000 = 0xFFFFF448 LE: 48 F4 FF FF
    EXPECT_EQ(frame.data[2], 0x48);
    EXPECT_EQ(frame.data[3], 0xF4);
    EXPECT_EQ(frame.data[4], 0xFF);
    EXPECT_EQ(frame.data[5], 0xFF);
}
```

- [ ] **Step 4: Run tests**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
ctest --output-on-failure -R test_mrc_protocol
```

Expected: All 8 tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/safemrc/mrc/mrc_protocol.hpp src/safemrc/mrc/mrc_protocol.cpp tests/test_mrc_protocol.cpp
git commit -m "feat: implement MRCProtocol encoder/decoder with CAN and CAN FD support"
```

---

### Task 5: MRCState — Thread-Safe State Container

**Files:**
- Modify: `include/safemrc/mrc/mrc_state.hpp`
- Modify: `src/safemrc/mrc/mrc_state.cpp`
- Modify: `tests/test_mrc_state.cpp`

- [ ] **Step 1: Write mrc_state.hpp**

```cpp
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
```

- [ ] **Step 2: Write mrc_state.cpp**

```cpp
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
```

- [ ] **Step 3: Write test_mrc_state.cpp**

```cpp
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
    fbk.mode = MRCMode::ADAPTATION;
    fbk.collision = true;
    fbk.encoder_value = 65535;
    fbk.present_current = 3500;

    state.update(fbk);

    EXPECT_EQ(state.mode(), MRCMode::ADAPTATION);
    EXPECT_TRUE(state.collision());
    EXPECT_NEAR(state.position_rad(), 1.0, 1e-4);
    EXPECT_NEAR(state.current_amp(), 3.5, 1e-6);
    EXPECT_FALSE(state.is_stale());
}

TEST(MRCState, FeedbackCopy) {
    MRCState state(3);
    MRCFeedback fbk;
    fbk.mode = MRCMode::FIX_LIMIT;
    fbk.collision = false;
    fbk.encoder_value = 1000;
    fbk.present_current = -500;

    state.update(fbk);
    auto copy = state.feedback();

    EXPECT_EQ(copy.mode, MRCMode::FIX_LIMIT);
    EXPECT_FALSE(copy.collision);
    EXPECT_EQ(copy.encoder_value, 1000);
    EXPECT_EQ(copy.present_current, -500);
}

TEST(MRCState, StalenessAfterTimeout) {
    MRCState state(4);
    MRCFeedback fbk{};
    state.update(fbk);

    EXPECT_FALSE(state.is_stale(std::chrono::milliseconds(100)));
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_TRUE(state.is_stale(std::chrono::milliseconds(10)));
}

TEST(MRCState, ConcurrentReadWrite) {
    MRCState state(5);
    constexpr int iterations = 10000;

    std::thread writer([&] {
        for (int i = 0; i < iterations; ++i) {
            MRCFeedback fbk;
            fbk.mode = MRCMode::FIX_LIMIT;
            fbk.encoder_value = i;
            fbk.present_current = static_cast<int16_t>(i % 1000);
            state.update(fbk);
        }
    });

    std::thread reader([&] {
        for (int i = 0; i < iterations; ++i) {
            auto fbk = state.feedback();
            // Should not crash; values should be internally consistent
            (void)fbk.position_rad();
            (void)fbk.current_amp();
        }
    });

    writer.join();
    reader.join();

    // If we get here without crashing/deadlocking, the test passes
    auto final_fbk = state.feedback();
    EXPECT_EQ(final_fbk.mode, MRCMode::FIX_LIMIT);
}
```

- [ ] **Step 4: Run tests**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
ctest --output-on-failure -R test_mrc_state
```

Expected: All 5 tests pass.

- [ ] **Step 5: Commit**

```bash
git add include/safemrc/mrc/mrc_state.hpp src/safemrc/mrc/mrc_state.cpp tests/test_mrc_state.cpp
git commit -m "feat: implement MRCState thread-safe device state container"
```

---

### Task 6: MRCDevice — Concrete CAN Device

**Files:**
- Modify: `include/safemrc/mrc/mrc_device.hpp`
- Modify: `src/safemrc/mrc/mrc_device.cpp`

- [ ] **Step 1: Write mrc_device.hpp**

```cpp
#pragma once

#include <safemrc/can/can_device.hpp>
#include <safemrc/mrc/mrc_protocol.hpp>
#include <safemrc/mrc/mrc_state.hpp>

namespace safemrc::mrc {

class MRCDevice : public can::CANDevice {
public:
    explicit MRCDevice(uint8_t device_id, bool use_fd = false);

    void on_frame(const can_frame& frame) override;
    void on_frame(const canfd_frame& frame) override;

    const MRCState& state() const { return state_; }
    uint8_t device_id() const { return device_id_; }

private:
    uint8_t device_id_;
    MRCState state_;
};

}  // namespace safemrc::mrc
```

- [ ] **Step 2: Write mrc_device.cpp**

```cpp
#include <safemrc/mrc/mrc_device.hpp>

namespace safemrc::mrc {

MRCDevice::MRCDevice(uint8_t device_id, bool use_fd)
    : CANDevice(MRCProtocol::cmd_id(device_id),
                MRCProtocol::fbk_id(device_id),
                CAN_SFF_MASK, use_fd),
      device_id_(device_id),
      state_(device_id) {}

void MRCDevice::on_frame(const can_frame& frame) {
    state_.update(MRCProtocol::decode_fbk(frame));
}

void MRCDevice::on_frame(const canfd_frame& frame) {
    state_.update(MRCProtocol::decode_fbk(frame));
}

}  // namespace safemrc::mrc
```

- [ ] **Step 3: Verify build**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
ctest --output-on-failure
```

Expected: All existing tests still pass.

- [ ] **Step 4: Commit**

```bash
git add include/safemrc/mrc/mrc_device.hpp src/safemrc/mrc/mrc_device.cpp
git commit -m "feat: implement MRCDevice bridging CAN frames to MRCState"
```

---

### Task 7: SafeMRC — Facade with Background Thread

**Files:**
- Modify: `include/safemrc/safemrc.hpp`
- Modify: `src/safemrc/safemrc.cpp`

- [ ] **Step 1: Write safemrc.hpp**

```cpp
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
#include <thread>
#include <vector>

namespace safemrc {

class SafeMRC {
public:
    explicit SafeMRC(const std::string& interface, bool use_fd = false);
    ~SafeMRC();

    SafeMRC(const SafeMRC&) = delete;
    SafeMRC& operator=(const SafeMRC&) = delete;

    // Device management
    void add_device(uint8_t device_id);
    void remove_device(uint8_t device_id);

    // Commands
    void send_command(uint8_t device_id, const mrc::MRCCommand& cmd);
    void send_command_all(const mrc::MRCCommand& cmd);

    // Convenience
    void set_free(uint8_t device_id);
    void set_free_all();
    void set_fix_limit(uint8_t device_id, double current_amp);
    void set_adaptation(uint8_t device_id, double current_amp);
    void reset_collision(uint8_t device_id);
    void set_zero(uint8_t device_id);

    // State queries (thread-safe)
    const mrc::MRCState& device_state(uint8_t device_id) const;
    std::vector<uint8_t> device_ids() const;
    size_t device_count() const;

    // Background thread control
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
```

- [ ] **Step 2: Write safemrc.cpp**

```cpp
#include <safemrc/safemrc.hpp>

#include <stdexcept>

namespace safemrc {

SafeMRC::SafeMRC(const std::string& interface, bool use_fd)
    : socket_(interface, use_fd) {}

SafeMRC::~SafeMRC() {
    stop();
}

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
    for (auto& [id, _] : mrc_devices_) {
        ids.push_back(id);
    }
    return ids;
}

size_t SafeMRC::device_count() const {
    return mrc_devices_.size();
}

void SafeMRC::start() {
    if (running_.load()) return;
    running_.store(true);
    recv_thread_ = std::thread(&SafeMRC::recv_loop, this);
}

void SafeMRC::stop() {
    running_.store(false);
    if (recv_thread_.joinable()) {
        recv_thread_.join();
    }
}

void SafeMRC::recv_loop() {
    while (running_.load()) {
        if (!socket_.is_data_available(1000)) continue;  // 1ms timeout

        if (socket_.is_fd_enabled()) {
            canfd_frame frame{};
            if (socket_.read_canfd_frame(frame)) {
                devices_.dispatch(frame);
            }
        } else {
            can_frame frame{};
            if (socket_.read_can_frame(frame)) {
                devices_.dispatch(frame);
            }
        }
    }
}

}  // namespace safemrc
```

- [ ] **Step 3: Verify build**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
ctest --output-on-failure
```

Expected: All existing tests still pass.

- [ ] **Step 4: Commit**

```bash
git add include/safemrc/safemrc.hpp src/safemrc/safemrc.cpp
git commit -m "feat: implement SafeMRC facade with background receive thread"
```

---

### Task 8: Integration Test with vcan

**Files:**
- Modify: `tests/test_integration.cpp`

- [ ] **Step 1: Write test_integration.cpp**

This test requires a `vcan0` interface. It will be skipped if `vcan0` is not available.

```cpp
#include <gtest/gtest.h>
#include <safemrc/safemrc.hpp>

#include <net/if.h>
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
    // Open two sockets: one for SafeMRC, one to simulate the device
    safemrc::SafeMRC mrc("vcan0");
    mrc.add_device(1);
    mrc.start();

    // Open a raw socket to simulate device response
    safemrc::can::CANSocket sim_socket("vcan0");

    // Send a command
    mrc.set_fix_limit(1, 2.0);

    // Read the command on sim_socket
    if (sim_socket.is_data_available(10000)) {  // 10ms
        can_frame cmd{};
        ASSERT_TRUE(sim_socket.read_can_frame(cmd));
        EXPECT_EQ(cmd.can_id, 0x101u);
        EXPECT_EQ(cmd.data[0], static_cast<uint8_t>(safemrc::mrc::MRCMode::FIX_LIMIT));
    }

    // Simulate device feedback
    can_frame fbk{};
    fbk.can_id = 0x201;
    fbk.can_dlc = 8;
    fbk.data[0] = 1;     // FIX_LIMIT
    fbk.data[1] = 0x00;  // no collision
    // encoder_value = 32768
    fbk.data[2] = 0x00;
    fbk.data[3] = 0x80;
    fbk.data[4] = 0x00;
    fbk.data[5] = 0x00;
    // present_current = 2000
    fbk.data[6] = 0xD0;
    fbk.data[7] = 0x07;
    sim_socket.write_can_frame(fbk);

    // Wait for background thread to process
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
    mrc.add_device(1);
    mrc.add_device(2);
    mrc.start();

    safemrc::can::CANSocket sim("vcan0");

    // Send feedback for device 1
    can_frame fbk1{};
    fbk1.can_id = 0x201;
    fbk1.can_dlc = 8;
    fbk1.data[0] = 0;  // FREE
    sim.write_can_frame(fbk1);

    // Send feedback for device 2
    can_frame fbk2{};
    fbk2.can_id = 0x202;
    fbk2.can_dlc = 8;
    fbk2.data[0] = 2;     // ADAPTATION
    fbk2.data[1] = 0x01;  // collision
    sim.write_can_frame(fbk2);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    EXPECT_EQ(mrc.device_state(1).mode(), safemrc::mrc::MRCMode::FREE);
    EXPECT_EQ(mrc.device_state(2).mode(), safemrc::mrc::MRCMode::ADAPTATION);
    EXPECT_TRUE(mrc.device_state(2).collision());

    mrc.stop();
}
```

- [ ] **Step 2: Run tests**

First set up vcan if not already:
```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan 2>/dev/null || true
sudo ip link set up vcan0
```

Then:
```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
ctest --output-on-failure -R test_integration
```

Expected: 2 tests pass (or skip if vcan not available).

- [ ] **Step 3: Commit**

```bash
git add tests/test_integration.cpp
git commit -m "feat: add integration tests using vcan0 loopback"
```

---

### Task 9: C++ Demo Example

**Files:**
- Modify: `examples/demo.cpp`

- [ ] **Step 1: Write demo.cpp**

```cpp
#include <safemrc/safemrc.hpp>

#include <chrono>
#include <cstdio>
#include <thread>

int main(int argc, char* argv[]) {
    const char* interface = (argc > 1) ? argv[1] : "can0";
    bool use_fd = false;

    try {
        safemrc::SafeMRC mrc(interface, use_fd);

        // Add devices
        mrc.add_device(1);
        printf("Added device 1 on %s (FD=%s)\n", interface, use_fd ? "true" : "false");

        // Start background receive thread
        mrc.start();

        // Zero the encoder
        mrc.set_zero(1);
        printf("Sent ZERO command\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        // Fixed torque control at 2A
        printf("Setting FIX_LIMIT mode at 2.0 A\n");
        for (int i = 0; i < 100; ++i) {
            mrc.set_fix_limit(1, 2.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

            auto& s = mrc.device_state(1);
            if (!s.is_stale()) {
                printf("[%3d] mode=%d collision=%d pos=%.4f rad cur=%.3f A\n",
                       i, static_cast<int>(s.mode()), s.collision(),
                       s.position_rad(), s.current_amp());
            }
        }

        // Check collision
        if (mrc.device_state(1).collision()) {
            printf("Collision detected! Resetting...\n");
            mrc.reset_collision(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Release
        mrc.set_free(1);
        printf("Released (FREE mode)\n");

        mrc.stop();

    } catch (const safemrc::can::CANSocketException& e) {
        fprintf(stderr, "CAN error: %s\n", e.what());
        return 1;
    }

    return 0;
}
```

- [ ] **Step 2: Verify build**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. && make -j$(nproc)
```

Expected: `safemrc-demo` binary builds.

- [ ] **Step 3: Commit**

```bash
git add examples/demo.cpp
git commit -m "feat: add C++ demo example for SafeMRC control loop"
```

---

### Task 10: Python Bindings

**Files:**
- Create: `python/CMakeLists.txt`
- Create: `python/pyproject.toml`
- Create: `python/src/safemrc_can.cpp`
- Create: `python/examples/demo.py`

- [ ] **Step 1: Write python/pyproject.toml**

```toml
[build-system]
build-backend = "scikit_build_core.build"
requires = ["scikit-build-core", "nanobind"]

[project]
name = "safemrc_can"
version = "0.1.0"
requires-python = ">= 3.10"
license = {text = "Apache-2.0"}
```

- [ ] **Step 2: Write python/CMakeLists.txt**

```cmake
cmake_minimum_required(VERSION 3.22)
project(
  safemrc_can_python
  VERSION 0.1.0
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(
  Python
  COMPONENTS Interpreter Development.Module
  REQUIRED)

# Find nanobind
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

# Find the C++ library
find_package(SafeMRCCAN REQUIRED)

nanobind_add_module(safemrc_can_python src/safemrc_can.cpp)
set_target_properties(safemrc_can_python PROPERTIES OUTPUT_NAME "safemrc_can")
target_link_libraries(safemrc_can_python PRIVATE SafeMRCCAN::safemrc_can)
install(TARGETS safemrc_can_python LIBRARY DESTINATION .)
```

- [ ] **Step 3: Write python/src/safemrc_can.cpp**

```cpp
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <safemrc/safemrc.hpp>

namespace nb = nanobind;

using namespace safemrc;
using namespace safemrc::mrc;

NB_MODULE(safemrc_can, m) {
    m.doc() = "SafeMRC CAN driver Python bindings";

    // MRCMode enum
    nb::enum_<MRCMode>(m, "MRCMode")
        .value("FREE", MRCMode::FREE)
        .value("FIX_LIMIT", MRCMode::FIX_LIMIT)
        .value("ADAPTATION", MRCMode::ADAPTATION)
        .value("DEBUG", MRCMode::DEBUG)
        .value("MRC_RESET", MRCMode::MRC_RESET)
        .value("ZERO", MRCMode::ZERO)
        .value("REFRESH", MRCMode::REFRESH)
        .export_values();

    // MRCCommand struct
    nb::class_<MRCCommand>(m, "MRCCommand")
        .def(nb::init<>())
        .def(
            "__init__",
            [](MRCCommand* cmd, MRCMode mode, int32_t current) {
                new (cmd) MRCCommand{mode, current};
            },
            nb::arg("mode"), nb::arg("des_coil_current") = 0)
        .def_rw("mode", &MRCCommand::mode)
        .def_rw("des_coil_current", &MRCCommand::des_coil_current);

    // MRCFeedback struct
    nb::class_<MRCFeedback>(m, "MRCFeedback")
        .def(nb::init<>())
        .def_rw("mode", &MRCFeedback::mode)
        .def_rw("collision", &MRCFeedback::collision)
        .def_rw("encoder_value", &MRCFeedback::encoder_value)
        .def_rw("present_current", &MRCFeedback::present_current)
        .def_prop_ro("position_rad", &MRCFeedback::position_rad)
        .def_prop_ro("current_amp", &MRCFeedback::current_amp);

    // MRCState (read-only from Python)
    nb::class_<MRCState>(m, "MRCState")
        .def_prop_ro("device_id", &MRCState::device_id)
        .def_prop_ro("mode", &MRCState::mode)
        .def_prop_ro("collision", &MRCState::collision)
        .def_prop_ro("position_rad", &MRCState::position_rad)
        .def_prop_ro("current_amp", &MRCState::current_amp)
        .def("is_stale", &MRCState::is_stale,
             nb::arg("timeout_ms") = std::chrono::milliseconds(5))
        .def("feedback", &MRCState::feedback);

    // SafeMRC class
    nb::class_<SafeMRC>(m, "SafeMRC")
        .def(nb::init<const std::string&, bool>(),
             nb::arg("interface"), nb::arg("use_fd") = false)
        .def("add_device", &SafeMRC::add_device, nb::arg("device_id"))
        .def("remove_device", &SafeMRC::remove_device, nb::arg("device_id"))
        .def("send_command", &SafeMRC::send_command,
             nb::arg("device_id"), nb::arg("cmd"))
        .def("send_command_all", &SafeMRC::send_command_all, nb::arg("cmd"))
        .def("set_free", &SafeMRC::set_free, nb::arg("device_id"))
        .def("set_free_all", &SafeMRC::set_free_all)
        .def("set_fix_limit", &SafeMRC::set_fix_limit,
             nb::arg("device_id"), nb::arg("current_amp"))
        .def("set_adaptation", &SafeMRC::set_adaptation,
             nb::arg("device_id"), nb::arg("current_amp"))
        .def("reset_collision", &SafeMRC::reset_collision, nb::arg("device_id"))
        .def("set_zero", &SafeMRC::set_zero, nb::arg("device_id"))
        .def("device_state", &SafeMRC::device_state,
             nb::arg("device_id"), nb::rv_policy::reference)
        .def("device_ids", &SafeMRC::device_ids)
        .def("device_count", &SafeMRC::device_count)
        .def("start", &SafeMRC::start)
        .def("stop", &SafeMRC::stop)
        .def("is_running", &SafeMRC::is_running);
}
```

- [ ] **Step 4: Write python/examples/demo.py**

```python
#!/usr/bin/env python3
"""SafeMRC CAN driver demo."""

import time
import safemrc_can as mrc


def main():
    sm = mrc.SafeMRC("can0", use_fd=False)
    sm.add_device(1)
    sm.start()
    print(f"Started SafeMRC with {sm.device_count()} device(s)")

    # Zero encoder
    sm.set_zero(1)
    time.sleep(0.01)

    # Fixed torque at 2A
    print("FIX_LIMIT mode at 2.0 A")
    for i in range(100):
        sm.set_fix_limit(1, 2.0)
        time.sleep(0.001)

        state = sm.device_state(1)
        if not state.is_stale():
            print(
                f"[{i:3d}] mode={state.mode} collision={state.collision} "
                f"pos={state.position_rad:.4f} rad cur={state.current_amp:.3f} A"
            )

    # Check collision
    if sm.device_state(1).collision:
        print("Collision! Resetting...")
        sm.reset_collision(1)
        time.sleep(0.01)

    sm.set_free_all()
    sm.stop()
    print("Done")


if __name__ == "__main__":
    main()
```

- [ ] **Step 5: Verify C++ library installs and Python bindings build**

```bash
# Install C++ library
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. -DCMAKE_INSTALL_PREFIX=/tmp/safemrc_install
make -j$(nproc) && make install

# Build Python bindings
cd /home/wenbo-li/lab_dev/safe-mrc-can/python
pip install nanobind scikit-build-core
CMAKE_PREFIX_PATH=/tmp/safemrc_install pip install -v --no-build-isolation .
```

Expected: Python package installs. `python3 -c "import safemrc_can; print('OK')"` prints OK.

- [ ] **Step 6: Commit**

```bash
git add python/
git commit -m "feat: add Python bindings via nanobind for SafeMRC facade"
```

---

### Task 11: Final Verification

- [ ] **Step 1: Run full test suite**

```bash
cd /home/wenbo-li/lab_dev/safe-mrc-can/build
cmake .. -DBUILD_TESTING=ON
make -j$(nproc)
ctest --output-on-failure
```

Expected: All unit tests pass. Integration tests pass if vcan0 is available, skip otherwise.

- [ ] **Step 2: Verify all headers are self-contained**

Each header should compile independently. Quick check:
```bash
for h in include/safemrc/can/*.hpp include/safemrc/mrc/*.hpp include/safemrc/safemrc.hpp; do
  echo "Checking $h..."
  g++ -std=c++17 -fsyntax-only -I include "$h" || echo "FAIL: $h"
done
```

Expected: All headers pass syntax check.

- [ ] **Step 3: Final commit if any cleanup needed**

```bash
git status
# If clean, nothing to do
# If changes needed, commit them
```
