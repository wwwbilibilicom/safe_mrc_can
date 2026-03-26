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

install(TARGETS safemrc_can EXPORT safemrc_can_export)
install(DIRECTORY include/safemrc TYPE INCLUDE)

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

add_executable(safemrc-demo examples/demo.cpp)
target_link_libraries(safemrc-demo safemrc_can)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()
