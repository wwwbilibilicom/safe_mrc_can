cmake_minimum_required(VERSION 3.22)
project(safemrc_can_python VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE
  OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

find_package(SafeMRCCAN REQUIRED)

nanobind_add_module(safemrc_can_python src/safemrc_can.cpp)
set_target_properties(safemrc_can_python PROPERTIES OUTPUT_NAME "safemrc_can")
target_link_libraries(safemrc_can_python PRIVATE SafeMRCCAN::safemrc_can)
install(TARGETS safemrc_can_python LIBRARY DESTINATION .)
