cmake_minimum_required(VERSION 3.16)
project(rescue_foxglove_telemetry LANGUAGES CXX)

option(RESCUE_BUILD_FOXGLOVE_PUBLISHER "Build the Foxglove WebSocket publisher" OFF)
set(FOXGLOVE_SDK_DIST "" CACHE PATH "Path to a Foxglove C++ SDK distribution")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rescue_telemetry_smoke src/telemetry_smoke.cpp)
target_compile_options(rescue_telemetry_smoke PRIVATE -Wall -Wextra -Wpedantic)

if(RESCUE_BUILD_FOXGLOVE_PUBLISHER)
  if(NOT FOXGLOVE_SDK_DIST)
    message(FATAL_ERROR "Set FOXGLOVE_SDK_DIST to a Foxglove SDK dist directory")
  endif()
  find_package(foxglove-sdk CONFIG REQUIRED HINTS "${FOXGLOVE_SDK_DIST}")
  foxglove_sdk_add_cpp_library(rescue_foxglove_wrapper TYPE SHARED REMOTE_ACCESS OFF)
  add_executable(rescue_foxglove_publisher src/foxglove_publisher.cpp)
  target_link_libraries(rescue_foxglove_publisher PRIVATE rescue_foxglove_wrapper)
  target_include_directories(rescue_foxglove_publisher PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/schemas")
  target_compile_options(rescue_foxglove_publisher PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS rescue_telemetry_smoke DESTINATION bin)
