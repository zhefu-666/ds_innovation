cmake_minimum_required(VERSION 3.10)

project(rescue_upper_host LANGUAGES CXX)

include(CTest)

# Explicit developer targets; never rewrite source files during a normal build.
find_package(Python3 COMPONENTS Interpreter QUIET)
if(Python3_Interpreter_FOUND)
    set(RESCUE_FORMAT_PYTHON "${Python3_EXECUTABLE}")
else()
    set(RESCUE_FORMAT_PYTHON python3)
endif()
add_custom_target(format-check
    COMMAND "${RESCUE_FORMAT_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/format_code.py" --check
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Checking project C/C++ formatting"
    VERBATIM)
add_custom_target(format
    COMMAND "${RESCUE_FORMAT_PYTHON}" "${CMAKE_CURRENT_SOURCE_DIR}/tools/format_code.py" --fix
    WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
    COMMENT "Formatting project C/C++ sources"
    VERBATIM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Use ccache when available; respect an explicitly supplied compiler launcher.
option(RESCUE_ENABLE_CCACHE "Use ccache for C++ compilation when available" ON)
if(RESCUE_ENABLE_CCACHE AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
    find_program(RESCUE_CCACHE_PROGRAM NAMES ccache HINTS "$ENV{HOME}/.local/bin")
    if(RESCUE_CCACHE_PROGRAM)
        set(CMAKE_CXX_COMPILER_LAUNCHER "${RESCUE_CCACHE_PROGRAM}")
        message(STATUS "C++ compiler cache: ${RESCUE_CCACHE_PROGRAM}")
    else()
        message(STATUS "ccache not found; compiling without a compiler cache")
    endif()
endif()

find_package(OpenCV REQUIRED)
find_package(nlohmann_json 3.6.0 REQUIRED)

set(RESCUE_TASK_SOURCES src/task/push_task.cpp src/task/capture_verification_stage.cpp
    src/task/delivery_verification_stage.cpp src/task/release_retreat_stage.cpp src/task/push_in_stage.cpp
    src/task/sweep_ledger.cpp src/task/disperse_turn_stage.cpp src/task/disperse_swing_stage.cpp
    src/task/disperse_rush_stage.cpp src/task/recovery_stage.cpp src/task/search_stage.cpp
    src/task/enter_stage.cpp src/task/disperse_cluster_stage.cpp src/task/carry_approach_stage.cpp
    src/task/carry_search_stage.cpp src/task/sweep_trigger_stage.cpp src/task/sweep_stage.cpp)

add_library(rescue_app_config STATIC
    src/app/config_loader.cpp
    src/app/runtime_config.cpp
    src/config/config.cpp
    src/config/task_calibration.cpp
)
target_include_directories(rescue_app_config PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_app_config PUBLIC ${OpenCV_LIBS} PRIVATE nlohmann_json::nlohmann_json)

option(RESCUE_ENABLE_RKNN "Build the RKNN inference backend (requires the vendor SDK header)" ON)
if(RESCUE_ENABLE_RKNN)
    find_path(RESCUE_RKNN_INCLUDE_DIR rknn_api.h
        HINTS ${CMAKE_CURRENT_SOURCE_DIR}/benchmark_results)
    if(NOT RESCUE_RKNN_INCLUDE_DIR)
        message(FATAL_ERROR "RKNN SDK header not found. Set RESCUE_RKNN_INCLUDE_DIR, or use -DRESCUE_ENABLE_RKNN=OFF for offline tests.")
    endif()
    add_library(rescue_rknn STATIC src/perception/rknn_model.cpp src/perception/rknn_output_layout.cpp)
    target_include_directories(rescue_rknn PRIVATE ${RESCUE_RKNN_INCLUDE_DIR})
    target_compile_definitions(rescue_rknn PUBLIC RESCUE_HAS_RKNN=1)
else()
    add_library(rescue_rknn STATIC src/perception/rknn_unavailable.cpp src/perception/rknn_output_layout.cpp)
    target_compile_definitions(rescue_rknn PUBLIC RESCUE_HAS_RKNN=0)
    message(STATUS "RKNN inference disabled; offline replay and CPU ONNX inference remain available")
endif()
target_include_directories(rescue_rknn PUBLIC include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_rknn PUBLIC ${OpenCV_LIBS} ${CMAKE_DL_LIBS})

add_executable(rescue_upper_host
    src/main.cpp
    src/app/runtime.cpp
    src/app/perception_pipeline.cpp
    src/app/observation_builder.cpp
    src/app/offline_tasks.cpp
    src/app/diagnostics.cpp
    src/control/controlled_field.cpp
    src/control/match_control.cpp
    src/control/match_server.cpp
    src/geometry/navigation_adapter.cpp
    src/perception/zone_color.cpp
    src/geometry/zone_estimate.cpp
    src/geometry/zone_geometry.cpp
    src/perception/keypoint_filter.cpp
    src/geometry/ground_pose_fitter.cpp
    src/geometry/geometry_pipeline.cpp
    src/perception/silhouette.cpp
    src/geometry/geometry_replay.cpp
    src/hardware/hipnuc_imu.cpp
    src/hardware/imu_adapter.cpp
    ${RESCUE_TASK_SOURCES}
    src/task/transport_rules.cpp
    src/perception/capture_monitor.cpp
    src/core/types.cpp
    src/core/utils.cpp
    src/hardware/uart_controller.cpp
    src/hardware/motion_link.cpp
    src/perception/yolo_detector.cpp
    src/perception/model_io.cpp
    src/perception/perception_adapter.cpp
    src/perception/vision_logic.cpp
    src/hardware/camera_capture.cpp
    src/perception/detector.cpp
    src/perception/zone_keypoints.cpp
    src/perception/tracker.cpp
    src/geometry/camera_calibration.cpp
    src/geometry/safe_zone_pose.cpp
    src/geometry/zone_layout.cpp
    src/geometry/planner.cpp
    src/hardware/sensor_fusion.cpp
    src/core/logger.cpp
)

option(RESCUE_ENABLE_TELEMETRY "Build asynchronous telemetry snapshot publisher" OFF)
if(RESCUE_ENABLE_TELEMETRY)
    target_sources(rescue_upper_host PRIVATE src/telemetry/telemetry_publisher.cpp)
    target_compile_definitions(rescue_upper_host PRIVATE RESCUE_ENABLE_TELEMETRY=1)
endif()

target_include_directories(rescue_upper_host PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rescue_upper_host PRIVATE
    rescue_app_config
    nlohmann_json::nlohmann_json
    rescue_rknn
    ${OpenCV_LIBS}
    pthread
    ${CMAKE_DL_LIBS}
)

add_executable(rescue_sensor_monitor src/tools/sensor_monitor.cpp src/hardware/uart_controller.cpp src/core/utils.cpp)
target_include_directories(rescue_sensor_monitor PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_sensor_monitor PRIVATE ${OpenCV_LIBS} pthread)

add_executable(rescue_imu_monitor src/tools/imu_monitor.cpp src/hardware/hipnuc_imu.cpp src/core/utils.cpp)
target_include_directories(rescue_imu_monitor PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_imu_monitor PRIVATE ${OpenCV_LIBS} pthread)

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
