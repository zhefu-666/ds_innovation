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

if(BUILD_TESTING)
    add_executable(rescue_config_loader_tests tests/config_loader_tests.cpp)
    target_link_libraries(rescue_config_loader_tests PRIVATE rescue_app_config)
    target_compile_options(rescue_config_loader_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_config_loader_tests COMMAND rescue_config_loader_tests)

    add_executable(rescue_runtime_config_tests tests/runtime_config_tests.cpp)
    target_link_libraries(rescue_runtime_config_tests PRIVATE rescue_app_config)
    target_compile_options(rescue_runtime_config_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_runtime_config_tests COMMAND rescue_runtime_config_tests)

    add_executable(rescue_core_tests tests/rescue_core_tests.cpp
        src/core/types.cpp
        src/geometry/zone_estimate.cpp
        src/geometry/zone_geometry.cpp
        src/perception/tracker.cpp
        src/geometry/camera_calibration.cpp
        src/geometry/safe_zone_pose.cpp
        src/geometry/zone_layout.cpp
        src/geometry/planner.cpp
        src/hardware/sensor_fusion.cpp
        src/hardware/imu_adapter.cpp
        src/hardware/hipnuc_imu.cpp
        src/core/logger.cpp
        src/hardware/uart_controller.cpp
        src/core/utils.cpp
    )
    target_include_directories(rescue_core_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(rescue_core_tests PRIVATE ${OpenCV_LIBS} pthread)
    target_compile_options(rescue_core_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_core_tests COMMAND rescue_core_tests)
endif()

if(BUILD_TESTING)
    add_executable(rescue_disperse_swing_stage_tests tests/disperse_swing_stage_tests.cpp
        src/task/disperse_swing_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_swing_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_swing_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_swing_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_swing_stage_tests COMMAND rescue_disperse_swing_stage_tests)

    add_executable(rescue_disperse_swing_task_integration_tests tests/disperse_swing_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_swing_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_swing_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_swing_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_swing_task_integration_tests COMMAND rescue_disperse_swing_task_integration_tests)
    set_tests_properties(rescue_disperse_swing_stage_tests rescue_disperse_swing_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_disperse_rush_stage_tests tests/disperse_rush_stage_tests.cpp
        src/task/disperse_rush_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_rush_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_rush_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_rush_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_rush_stage_tests COMMAND rescue_disperse_rush_stage_tests)

    add_executable(rescue_disperse_rush_task_integration_tests tests/disperse_rush_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_rush_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_rush_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_rush_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_rush_task_integration_tests COMMAND rescue_disperse_rush_task_integration_tests)
    set_tests_properties(rescue_disperse_rush_stage_tests rescue_disperse_rush_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_disperse_turn_stage_tests tests/disperse_turn_stage_tests.cpp
        src/task/disperse_turn_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_turn_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_turn_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_turn_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_turn_stage_tests COMMAND rescue_disperse_turn_stage_tests)

    add_executable(rescue_disperse_turn_task_integration_tests tests/disperse_turn_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_turn_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_turn_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_turn_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_turn_task_integration_tests COMMAND rescue_disperse_turn_task_integration_tests)
    set_tests_properties(rescue_disperse_turn_stage_tests rescue_disperse_turn_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_sweep_ledger_tests tests/sweep_ledger_tests.cpp
        src/task/sweep_ledger.cpp)
    target_include_directories(rescue_sweep_ledger_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_sweep_ledger_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_sweep_ledger_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_sweep_ledger_tests COMMAND rescue_sweep_ledger_tests)

    add_executable(rescue_sweep_task_integration_tests tests/sweep_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_sweep_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_sweep_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_sweep_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_sweep_task_integration_tests COMMAND rescue_sweep_task_integration_tests)
    set_tests_properties(rescue_sweep_ledger_tests rescue_sweep_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_push_in_stage_tests tests/push_in_stage_tests.cpp
        src/task/push_in_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_push_in_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_push_in_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_push_in_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_push_in_stage_tests COMMAND rescue_push_in_stage_tests)

    add_executable(rescue_push_in_task_integration_tests tests/push_in_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_push_in_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_push_in_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_push_in_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_push_in_task_integration_tests COMMAND rescue_push_in_task_integration_tests)
    set_tests_properties(rescue_push_in_stage_tests rescue_push_in_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_release_stage_tests tests/release_stage_tests.cpp
        src/task/release_retreat_stage.cpp)
    target_include_directories(rescue_release_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_release_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_release_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_release_stage_tests COMMAND rescue_release_stage_tests)

    add_executable(rescue_release_task_integration_tests tests/release_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_release_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_release_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_release_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_release_task_integration_tests COMMAND rescue_release_task_integration_tests)
    set_tests_properties(rescue_release_stage_tests rescue_release_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_recovery_stage_tests tests/recovery_stage_tests.cpp
        src/task/recovery_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_recovery_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_recovery_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_recovery_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_recovery_stage_tests COMMAND rescue_recovery_stage_tests)
    add_executable(rescue_enter_stage_tests tests/enter_stage_tests.cpp
        src/task/enter_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_enter_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_enter_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_enter_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_enter_stage_tests COMMAND rescue_enter_stage_tests)
    add_executable(rescue_disperse_cluster_stage_tests tests/disperse_cluster_stage_tests.cpp
        src/task/disperse_cluster_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_disperse_cluster_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_disperse_cluster_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_disperse_cluster_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_disperse_cluster_stage_tests COMMAND rescue_disperse_cluster_stage_tests)
    add_executable(rescue_carry_approach_stage_tests tests/carry_approach_stage_tests.cpp
        src/task/carry_approach_stage.cpp src/task/carry_search_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_carry_approach_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_carry_approach_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_carry_approach_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_carry_approach_stage_tests COMMAND rescue_carry_approach_stage_tests)
    add_executable(rescue_carry_search_stage_tests tests/carry_search_stage_tests.cpp
        src/task/carry_search_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_carry_search_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_carry_search_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_carry_search_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_carry_search_stage_tests COMMAND rescue_carry_search_stage_tests)
    add_executable(rescue_sweep_trigger_stage_tests tests/sweep_trigger_stage_tests.cpp
        src/task/sweep_trigger_stage.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_sweep_trigger_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_sweep_trigger_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_sweep_trigger_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_sweep_trigger_stage_tests COMMAND rescue_sweep_trigger_stage_tests)
    add_executable(rescue_search_stage_tests tests/search_stage_tests.cpp
        src/task/search_stage.cpp src/task/sweep_ledger.cpp src/task/disperse_turn_stage.cpp
        src/task/disperse_swing_stage.cpp src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_search_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_search_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_search_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_search_stage_tests COMMAND rescue_search_stage_tests)
    add_executable(rescue_sweep_stage_tests tests/sweep_stage_tests.cpp
        src/task/sweep_stage.cpp src/task/sweep_ledger.cpp src/task/disperse_rush_stage.cpp
        src/task/disperse_turn_stage.cpp src/task/disperse_swing_stage.cpp src/task/transport_rules.cpp
        src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_sweep_stage_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_sweep_stage_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_sweep_stage_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_sweep_stage_tests COMMAND rescue_sweep_stage_tests)

    add_executable(rescue_delivery_verification_tests tests/delivery_verification_tests.cpp
        src/task/delivery_verification_stage.cpp src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_delivery_verification_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_delivery_verification_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_delivery_verification_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_delivery_verification_tests COMMAND rescue_delivery_verification_tests)

    add_executable(rescue_delivery_task_integration_tests tests/delivery_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_delivery_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_delivery_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_delivery_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_delivery_task_integration_tests COMMAND rescue_delivery_task_integration_tests)
    set_tests_properties(rescue_delivery_verification_tests rescue_delivery_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_capture_verification_tests tests/capture_verification_tests.cpp
        src/task/capture_verification_stage.cpp src/task/transport_rules.cpp)
    target_include_directories(rescue_capture_verification_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_capture_verification_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_capture_verification_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_capture_verification_tests COMMAND rescue_capture_verification_tests)

    add_executable(rescue_capture_task_integration_tests tests/capture_task_integration_tests.cpp
        ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_capture_task_integration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_capture_task_integration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_capture_task_integration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_capture_task_integration_tests COMMAND rescue_capture_task_integration_tests)
    set_tests_properties(rescue_capture_verification_tests rescue_capture_task_integration_tests PROPERTIES TIMEOUT 10)

    add_executable(rescue_push_tests tests/push_task_tests.cpp ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp
        src/perception/capture_monitor.cpp src/core/types.cpp src/config/config.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_push_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_push_tests PRIVATE ${OpenCV_LIBS})
    # These regression tests use assert, including in Release builds.
    target_compile_options(rescue_push_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_push_tests COMMAND rescue_push_tests)
    # The fixture was generated by the simulator with the TaskTuning presets.
    add_test(NAME rescue_push_replay COMMAND rescue_upper_host --startup-advance-ms 0 --pitch-presets 500,500,4000 --push-replay
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/push_delivery.json)
    set_tests_properties(rescue_push_replay PROPERTIES PASS_REGULAR_EXPRESSION "state=SCAN batch=0 delivered=1 first=1")
endif()

if(BUILD_TESTING)
    add_executable(rescue_vision_tests tests/model_io_tests.cpp src/perception/model_io.cpp src/perception/perception_adapter.cpp src/task/transport_rules.cpp
        src/perception/zone_keypoints.cpp)
    target_include_directories(rescue_vision_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_vision_tests PRIVATE rescue_rknn ${OpenCV_LIBS} ${CMAKE_DL_LIBS})
    target_compile_options(rescue_vision_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_vision_tests COMMAND rescue_vision_tests)

    add_executable(rescue_rknn_output_layout_tests tests/rknn_output_layout_tests.cpp
        src/perception/rknn_output_layout.cpp)
    target_include_directories(rescue_rknn_output_layout_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_rknn_output_layout_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_rknn_output_layout_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_rknn_output_layout_tests COMMAND rescue_rknn_output_layout_tests)
endif()

add_executable(rescue_sensor_monitor src/tools/sensor_monitor.cpp src/hardware/uart_controller.cpp src/core/utils.cpp)
target_include_directories(rescue_sensor_monitor PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_sensor_monitor PRIVATE ${OpenCV_LIBS} pthread)

if(BUILD_TESTING)
    add_executable(rescue_sensor_protocol_tests tests/sensor_protocol_tests.cpp src/hardware/uart_controller.cpp src/hardware/motion_link.cpp src/core/utils.cpp)
    target_include_directories(rescue_sensor_protocol_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_sensor_protocol_tests PRIVATE ${OpenCV_LIBS} pthread)
    target_compile_options(rescue_sensor_protocol_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_sensor_protocol_tests COMMAND rescue_sensor_protocol_tests)
    set_tests_properties(rescue_sensor_protocol_tests PROPERTIES TIMEOUT 10)
endif()

if(BUILD_TESTING AND RESCUE_ENABLE_TELEMETRY)
    add_executable(rescue_telemetry_tests tests/telemetry_tests.cpp src/telemetry/telemetry_publisher.cpp ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_telemetry_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_telemetry_tests PRIVATE rescue_app_config ${OpenCV_LIBS} pthread nlohmann_json::nlohmann_json)
    target_compile_options(rescue_telemetry_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_telemetry_tests COMMAND rescue_telemetry_tests)
    set_tests_properties(rescue_telemetry_tests PROPERTIES TIMEOUT 10)
endif()

add_executable(rescue_imu_monitor src/tools/imu_monitor.cpp src/hardware/hipnuc_imu.cpp src/core/utils.cpp)
target_include_directories(rescue_imu_monitor PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_imu_monitor PRIVATE ${OpenCV_LIBS} pthread)
if(BUILD_TESTING)
    add_executable(rescue_imu_tests tests/imu_tests.cpp src/hardware/hipnuc_imu.cpp src/config/config.cpp)
    target_include_directories(rescue_imu_tests PRIVATE include)
    target_link_libraries(rescue_imu_tests PRIVATE pthread)
    target_compile_options(rescue_imu_tests PRIVATE -UNDEBUG)
    target_compile_definitions(rescue_imu_tests PRIVATE IMU_FIXTURE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/hi91-capture.bin")
    add_test(NAME rescue_imu_tests COMMAND rescue_imu_tests)
    set_tests_properties(rescue_imu_tests PROPERTIES TIMEOUT 10)
endif()


if(BUILD_TESTING)
    add_executable(rescue_stage01_tests tests/stage01_tests.cpp
        src/geometry/zone_estimate.cpp src/geometry/zone_geometry.cpp src/perception/keypoint_filter.cpp
        src/geometry/ground_pose_fitter.cpp src/geometry/geometry_pipeline.cpp src/perception/silhouette.cpp src/geometry/camera_calibration.cpp
        src/geometry/safe_zone_pose.cpp src/geometry/zone_layout.cpp ${RESCUE_TASK_SOURCES} src/task/transport_rules.cpp
        src/hardware/uart_controller.cpp src/core/utils.cpp)
    target_include_directories(rescue_stage01_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_stage01_tests PRIVATE ${OpenCV_LIBS} pthread)
    target_compile_options(rescue_stage01_tests PRIVATE -UNDEBUG -Wall -Wextra)
    target_compile_definitions(rescue_stage01_tests PRIVATE ZONE_GEOMETRY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/zone_geometry.json"
        LEGACY_ZONE_GEOMETRY_PATH="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/zone_geometry_inner_v1.json")
    add_executable(rescue_silhouette_tests tests/silhouette_tests.cpp src/perception/silhouette.cpp)
    target_include_directories(rescue_silhouette_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_silhouette_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_silhouette_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_silhouette_tests COMMAND rescue_silhouette_tests)
    add_test(NAME rescue_stage01_tests COMMAND rescue_stage01_tests)
    set_tests_properties(rescue_stage01_tests PROPERTIES TIMEOUT 20)
    # Recorded fixture describes the red zone; independent of the live default team.
    add_test(NAME rescue_geometry_replay COMMAND rescue_upper_host --team red
        --geometry-replay ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/stage01_geometry.json
        --calibration ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/stage01-camera.synthetic.json
        --zone-geometry ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/zone_geometry_inner_v1.json)
    set_tests_properties(rescue_geometry_replay PROPERTIES
        PASS_REGULAR_EXPRESSION "Geometry replay frames=3 accepted=2 hardware_output=disabled" TIMEOUT 10)
endif()

if(BUILD_TESTING)
    add_executable(rescue_match_tests tests/match_tests.cpp src/control/match_control.cpp src/control/match_server.cpp)
    target_include_directories(rescue_match_tests PRIVATE include)
    target_link_libraries(rescue_match_tests PRIVATE pthread)
    target_compile_options(rescue_match_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_match_tests COMMAND rescue_match_tests)
    set_tests_properties(rescue_match_tests PROPERTIES TIMEOUT 10)
endif()

if(BUILD_TESTING)
    add_executable(rescue_task_calibration_tests tests/task_calibration_tests.cpp src/config/task_calibration.cpp)
    target_include_directories(rescue_task_calibration_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_task_calibration_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_task_calibration_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_task_calibration_tests COMMAND rescue_task_calibration_tests)
endif()

if(BUILD_TESTING)
    add_executable(rescue_navigation_tests tests/navigation_tests.cpp src/geometry/planner.cpp src/geometry/navigation_adapter.cpp
        src/geometry/zone_geometry.cpp src/geometry/zone_estimate.cpp)
    target_include_directories(rescue_navigation_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_navigation_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_navigation_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_navigation_tests COMMAND rescue_navigation_tests)
    set_tests_properties(rescue_navigation_tests PROPERTIES TIMEOUT 20)
endif()

if(BUILD_TESTING)
    add_executable(rescue_zone_color_tests tests/zone_color_tests.cpp src/perception/zone_color.cpp)
    target_include_directories(rescue_zone_color_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_zone_color_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_zone_color_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_zone_color_tests COMMAND rescue_zone_color_tests)
endif()

if(BUILD_TESTING)
    add_executable(rescue_controlled_field_tests tests/controlled_field_tests.cpp
        src/control/controlled_field.cpp src/geometry/zone_geometry.cpp src/geometry/zone_estimate.cpp src/task/transport_rules.cpp)
    target_include_directories(rescue_controlled_field_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_controlled_field_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_controlled_field_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_controlled_field_tests COMMAND rescue_controlled_field_tests)
endif()

if(BUILD_TESTING)
    add_test(NAME rescue_project_config_check COMMAND rescue_upper_host --config
        ${CMAKE_CURRENT_SOURCE_DIR}/config/project.json --check-config)
endif()

if(BUILD_TESTING)
    add_test(NAME rescue_controlled_config_check COMMAND rescue_upper_host --check-config
        --controlled-empty-field --team blue
        --calibration ${CMAKE_CURRENT_SOURCE_DIR}/config/camera.yaml
        --zone-geometry ${CMAKE_CURRENT_SOURCE_DIR}/config/zone_geometry.json
        --task-calibration ${CMAKE_CURRENT_SOURCE_DIR}/config/task_calibration.40_runtime.json
        --pose-model-blue models/zone_pose_fp.rknn)
    set_tests_properties(rescue_controlled_config_check PROPERTIES WILL_FAIL TRUE)
endif()

if(BUILD_TESTING)
    add_executable(async_log_tests tests/async_log_tests.cpp)
    target_include_directories(async_log_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(async_log_tests PRIVATE pthread)
    add_test(NAME async_log_tests COMMAND async_log_tests)
endif()

if(BUILD_TESTING)
    add_executable(rescue_mechanical_tests tests/mechanical_integration_tests.cpp src/task/transport_rules.cpp)
    target_include_directories(rescue_mechanical_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_mechanical_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_mechanical_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_mechanical_tests COMMAND rescue_mechanical_tests)
endif()

if(BUILD_TESTING)
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    add_test(NAME rescue_runtime_config_integration
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/runtime_config_integration.py
            $<TARGET_FILE:rescue_upper_host> ${CMAKE_CURRENT_SOURCE_DIR})
    set_tests_properties(rescue_runtime_config_integration PROPERTIES TIMEOUT 30)
    set(RESCUE_PROTOCOL_BASELINE_ARGS --protocol-baseline
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/runtime_protocol_20261023.json)
    set(RESCUE_PLATFORM_BASELINE_ARGS)
    if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
        list(APPEND RESCUE_PLATFORM_BASELINE_ARGS --platform-baseline
            ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/runtime_legacy_aarch64.json)
        message(STATUS "Legacy baseline: explicit verified aarch64 geometry expectation")
    endif()
    add_test(NAME rescue_runtime_legacy_baseline
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/runtime/record_baseline.py
            $<TARGET_FILE:rescue_upper_host> --compare ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/runtime_legacy_baseline.json
            ${RESCUE_PROTOCOL_BASELINE_ARGS}
            ${RESCUE_PLATFORM_BASELINE_ARGS})
    set_tests_properties(rescue_runtime_legacy_baseline PROPERTIES TIMEOUT 30)
    add_test(NAME rescue_runtime_baseline_comparator_tests
        COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/runtime/test_record_baseline.py)
    set_tests_properties(rescue_runtime_baseline_comparator_tests PROPERTIES TIMEOUT 10)
    add_test(NAME frame_virtual_serial_tests COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_frame_ctl.py)
    add_test(NAME motion_packet_contract_tests COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_motion_protocol.py)
    add_test(NAME camera_hold_tests COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pitch_prepare5.py)
    set_tests_properties(frame_virtual_serial_tests camera_hold_tests PROPERTIES TIMEOUT 15)
endif()

if(BUILD_TESTING)
    add_executable(rescue_demo_mode_tests tests/demo_mode_tests.cpp src/config/config.cpp)
    target_include_directories(rescue_demo_mode_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_demo_mode_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_demo_mode_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_demo_mode_tests COMMAND rescue_demo_mode_tests)
    add_test(NAME demo_launcher_tests COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_demo_launchers.py)
    set_tests_properties(demo_launcher_tests PROPERTIES TIMEOUT 30)
endif()
