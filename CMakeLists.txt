cmake_minimum_required(VERSION 3.10)

project(rescue_upper_host LANGUAGES CXX)

include(CTest)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)

add_executable(rescue_upper_host
    src/main.cpp
    src/push_task.cpp
    src/config.cpp
    src/types.cpp
    src/utils.cpp
    src/uart_controller.cpp
    src/yolo_detector.cpp
    src/model_io.cpp
    src/perception_adapter.cpp
    src/vision_logic.cpp
    src/camera_capture.cpp
    src/detector.cpp
    src/tracker.cpp
    src/camera_calibration.cpp
    src/safe_zone_pose.cpp
    src/zone_layout.cpp
    src/planner.cpp
    src/sensor_fusion.cpp
    src/logger.cpp
)

option(RESCUE_ENABLE_TELEMETRY "Build asynchronous telemetry snapshot publisher" OFF)
if(RESCUE_ENABLE_TELEMETRY)
    target_sources(rescue_upper_host PRIVATE src/telemetry_publisher.cpp)
    target_compile_definitions(rescue_upper_host PRIVATE RESCUE_ENABLE_TELEMETRY=1)
endif()

target_include_directories(rescue_upper_host PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(rescue_upper_host PRIVATE
    ${OpenCV_LIBS}
    pthread
    ${CMAKE_DL_LIBS}
)

if(BUILD_TESTING)
    add_executable(rescue_core_tests tests/rescue_core_tests.cpp
        src/types.cpp
        src/tracker.cpp
        src/camera_calibration.cpp
        src/safe_zone_pose.cpp
        src/zone_layout.cpp
        src/planner.cpp
        src/sensor_fusion.cpp
        src/rescue_state_machine.cpp
        src/logger.cpp
        src/uart_controller.cpp
        src/utils.cpp
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
    add_executable(rescue_push_tests tests/push_task_tests.cpp src/push_task.cpp src/config.cpp)
    target_include_directories(rescue_push_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_push_tests PRIVATE ${OpenCV_LIBS})
    # These regression tests use assert, including in Release builds.
    target_compile_options(rescue_push_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_push_tests COMMAND rescue_push_tests)
    add_test(NAME rescue_push_replay COMMAND rescue_upper_host --push-replay
        ${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures/push_delivery.json)
    set_tests_properties(rescue_push_replay PROPERTIES PASS_REGULAR_EXPRESSION "SEARCH_TARGET batch=0 delivered=1")
endif()

if(BUILD_TESTING)
    add_executable(rescue_vision_tests tests/model_io_tests.cpp src/model_io.cpp src/perception_adapter.cpp)
    target_include_directories(rescue_vision_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_vision_tests PRIVATE ${OpenCV_LIBS})
    target_compile_options(rescue_vision_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_vision_tests COMMAND rescue_vision_tests)
endif()

add_executable(rescue_sensor_monitor src/sensor_monitor.cpp src/uart_controller.cpp src/utils.cpp)
target_include_directories(rescue_sensor_monitor PRIVATE include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_sensor_monitor PRIVATE ${OpenCV_LIBS} pthread)

if(BUILD_TESTING)
    add_executable(rescue_sensor_protocol_tests tests/sensor_protocol_tests.cpp src/uart_controller.cpp src/utils.cpp)
    target_include_directories(rescue_sensor_protocol_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_sensor_protocol_tests PRIVATE ${OpenCV_LIBS} pthread)
    target_compile_options(rescue_sensor_protocol_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_sensor_protocol_tests COMMAND rescue_sensor_protocol_tests)
    set_tests_properties(rescue_sensor_protocol_tests PROPERTIES TIMEOUT 10)
endif()

if(BUILD_TESTING AND RESCUE_ENABLE_TELEMETRY)
    add_executable(rescue_telemetry_tests tests/telemetry_tests.cpp src/telemetry_publisher.cpp src/push_task.cpp)
    target_include_directories(rescue_telemetry_tests PRIVATE include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_telemetry_tests PRIVATE ${OpenCV_LIBS} pthread)
    target_compile_options(rescue_telemetry_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_telemetry_tests COMMAND rescue_telemetry_tests)
    set_tests_properties(rescue_telemetry_tests PROPERTIES TIMEOUT 10)
endif()
