# Test targets live here; names and output locations remain compatible.
find_package(Python3 COMPONENTS Interpreter REQUIRED)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}")

set(RESCUE_TEST_TASK_SOURCES)
foreach(source IN LISTS RESCUE_TASK_SOURCES)
    list(APPEND RESCUE_TEST_TASK_SOURCES "${PROJECT_SOURCE_DIR}/${source}")
endforeach()

function(rescue_add_task_test target source)
    add_executable(${target} "${CMAKE_CURRENT_SOURCE_DIR}/${source}")
    target_include_directories(${target} PRIVATE "${PROJECT_SOURCE_DIR}/include" ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${target} PRIVATE rescue_test_task_common ${OpenCV_LIBS})
    target_compile_options(${target} PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME ${target} COMMAND ${target})
endfunction()

# Keep assertions enabled in shared test code, independently of the Release app.
# Static linking pulls only the task/geometry objects needed by each test.
add_library(rescue_test_task_common STATIC
    ${RESCUE_TEST_TASK_SOURCES}
    ${PROJECT_SOURCE_DIR}/src/task/transport_rules.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_estimate.cpp
)
target_include_directories(rescue_test_task_common PUBLIC ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_test_task_common PUBLIC ${OpenCV_LIBS})
target_compile_options(rescue_test_task_common PRIVATE -UNDEBUG -Wall -Wextra)

add_executable(rescue_config_loader_tests ${PROJECT_SOURCE_DIR}/tests/config/config_loader_tests.cpp)
target_link_libraries(rescue_config_loader_tests PRIVATE rescue_app_config)
target_compile_options(rescue_config_loader_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_config_loader_tests COMMAND rescue_config_loader_tests)

add_executable(rescue_runtime_config_tests ${PROJECT_SOURCE_DIR}/tests/config/runtime_config_tests.cpp)
target_link_libraries(rescue_runtime_config_tests PRIVATE rescue_app_config)
target_compile_options(rescue_runtime_config_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_runtime_config_tests COMMAND rescue_runtime_config_tests)

add_executable(rescue_core_tests ${PROJECT_SOURCE_DIR}/tests/regression/rescue_core_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/core/types.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/tracker.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/camera_calibration.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/safe_zone_pose.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_layout.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/planner.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/sensor_fusion.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/imu_adapter.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/hipnuc_imu.cpp
    ${PROJECT_SOURCE_DIR}/src/core/logger.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/uart_controller.cpp
    ${PROJECT_SOURCE_DIR}/src/core/utils.cpp)
target_include_directories(rescue_core_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(rescue_core_tests PRIVATE rescue_test_task_common ${OpenCV_LIBS} pthread)
target_compile_options(rescue_core_tests PRIVATE -UNDEBUG)
add_test(NAME rescue_core_tests COMMAND rescue_core_tests)

rescue_add_task_test(rescue_disperse_swing_stage_tests task/stages/disperse_swing_stage_tests.cpp)

rescue_add_task_test(rescue_disperse_rush_stage_tests task/stages/disperse_rush_stage_tests.cpp)

set_tests_properties(rescue_disperse_rush_stage_tests PROPERTIES TIMEOUT 10)

rescue_add_task_test(rescue_sweep_ledger_tests task/stages/sweep_ledger_tests.cpp)

set_tests_properties(rescue_sweep_ledger_tests PROPERTIES TIMEOUT 10)

rescue_add_task_test(rescue_push_in_stage_tests task/stages/push_in_stage_tests.cpp)

rescue_add_task_test(rescue_push_in_task_integration_tests task/integration/push_in_task_integration_tests.cpp)
set_tests_properties(rescue_push_in_stage_tests rescue_push_in_task_integration_tests PROPERTIES TIMEOUT 10)

rescue_add_task_test(rescue_release_stage_tests task/stages/release_stage_tests.cpp)

rescue_add_task_test(rescue_release_task_integration_tests task/integration/release_task_integration_tests.cpp)
set_tests_properties(rescue_release_stage_tests rescue_release_task_integration_tests PROPERTIES TIMEOUT 10)

rescue_add_task_test(rescue_recovery_stage_tests task/stages/recovery_stage_tests.cpp)
rescue_add_task_test(rescue_enter_stage_tests task/stages/enter_stage_tests.cpp)
rescue_add_task_test(rescue_disperse_cluster_stage_tests task/stages/disperse_cluster_stage_tests.cpp)
rescue_add_task_test(rescue_carry_approach_stage_tests task/stages/carry_approach_stage_tests.cpp)
rescue_add_task_test(rescue_carry_task_integration_tests task/integration/carry_task_integration_tests.cpp)
set_tests_properties(rescue_carry_task_integration_tests PROPERTIES TIMEOUT 10 LABELS "integration;task")
rescue_add_task_test(rescue_carry_search_stage_tests task/stages/carry_search_stage_tests.cpp)
rescue_add_task_test(rescue_sweep_trigger_stage_tests task/stages/sweep_trigger_stage_tests.cpp)
rescue_add_task_test(rescue_search_stage_tests task/stages/search_stage_tests.cpp)

rescue_add_task_test(rescue_delivery_verification_tests task/stages/delivery_verification_tests.cpp)

rescue_add_task_test(rescue_delivery_task_integration_tests task/integration/delivery_task_integration_tests.cpp)
set_tests_properties(rescue_delivery_verification_tests rescue_delivery_task_integration_tests PROPERTIES TIMEOUT 10)

rescue_add_task_test(rescue_capture_verification_tests task/stages/capture_verification_tests.cpp)

rescue_add_task_test(rescue_capture_task_integration_tests task/integration/capture_task_integration_tests.cpp)
set_tests_properties(rescue_capture_verification_tests rescue_capture_task_integration_tests PROPERTIES TIMEOUT 10)

add_executable(rescue_first_trip_capture_review_tests
    task/integration/first_trip_capture_review_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/capture_monitor.cpp
    ${PROJECT_SOURCE_DIR}/src/core/types.cpp)
target_include_directories(rescue_first_trip_capture_review_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_first_trip_capture_review_tests PRIVATE
    rescue_test_task_common rescue_app_config ${OpenCV_LIBS} nlohmann_json::nlohmann_json)
# Large scenario fixture: avoid GCC IPA optimizer failures; production stays at Release settings.
target_compile_options(rescue_first_trip_capture_review_tests PRIVATE -O0 -UNDEBUG -Wall -Wextra)
foreach(scenario dangerous-front-risk supplied-grasp-ranges injured-removal postsplit-search search-view-selection
                 front-split-recovery upper-corner-motion corner-imu-drift corner-turn-brake capture-checkpoint postsplit-approach split-preempt retreat-overshoot retreat-continuity startup-first-grab postclose-held-only)
    add_test(NAME rescue_capture_${scenario}
        COMMAND rescue_first_trip_capture_review_tests
            ${PROJECT_SOURCE_DIR}/tests/fixtures/first_trip_20261029_212327_boxes.txt ${scenario})
    set_tests_properties(rescue_capture_${scenario} PROPERTIES TIMEOUT 15 LABELS "integration;task;perception")
endforeach()

add_executable(rescue_observation_builder_capture_tests
    app/observation_builder_capture_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/app/observation_builder.cpp
    ${PROJECT_SOURCE_DIR}/src/control/controlled_field.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/capture_monitor.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/pushboard_floor.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/zone_color.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/perception_adapter.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/silhouette.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/zone_keypoints.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/keypoint_filter.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/ground_pose_fitter.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/geometry_pipeline.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/camera_calibration.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/safe_zone_pose.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_layout.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/planner.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/hipnuc_imu.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/imu_adapter.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/uart_controller.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/sensor_fusion.cpp
    ${PROJECT_SOURCE_DIR}/src/core/utils.cpp
    ${PROJECT_SOURCE_DIR}/src/core/types.cpp
    ${PROJECT_SOURCE_DIR}/src/core/logger.cpp)
target_include_directories(rescue_observation_builder_capture_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_observation_builder_capture_tests PRIVATE
    rescue_test_task_common rescue_app_config rescue_rknn ${OpenCV_LIBS} pthread)
target_compile_options(rescue_observation_builder_capture_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_observation_builder_capture_tests COMMAND rescue_observation_builder_capture_tests)
set_tests_properties(rescue_observation_builder_capture_tests PROPERTIES TIMEOUT 15
    LABELS "integration;task;perception")
add_test(NAME rescue_near_target_selection_tests
    COMMAND rescue_observation_builder_capture_tests --near-selection-only)
set_tests_properties(rescue_near_target_selection_tests PROPERTIES TIMEOUT 15
    LABELS "integration;task;perception")

add_test(NAME rescue_observation_builder_zone_sight_tests
    COMMAND rescue_observation_builder_capture_tests --zone-sight-only)
set_tests_properties(rescue_observation_builder_zone_sight_tests PROPERTIES TIMEOUT 15
    LABELS "integration;task;perception")


rescue_add_task_test(rescue_task_phase_tests task/integration/task_phase_tests.cpp)
add_test(NAME rescue_search_capture_boundary_tests COMMAND rescue_task_phase_tests --search-capture-boundary)
set_tests_properties(rescue_search_capture_boundary_tests PROPERTIES TIMEOUT 10 LABELS "integration;task")
add_test(NAME rescue_search_ui120_speed_tests COMMAND rescue_task_phase_tests --search-ui120-speed)
add_test(NAME rescue_search_ui120_handoff_tests COMMAND rescue_task_phase_tests --search-ui120-handoff)
set_tests_properties(rescue_search_ui120_speed_tests rescue_search_ui120_handoff_tests
    PROPERTIES TIMEOUT 10 LABELS "integration;task")
add_test(NAME rescue_search_unseen_zone_tests COMMAND rescue_task_phase_tests --search-unseen-zone)
set_tests_properties(rescue_search_unseen_zone_tests PROPERTIES TIMEOUT 10 LABELS "integration;task")
add_test(NAME rescue_search_region_builder_tests
    COMMAND rescue_observation_builder_capture_tests --search-region-builder-only)
set_tests_properties(rescue_search_region_builder_tests PROPERTIES TIMEOUT 15 LABELS "integration;task;perception")

set_tests_properties(rescue_task_phase_tests PROPERTIES TIMEOUT 10 LABELS "integration;task")

add_executable(rescue_push_tests ${PROJECT_SOURCE_DIR}/tests/task/regression/push_task_tests.cpp
    task/regression/push_task_capture_cases.cpp
    task/regression/push_task_search_cases.cpp
    task/regression/push_task_sweep_actions_cases.cpp
    task/regression/push_task_sweep_scenarios_cases.cpp
    task/regression/push_task_delivery_cases.cpp
    task/regression/push_task_control_cases.cpp
    task/regression/push_task_fixture.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/capture_monitor.cpp
    ${PROJECT_SOURCE_DIR}/src/core/types.cpp
    ${PROJECT_SOURCE_DIR}/src/config/config.cpp)
target_include_directories(rescue_push_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_push_tests PRIVATE rescue_test_task_common ${OpenCV_LIBS})
# GCC 13 on the board ICEs in IPA ICF while hashing this large fixture.
# Disable only that test-file optimization; production retains Release options.
set_property(SOURCE task/regression/push_task_delivery_cases.cpp APPEND
    PROPERTY COMPILE_OPTIONS "$<$<CXX_COMPILER_ID:GNU>:-fno-ipa-icf>")
# These regression tests use assert, including in Release builds.
target_compile_options(rescue_push_tests PRIVATE -UNDEBUG)
add_test(NAME rescue_target_selection_tests COMMAND rescue_push_tests --selection-only)
set_tests_properties(rescue_target_selection_tests PROPERTIES TIMEOUT 10 LABELS "integration;task;perception")
add_test(NAME rescue_far_target_smoothing_tests COMMAND rescue_push_tests --far-smoothing-only)
set_tests_properties(rescue_far_target_smoothing_tests PROPERTIES TIMEOUT 10 LABELS "integration;task")
add_test(NAME rescue_pixel_capture_tests COMMAND rescue_push_tests --capture-only)
set_tests_properties(rescue_pixel_capture_tests PROPERTIES TIMEOUT 10 LABELS "integration;task")
# The fixture was generated by the simulator with the TaskTuning presets.
add_test(NAME rescue_push_replay COMMAND rescue_upper_host --startup-advance-ms 0 --pitch-presets 500,500,4000 --push-replay
    ${PROJECT_SOURCE_DIR}/tests/fixtures/push_delivery.json)
set_tests_properties(rescue_push_replay PROPERTIES PASS_REGULAR_EXPRESSION "state=SCAN batch=0 delivered=1 first=1")

add_executable(rescue_vision_tests ${PROJECT_SOURCE_DIR}/tests/perception/model_io_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/model_io.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/perception_adapter.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/zone_keypoints.cpp)
target_include_directories(rescue_vision_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_vision_tests PRIVATE rescue_test_task_common rescue_rknn ${OpenCV_LIBS} ${CMAKE_DL_LIBS})
target_compile_options(rescue_vision_tests PRIVATE -UNDEBUG)
add_test(NAME rescue_vision_tests COMMAND rescue_vision_tests)

add_executable(rescue_rknn_output_layout_tests ${PROJECT_SOURCE_DIR}/tests/perception/rknn_output_layout_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/rknn_output_layout.cpp)
target_include_directories(rescue_rknn_output_layout_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_rknn_output_layout_tests PRIVATE ${OpenCV_LIBS})
target_compile_options(rescue_rknn_output_layout_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_rknn_output_layout_tests COMMAND rescue_rknn_output_layout_tests)

add_executable(rescue_sensor_protocol_tests ${PROJECT_SOURCE_DIR}/tests/hardware/sensor_protocol_tests.cpp ${PROJECT_SOURCE_DIR}/src/hardware/uart_controller.cpp ${PROJECT_SOURCE_DIR}/src/hardware/motion_link.cpp ${PROJECT_SOURCE_DIR}/src/core/utils.cpp)
target_include_directories(rescue_sensor_protocol_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_sensor_protocol_tests PRIVATE ${OpenCV_LIBS} pthread)
target_compile_options(rescue_sensor_protocol_tests PRIVATE -UNDEBUG)
add_test(NAME rescue_sensor_protocol_tests COMMAND rescue_sensor_protocol_tests)
set_tests_properties(rescue_sensor_protocol_tests PROPERTIES TIMEOUT 10)

add_executable(rescue_uart_latency_tests hardware/uart_latency_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/motion_link.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/uart_controller.cpp
    ${PROJECT_SOURCE_DIR}/src/control/match_control.cpp
    ${PROJECT_SOURCE_DIR}/src/core/utils.cpp)
target_include_directories(rescue_uart_latency_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_uart_latency_tests PRIVATE pthread
    -Wl,--wrap=tcdrain -Wl,--wrap=write)
target_compile_options(rescue_uart_latency_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_uart_latency_tests COMMAND rescue_uart_latency_tests)
set_tests_properties(rescue_uart_latency_tests PROPERTIES TIMEOUT 5)

if(RESCUE_ENABLE_TELEMETRY)
    add_executable(rescue_telemetry_tests ${PROJECT_SOURCE_DIR}/tests/telemetry/telemetry_tests.cpp
        ${PROJECT_SOURCE_DIR}/src/telemetry/telemetry_publisher.cpp)
    target_include_directories(rescue_telemetry_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_telemetry_tests PRIVATE rescue_test_task_common rescue_app_config ${OpenCV_LIBS} pthread nlohmann_json::nlohmann_json)
    target_compile_options(rescue_telemetry_tests PRIVATE -UNDEBUG)
    add_test(NAME rescue_telemetry_tests COMMAND rescue_telemetry_tests)
    set_tests_properties(rescue_telemetry_tests PROPERTIES TIMEOUT 10)
endif()

add_executable(rescue_imu_tests ${PROJECT_SOURCE_DIR}/tests/hardware/imu_tests.cpp ${PROJECT_SOURCE_DIR}/src/hardware/hipnuc_imu.cpp ${PROJECT_SOURCE_DIR}/src/config/config.cpp)
target_include_directories(rescue_imu_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(rescue_imu_tests PRIVATE pthread)
target_compile_options(rescue_imu_tests PRIVATE -UNDEBUG)
target_compile_definitions(rescue_imu_tests PRIVATE IMU_FIXTURE_PATH="${PROJECT_SOURCE_DIR}/tests/fixtures/hi91-capture.bin")
add_test(NAME rescue_imu_tests COMMAND rescue_imu_tests)
set_tests_properties(rescue_imu_tests PROPERTIES TIMEOUT 10)

add_executable(rescue_stage01_tests ${PROJECT_SOURCE_DIR}/tests/geometry/geometry_pipeline_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/keypoint_filter.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/ground_pose_fitter.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/geometry_pipeline.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/silhouette.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/camera_calibration.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/safe_zone_pose.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_layout.cpp
    ${PROJECT_SOURCE_DIR}/src/hardware/uart_controller.cpp
    ${PROJECT_SOURCE_DIR}/src/core/utils.cpp)
target_include_directories(rescue_stage01_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_stage01_tests PRIVATE rescue_test_task_common ${OpenCV_LIBS} pthread)
target_compile_options(rescue_stage01_tests PRIVATE -UNDEBUG -Wall -Wextra)
target_compile_definitions(rescue_stage01_tests PRIVATE PROJECT_CONFIG_PATH="${PROJECT_SOURCE_DIR}/config/project.json"
    LEGACY_ZONE_GEOMETRY_PATH="${PROJECT_SOURCE_DIR}/tests/fixtures/zone_geometry_inner_v1.json")
add_executable(rescue_silhouette_tests ${PROJECT_SOURCE_DIR}/tests/perception/silhouette_tests.cpp ${PROJECT_SOURCE_DIR}/src/perception/silhouette.cpp)
target_include_directories(rescue_silhouette_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_silhouette_tests PRIVATE ${OpenCV_LIBS})
target_compile_options(rescue_silhouette_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_silhouette_tests COMMAND rescue_silhouette_tests)
add_test(NAME rescue_stage01_tests COMMAND rescue_stage01_tests)
set_tests_properties(rescue_stage01_tests PROPERTIES TIMEOUT 20)
# Recorded fixture describes the red zone; independent of the live default team.
add_test(NAME rescue_geometry_replay COMMAND rescue_upper_host --team red
    --geometry-replay ${PROJECT_SOURCE_DIR}/tests/fixtures/stage01_geometry.json
    --calibration ${PROJECT_SOURCE_DIR}/tests/fixtures/stage01-camera.synthetic.json
    --zone-geometry ${PROJECT_SOURCE_DIR}/tests/fixtures/zone_geometry_inner_v1.json)
set_tests_properties(rescue_geometry_replay PROPERTIES
    PASS_REGULAR_EXPRESSION "Geometry replay frames=3 accepted=2 hardware_output=disabled" TIMEOUT 10)

add_executable(rescue_match_tests ${PROJECT_SOURCE_DIR}/tests/control/match_tests.cpp ${PROJECT_SOURCE_DIR}/src/control/match_control.cpp ${PROJECT_SOURCE_DIR}/src/control/match_server.cpp)
target_include_directories(rescue_match_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(rescue_match_tests PRIVATE pthread)
target_compile_options(rescue_match_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_match_tests COMMAND rescue_match_tests)
set_tests_properties(rescue_match_tests PROPERTIES TIMEOUT 10)

add_executable(rescue_task_calibration_tests ${PROJECT_SOURCE_DIR}/tests/config/task_calibration_tests.cpp ${PROJECT_SOURCE_DIR}/src/config/task_calibration.cpp)
target_include_directories(rescue_task_calibration_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_task_calibration_tests PRIVATE ${OpenCV_LIBS})
target_compile_options(rescue_task_calibration_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_task_calibration_tests COMMAND rescue_task_calibration_tests)

add_executable(rescue_navigation_tests ${PROJECT_SOURCE_DIR}/tests/geometry/navigation_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/planner.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/navigation_adapter.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_geometry.cpp)
target_include_directories(rescue_navigation_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_navigation_tests PRIVATE rescue_test_task_common ${OpenCV_LIBS})
target_compile_options(rescue_navigation_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_navigation_tests COMMAND rescue_navigation_tests)
set_tests_properties(rescue_navigation_tests PROPERTIES TIMEOUT 20)

add_executable(rescue_zone_color_tests ${PROJECT_SOURCE_DIR}/tests/perception/zone_color_tests.cpp ${PROJECT_SOURCE_DIR}/src/perception/zone_color.cpp)
target_include_directories(rescue_zone_color_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_zone_color_tests PRIVATE ${OpenCV_LIBS})
target_compile_options(rescue_zone_color_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_zone_color_tests COMMAND rescue_zone_color_tests)

add_executable(rescue_controlled_field_tests ${PROJECT_SOURCE_DIR}/tests/control/controlled_field_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/control/controlled_field.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_geometry.cpp)
target_include_directories(rescue_controlled_field_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_controlled_field_tests PRIVATE rescue_test_task_common ${OpenCV_LIBS})
target_compile_options(rescue_controlled_field_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_controlled_field_tests COMMAND rescue_controlled_field_tests)

add_test(NAME rescue_project_config_check COMMAND rescue_upper_host --config
    ${PROJECT_SOURCE_DIR}/config/project.json --check-config --known-frame-angle 0 --known-frame-id 1)

add_test(NAME rescue_controlled_config_check
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/config/controlled_config_check.py
        $<TARGET_FILE:rescue_upper_host> ${PROJECT_SOURCE_DIR})
set_tests_properties(rescue_controlled_config_check PROPERTIES TIMEOUT 20)

add_executable(async_log_tests ${PROJECT_SOURCE_DIR}/tests/telemetry/async_log_tests.cpp)
target_include_directories(async_log_tests PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(async_log_tests PRIVATE pthread)
add_test(NAME async_log_tests COMMAND async_log_tests)

rescue_add_task_test(rescue_mechanical_tests perception/capture_selection_integration_tests.cpp)
target_sources(rescue_mechanical_tests PRIVATE ${PROJECT_SOURCE_DIR}/src/perception/perception_adapter.cpp)

add_test(NAME rescue_runtime_config_integration
    COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/config/runtime_config_integration.py
        $<TARGET_FILE:rescue_upper_host> ${PROJECT_SOURCE_DIR})
set_tests_properties(rescue_runtime_config_integration PROPERTIES TIMEOUT 30)
add_test(NAME frame_virtual_serial_tests COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/tools/test_frame_ctl.py)
add_test(NAME motion_packet_contract_tests COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/hardware/test_motion_protocol.py)
add_test(NAME camera_hold_tests COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/tools/test_pitch_prepare5.py)
set_tests_properties(frame_virtual_serial_tests camera_hold_tests PROPERTIES TIMEOUT 15)

add_executable(rescue_demo_mode_tests ${PROJECT_SOURCE_DIR}/tests/config/demo_mode_tests.cpp ${PROJECT_SOURCE_DIR}/src/config/config.cpp)
target_include_directories(rescue_demo_mode_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_demo_mode_tests PRIVATE ${OpenCV_LIBS})
target_compile_options(rescue_demo_mode_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_demo_mode_tests COMMAND rescue_demo_mode_tests)
add_test(NAME demo_launcher_tests COMMAND ${Python3_EXECUTABLE} ${PROJECT_SOURCE_DIR}/tests/tools/test_demo_launchers.py)
set_tests_properties(demo_launcher_tests PROPERTIES TIMEOUT 30)

get_property(existing_tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${existing_tests} PROPERTIES WORKING_DIRECTORY "${PROJECT_BINARY_DIR}")

# Tool suites use synthetic images, temporary files and loopback services.
# Missing dependencies fail visibly instead of silently removing tests.
execute_process(COMMAND "${Python3_EXECUTABLE}" -c "import cv2, numpy, websockets"
    RESULT_VARIABLE tool_dependencies_result ERROR_VARIABLE tool_dependencies_error)
if(NOT tool_dependencies_result EQUAL 0)
    message(FATAL_ERROR "Python tool test dependencies are missing: ${tool_dependencies_error}")
endif()
function(rescue_add_tool_test name script labels)
    get_filename_component(script_dir "${PROJECT_SOURCE_DIR}/${script}" DIRECTORY)
    add_test(NAME ${name} COMMAND "${Python3_EXECUTABLE}" "${PROJECT_SOURCE_DIR}/${script}")
    set_tests_properties(${name} PROPERTIES WORKING_DIRECTORY "${script_dir}"
        TIMEOUT 60 LABELS "${labels}")
endfunction()
rescue_add_tool_test(board_monitor_tests tools/board_monitor/test_monitor.py "tool;telemetry")
rescue_add_tool_test(camera_overlap_tests tools/camera_calibration/test_overlap.py "tool;calibration")
rescue_add_tool_test(camera_calibration_tests tools/camera_calibration/test_calibrate.py "tool;calibration")
rescue_add_tool_test(camera_guided_tests tools/camera_calibration/test_guided.py "tool;calibration")
rescue_add_tool_test(camera_handeye_tests tools/camera_calibration/test_handeye.py "tool;calibration")
rescue_add_tool_test(camera_foxglove_bridge_tests tools/camera_calibration/test_foxglove_bridge.py "tool;calibration;telemetry")
rescue_add_tool_test(project_telemetry_bridge_tests tools/remote_camera_telemetry/test_project_bridge.py "tool;telemetry")

# Each test has a layer label and subsystem labels.
set_tests_properties(rescue_config_loader_tests PROPERTIES LABELS "unit;config")
set_tests_properties(rescue_runtime_config_tests PROPERTIES LABELS "unit;config")
set_tests_properties(rescue_core_tests PROPERTIES LABELS "regression;core")
set_tests_properties(rescue_disperse_swing_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_disperse_rush_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_sweep_ledger_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_push_in_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_push_in_task_integration_tests PROPERTIES LABELS "integration;task")
set_tests_properties(rescue_release_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_release_task_integration_tests PROPERTIES LABELS "integration;task")
set_tests_properties(rescue_recovery_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_enter_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_disperse_cluster_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_carry_approach_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_carry_search_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_sweep_trigger_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_search_stage_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_delivery_verification_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_delivery_task_integration_tests PROPERTIES LABELS "integration;task")
set_tests_properties(rescue_capture_verification_tests PROPERTIES LABELS "unit;task")
set_tests_properties(rescue_capture_task_integration_tests PROPERTIES LABELS "integration;task")
set_tests_properties(rescue_push_replay PROPERTIES LABELS "regression;task")
set_tests_properties(rescue_vision_tests PROPERTIES LABELS "unit;perception")
set_tests_properties(rescue_rknn_output_layout_tests PROPERTIES LABELS "unit;perception")
set_tests_properties(rescue_sensor_protocol_tests PROPERTIES LABELS "unit;protocol")
if(RESCUE_ENABLE_TELEMETRY)
set_tests_properties(rescue_telemetry_tests PROPERTIES LABELS "integration;telemetry")
endif()
set_tests_properties(rescue_imu_tests PROPERTIES LABELS "unit;hardware")
set_tests_properties(rescue_silhouette_tests PROPERTIES LABELS "unit;perception")
set_tests_properties(rescue_stage01_tests PROPERTIES LABELS "integration;geometry")
set_tests_properties(rescue_geometry_replay PROPERTIES LABELS "regression;geometry")
set_tests_properties(rescue_match_tests PROPERTIES LABELS "unit;control")
set_tests_properties(rescue_task_calibration_tests PROPERTIES LABELS "unit;config")
set_tests_properties(rescue_navigation_tests PROPERTIES LABELS "unit;geometry")
set_tests_properties(rescue_zone_color_tests PROPERTIES LABELS "unit;perception")
set_tests_properties(rescue_controlled_field_tests PROPERTIES LABELS "unit;control")
set_tests_properties(rescue_project_config_check PROPERTIES LABELS "integration;config")
set_tests_properties(rescue_controlled_config_check PROPERTIES LABELS "integration;config")
set_tests_properties(async_log_tests PROPERTIES LABELS "unit;telemetry")
set_tests_properties(rescue_mechanical_tests PROPERTIES LABELS "integration;perception")
set_tests_properties(rescue_runtime_config_integration PROPERTIES LABELS "integration;config")
set_tests_properties(frame_virtual_serial_tests PROPERTIES LABELS "tool;protocol")
set_tests_properties(motion_packet_contract_tests PROPERTIES LABELS "tool;protocol")
set_tests_properties(camera_hold_tests PROPERTIES LABELS "tool;protocol")
set_tests_properties(rescue_demo_mode_tests PROPERTIES LABELS "unit;config")
set_tests_properties(demo_launcher_tests PROPERTIES LABELS "tool;config")

get_property(all_tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${all_tests} PROPERTIES ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1")

rescue_add_task_test(rescue_search_entry_tests task/integration/search_entry_tests.cpp)
set_tests_properties(rescue_search_entry_tests PROPERTIES LABELS "integration;task")

add_executable(rescue_pushboard_floor_tests perception/pushboard_floor_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/pushboard_floor.cpp)
target_include_directories(rescue_pushboard_floor_tests PRIVATE
    ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_pushboard_floor_tests PRIVATE ${OpenCV_LIBS})
target_compile_options(rescue_pushboard_floor_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_pushboard_floor_tests COMMAND rescue_pushboard_floor_tests)

add_test(NAME rescue_capture_score_tests
    COMMAND rescue_first_trip_capture_review_tests
        ${PROJECT_SOURCE_DIR}/tests/fixtures/first_trip_20261029_212327_boxes.txt capture-score)
set_tests_properties(rescue_capture_score_tests PROPERTIES TIMEOUT 15 LABELS "integration;task;perception")


# Real SDK regression: inference and teardown only; never opens UART, IMU or camera.
if(RESCUE_ENABLE_RKNN AND
   EXISTS "${PROJECT_SOURCE_DIR}/models/detect_v5_int8.rknn" AND
   EXISTS "${PROJECT_SOURCE_DIR}/models/zone_pose_int8.rknn" AND
   EXISTS "${PROJECT_SOURCE_DIR}/benchmark_results/librknnrt.so")
    add_executable(rescue_parallel_inference_tests
        perception/perception_pipeline_tests.cpp
        ${PROJECT_SOURCE_DIR}/src/app/perception_pipeline.cpp
        ${PROJECT_SOURCE_DIR}/src/perception/detector.cpp
        ${PROJECT_SOURCE_DIR}/src/perception/yolo_detector.cpp
        ${PROJECT_SOURCE_DIR}/src/perception/model_io.cpp
        ${PROJECT_SOURCE_DIR}/src/perception/zone_keypoints.cpp
        ${PROJECT_SOURCE_DIR}/src/core/utils.cpp)
    target_include_directories(rescue_parallel_inference_tests PRIVATE
        ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(rescue_parallel_inference_tests PRIVATE
        rescue_rknn rescue_pose_smoothing ${OpenCV_LIBS} pthread)
    target_compile_options(rescue_parallel_inference_tests PRIVATE -UNDEBUG -Wall -Wextra)
    add_test(NAME rescue_parallel_inference_tests COMMAND rescue_parallel_inference_tests
        ${PROJECT_SOURCE_DIR}/models/detect_v5_int8.rknn
        ${PROJECT_SOURCE_DIR}/models/zone_pose_int8.rknn
        ${PROJECT_SOURCE_DIR}/benchmark_results/librknnrt.so
        ${PROJECT_SOURCE_DIR}/tests/samples/ui120-left-half.jpg)
    add_test(NAME rescue_parallel_inference_cli COMMAND rescue_upper_host
        --config ${PROJECT_SOURCE_DIR}/config/project.json
        --calibration ""
        --detect-image ${PROJECT_SOURCE_DIR}/tests/samples/ui120-left-half.jpg
        --no-show --parallel-infer)
    set_tests_properties(rescue_parallel_inference_tests rescue_parallel_inference_cli
        PROPERTIES TIMEOUT 30 LABELS "integration;perception"
        ENVIRONMENT "PYTHONDONTWRITEBYTECODE=1;ASAN_OPTIONS=detect_leaks=0:detect_stack_use_after_return=0;UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
endif()

add_executable(rescue_task_revision_tests
    perception/task_revision_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/control/controlled_field.cpp
    ${PROJECT_SOURCE_DIR}/src/geometry/zone_geometry.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/perception_adapter.cpp)
target_link_libraries(rescue_task_revision_tests PRIVATE rescue_test_task_common)
# GCC 13 on the board ICEs in the fixture's GIMPLE PRE pass at -O3.
# Production and the shared task implementation retain Release optimization.
target_compile_options(rescue_task_revision_tests PRIVATE -O0 -UNDEBUG -Wall -Wextra)
foreach(scenario startup score transport reject)
    add_test(NAME rescue_revision_${scenario} COMMAND rescue_task_revision_tests ${scenario})
    set_tests_properties(rescue_revision_${scenario} PROPERTIES TIMEOUT 15 LABELS "regression;task;perception")
endforeach()

add_test(NAME rescue_transport_inspected_lifecycle
    COMMAND rescue_carry_task_integration_tests --transport-inspection)
set_tests_properties(rescue_transport_inspected_lifecycle PROPERTIES TIMEOUT 15 LABELS "integration;task")

# Keep large synthetic fixtures out of GCC 13's failing O3 passes on the
# board (observation builder and carry integration); production stays optimized.
set_source_files_properties(app/observation_builder_capture_tests.cpp
    task/integration/carry_task_integration_tests.cpp
    PROPERTIES COMPILE_OPTIONS "-O0")

add_executable(rescue_pose_smoothing_tests perception/pose_smoothing_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/zone_keypoints.cpp)
target_include_directories(rescue_pose_smoothing_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_pose_smoothing_tests PRIVATE rescue_pose_smoothing rescue_rknn ${OpenCV_LIBS})
target_compile_options(rescue_pose_smoothing_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_pose_smoothing_tests COMMAND rescue_pose_smoothing_tests)

add_executable(rescue_dual_expert_pose_tests
    perception/dual_expert_pose_tests.cpp
    ${PROJECT_SOURCE_DIR}/src/app/perception_pipeline.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/detector.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/yolo_detector.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/model_io.cpp
    ${PROJECT_SOURCE_DIR}/src/perception/zone_keypoints.cpp
    ${PROJECT_SOURCE_DIR}/src/core/utils.cpp)
target_include_directories(rescue_dual_expert_pose_tests PRIVATE ${PROJECT_SOURCE_DIR}/include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(rescue_dual_expert_pose_tests PRIVATE rescue_rknn rescue_pose_smoothing nlohmann_json::nlohmann_json ${OpenCV_LIBS} pthread)
target_compile_options(rescue_dual_expert_pose_tests PRIVATE -UNDEBUG -Wall -Wextra)
add_test(NAME rescue_dual_expert_pose_tests COMMAND rescue_dual_expert_pose_tests)

if(RESCUE_ENABLE_RKNN AND EXISTS "${PROJECT_SOURCE_DIR}/models/a_int8_20261105/A_INT8_confidence_fixed_EXPERIMENTAL.rknn")
    add_test(NAME rescue_dual_expert_pose_cli COMMAND rescue_upper_host
        --config ${PROJECT_SOURCE_DIR}/config/project.json --calibration ""
        --pose-model ${PROJECT_SOURCE_DIR}/models/a_int8_20261105/A_INT8_confidence_fixed_EXPERIMENTAL.rknn
        --detect-image ${PROJECT_SOURCE_DIR}/models/a_int8_20261105/fixtures/near_f000000.png
        --pose-preview overlay --no-show --parallel-infer)
    set_tests_properties(rescue_dual_expert_pose_cli PROPERTIES TIMEOUT 30 LABELS "integration;perception")
endif()

# Validate the configured default with the real NPU, without opening devices.
if(RESCUE_ENABLE_RKNN AND EXISTS "${PROJECT_SOURCE_DIR}/models/a_rawxy_20261105/A_rawxy_INT8_EXPERIMENTAL.rknn")
    add_test(NAME rescue_rawxy_pose_cli COMMAND rescue_upper_host
        --config ${PROJECT_SOURCE_DIR}/config/project.json --calibration ""
        --detect-image ${PROJECT_SOURCE_DIR}/models/a_int8_20261105/fixtures/near_f000000.png
        --pose-preview overlay --no-show --parallel-infer)
    set_tests_properties(rescue_rawxy_pose_cli PROPERTIES TIMEOUT 30 LABELS "integration;perception"
        PASS_REGULAR_EXPRESSION "base_xy_raw")
endif()

# A7 repeat-start acceptance: pseudo-terminal only, no physical devices.
add_test(NAME rescue_a7_restart_wire_tests COMMAND rescue_uart_latency_tests --a7-restart-only)
add_test(NAME rescue_a7_restart_task_tests COMMAND rescue_task_phase_tests --a7-reset)
set_tests_properties(rescue_a7_restart_wire_tests rescue_a7_restart_task_tests
    PROPERTIES TIMEOUT 10 LABELS "integration;task;protocol")
