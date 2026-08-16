cmake_minimum_required(VERSION 3.16)

project(vehicle_host_tests C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(vehicle_host_tests
    test_vehicle.c
    ../code/vehicle_calibration.cpp
    ../code/vehicle_control.cpp
    ../code/vehicle_diagnostics.cpp
    ../code/vehicle_encoder.cpp
    ../code/vehicle_fault.cpp
    ../code/vehicle_localization.cpp
    ../code/vehicle_math.cpp
    ../code/vehicle_mt6701.cpp
    ../code/vehicle_path.cpp
    ../code/vehicle_reverse_tracker.cpp
    ../code/vehicle_safety.cpp
    ../code/vehicle_state_machine.cpp
)

set_source_files_properties(test_vehicle.c vision_tag_tracker_host_test.c
    PROPERTIES LANGUAGE CXX)

target_include_directories(vehicle_host_tests PRIVATE ../code)

if(MSVC)
    target_compile_options(vehicle_host_tests PRIVATE /W4)
else()
    target_compile_options(vehicle_host_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(vehicle_host_tests PRIVATE m)
endif()

enable_testing()
add_test(NAME vehicle_host_tests COMMAND vehicle_host_tests)

add_executable(vision_tag_tracker_host_tests
    vision_tag_tracker_host_test.c
    ../code/vision_tag_tracker.cpp
)

target_include_directories(vision_tag_tracker_host_tests PRIVATE ../code)

if(MSVC)
    target_compile_options(vision_tag_tracker_host_tests PRIVATE /W4)
else()
    target_compile_options(vision_tag_tracker_host_tests PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_test(NAME vision_tag_tracker_host_tests COMMAND vision_tag_tracker_host_tests)
