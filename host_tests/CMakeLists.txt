cmake_minimum_required(VERSION 3.16)

project(vehicle_host_tests C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(vehicle_host_tests
    test_vehicle.c
    ../code/vehicle_calibration.c
    ../code/vehicle_control.c
    ../code/vehicle_encoder.c
    ../code/vehicle_fault.c
    ../code/vehicle_localization.c
    ../code/vehicle_math.c
    ../code/vehicle_mt6701.c
    ../code/vehicle_path.c
    ../code/vehicle_reverse_tracker.c
    ../code/vehicle_safety.c
    ../code/vehicle_state_machine.c
)

target_include_directories(vehicle_host_tests PRIVATE ../code)

if(MSVC)
    target_compile_options(vehicle_host_tests PRIVATE /W4)
else()
    target_compile_options(vehicle_host_tests PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(vehicle_host_tests PRIVATE m)
endif()

enable_testing()
add_test(NAME vehicle_host_tests COMMAND vehicle_host_tests)
