cmake_minimum_required(VERSION 3.16)
project(ImpulseEngine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(engine_core INTERFACE)
target_include_directories(engine_core INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(engine_core INTERFACE cxx_std_17)

enable_testing()

add_executable(impulse_tests
    tests/test_physics.cpp
)
target_link_libraries(impulse_tests PRIVATE engine_core)

if(MSVC)
    target_compile_options(impulse_tests PRIVATE /W3)
else()
    target_compile_options(impulse_tests PRIVATE -Wall -Wextra)
endif()

add_test(NAME unit COMMAND impulse_tests --unit)
set_tests_properties(unit PROPERTIES LABELS "unit")

add_test(NAME scenario_all COMMAND impulse_tests --scenario)
set_tests_properties(scenario_all PROPERTIES LABELS "scenario")

add_test(NAME stress_all COMMAND impulse_tests --stress)
set_tests_properties(stress_all PROPERTIES LABELS "stress")

add_test(NAME bench_phase COMMAND impulse_tests --phase)
set_tests_properties(bench_phase PROPERTIES LABELS "benchmark")