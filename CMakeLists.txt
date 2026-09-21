cmake_minimum_required(VERSION 3.20)
project(SARX VERSION 0.3.0 LANGUAGES CXX)

option(SARX_BUILD_TESTS "Build SARX tests" ON)

add_library(sarx
    src/body.cpp
    src/damage.cpp
    src/broad_phase.cpp
)

target_include_directories(sarx
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(sarx PUBLIC cxx_std_20)

if (MSVC)
    target_compile_options(sarx PRIVATE /W4 /permissive-)
else()
    target_compile_options(sarx PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (SARX_BUILD_TESTS)
    enable_testing()

    add_executable(sarx_tests
        tests/test_main.cpp
    )
    target_link_libraries(sarx_tests PRIVATE sarx)
    target_compile_features(sarx_tests PRIVATE cxx_std_20)

    add_test(NAME sarx_tests COMMAND sarx_tests)
endif()
