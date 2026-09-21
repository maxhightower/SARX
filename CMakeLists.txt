cmake_minimum_required(VERSION 3.20)
project(SARX VERSION 0.4.3 LANGUAGES CXX)

option(SARX_BUILD_TESTS "Build SARX tests" ON)

add_library(sarx
    src/body.cpp
    src/damage.cpp
    src/broad_phase.cpp
    src/volume.cpp
    src/adaptive.cpp
    src/soa.cpp
    src/debug_render.cpp
    src/humanoid.cpp
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

add_executable(sarx_severance_demo
    tools/sarx_severance_demo.cpp
)
target_link_libraries(sarx_severance_demo PRIVATE sarx)
target_compile_features(sarx_severance_demo PRIVATE cxx_std_20)

add_executable(sarx_humanoid_demo
    tools/sarx_humanoid_demo.cpp
)
target_link_libraries(sarx_humanoid_demo PRIVATE sarx)
target_compile_features(sarx_humanoid_demo PRIVATE cxx_std_20)

if (SARX_BUILD_TESTS)
    enable_testing()

    add_executable(sarx_tests
        tests/test_main.cpp
    )
    target_link_libraries(sarx_tests PRIVATE sarx)
    target_compile_features(sarx_tests PRIVATE cxx_std_20)

    add_test(NAME sarx_tests COMMAND sarx_tests)

    add_test(
        NAME sarx_demo_smoke
        COMMAND sarx_severance_demo
            --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_demo_frames
            --frames 5
            --cut-frame 2
    )

    add_test(
        NAME sarx_humanoid_demo_smoke
        COMMAND sarx_humanoid_demo
            --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_humanoid_frames
            --frames 5
            --cut-frame 2
    )
endif()
