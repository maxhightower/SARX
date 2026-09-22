cmake_minimum_required(VERSION 3.20)
project(SARX VERSION 0.5.0 LANGUAGES CXX)

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
    src/gltf_character.cpp
    src/character_render.cpp
    src/anatomy.cpp
    src/anatomy_render.cpp
)

target_include_directories(sarx
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_compile_features(sarx PUBLIC cxx_std_20)

if (MSVC)
    target_compile_options(sarx PRIVATE /W4 /permissive-)
else()
    target_compile_options(sarx PRIVATE -Wall -Wextra -Wpedantic)
    # The anatomy solver never inspects errno; this lets sqrt/cbrt inline.
    set_source_files_properties(src/anatomy.cpp PROPERTIES COMPILE_OPTIONS -fno-math-errno)
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

add_executable(sarx_character_demo
    tools/sarx_character_demo.cpp
)
target_link_libraries(sarx_character_demo PRIVATE sarx)
target_compile_features(sarx_character_demo PRIVATE cxx_std_20)

add_executable(sarx_character_severance_demo
    tools/sarx_character_severance_demo.cpp
)
target_link_libraries(sarx_character_severance_demo PRIVATE sarx)
target_compile_features(sarx_character_severance_demo PRIVATE cxx_std_20)

add_executable(sarx_anatomy_demo
    tools/sarx_anatomy_demo.cpp
)
target_link_libraries(sarx_anatomy_demo PRIVATE sarx)
target_compile_features(sarx_anatomy_demo PRIVATE cxx_std_20)

add_executable(sarx_anatomy_bench
    tools/sarx_anatomy_bench.cpp
)
target_link_libraries(sarx_anatomy_bench PRIVATE sarx)
target_compile_features(sarx_anatomy_bench PRIVATE cxx_std_20)

if (SARX_BUILD_TESTS)
    enable_testing()

    add_executable(sarx_tests
        tests/test_main.cpp
    )
    target_link_libraries(sarx_tests PRIVATE sarx)
    target_compile_features(sarx_tests PRIVATE cxx_std_20)

    add_test(NAME sarx_tests COMMAND sarx_tests)

    add_executable(sarx_anatomy_tests
        tests/test_anatomy.cpp
    )
    target_link_libraries(sarx_anatomy_tests PRIVATE sarx)
    target_compile_features(sarx_anatomy_tests PRIVATE cxx_std_20)
    add_test(NAME sarx_anatomy_tests COMMAND sarx_anatomy_tests)

    add_test(
        NAME sarx_anatomy_demo_smoke
        COMMAND sarx_anatomy_demo
            --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_anatomy_frames
            --frames 40
            --every 20
            --width 320
            --height 180
    )

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

    if (
        EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb"
        AND
        EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb"
    )
        add_test(
            NAME sarx_real_character_walk_smoke
            COMMAND sarx_character_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_character_frames
                --frames 3
                --fps 30
        )

        add_test(
            NAME sarx_real_character_severance_smoke
            COMMAND sarx_character_severance_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --detach-root upperarm_r
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_severance_frames
                --frames 5
                --cut-frame 2
                --fps 30
        )

        add_test(
            NAME sarx_real_character_all_clips
            COMMAND sarx_character_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_character_validation
                --frames 1
                --fps 30
                --validate-all-clips
        )

        foreach(SARX_DETACH_ROOT
            upperarm_l
            upperarm_r
            lowerarm_r
            thigh_l
            calf_r
        )
            add_test(
                NAME sarx_real_branch_${SARX_DETACH_ROOT}
                COMMAND sarx_character_severance_demo
                    --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                    --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                    --clip Walk
                    --detach-root ${SARX_DETACH_ROOT}
                    --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_branch_${SARX_DETACH_ROOT}
                    --frames 5
                    --cut-frame 2
                    --fps 30
                    --validate-branch-only
            )
        endforeach()
    endif()
endif()
