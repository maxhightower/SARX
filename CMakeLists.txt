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
    src/gltf_character.cpp
    src/character_render.cpp
    src/voxel_character.cpp
    src/motion_viability.cpp
    src/motion_recovery.cpp
    src/detached_articulation.cpp
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

add_executable(sarx_character_head_severance_demo
    tools/sarx_character_head_severance_demo.cpp
)
target_link_libraries(sarx_character_head_severance_demo PRIVATE sarx)
target_compile_features(sarx_character_head_severance_demo PRIVATE cxx_std_20)

add_executable(sarx_character_voxel_demo
    tools/sarx_character_voxel_demo.cpp
)
target_link_libraries(sarx_character_voxel_demo PRIVATE sarx)
target_compile_features(sarx_character_voxel_demo PRIVATE cxx_std_20)

add_executable(sarx_character_voxel_thigh_demo
    tools/sarx_character_voxel_thigh_demo.cpp
)
target_link_libraries(sarx_character_voxel_thigh_demo PRIVATE sarx)
target_compile_features(sarx_character_voxel_thigh_demo PRIVATE cxx_std_20)

add_executable(sarx_character_voxel_foot_injury_demo
    tools/sarx_character_voxel_foot_injury_demo.cpp
)
target_link_libraries(sarx_character_voxel_foot_injury_demo PRIVATE sarx)
target_compile_features(sarx_character_voxel_foot_injury_demo PRIVATE cxx_std_20)

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
            NAME sarx_real_character_left_arm_severance_smoke
            COMMAND sarx_character_severance_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --detach-root upperarm_l
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_left_arm_severance_frames
                --frames 5
                --cut-frame 2
                --fps 30
        )

        add_test(
            NAME sarx_real_character_head_severance_smoke
            COMMAND sarx_character_head_severance_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_head_severance_frames
                --frames 5
                --cut-frame 2
                --fps 30
        )

        add_test(
            NAME sarx_real_character_voxel_smoke
            COMMAND sarx_character_voxel_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_voxel_frames
                --frames 5
                --damage-frame 1
                --fps 30
                --voxel-size 0.045
                --require-damage
                --require-detachment
                --require-anatomical-isolation
                --require-walk-viability
                --require-stable-camera
        )

        add_test(
            NAME sarx_real_character_voxel_thigh_smoke
            COMMAND sarx_character_voxel_thigh_demo
                --character ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/character.glb
                --animations ${CMAKE_CURRENT_SOURCE_DIR}/assets/quaternius/animations.glb
                --clip Walk
                --output ${CMAKE_CURRENT_BINARY_DIR}/sarx_real_voxel_thigh_frames
                --frames 6
                --cut-frame 1
                --fps 30
                --voxel-size 0.045
                --require-damage
                --require-detachment
                --require-walk-invalidation
                --require-anatomical-isolation
                --require-stable-camera
                --require-articulated-leg
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
            Head
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
