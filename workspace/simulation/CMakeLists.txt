cmake_minimum_required(VERSION 3.20)
project(simulation VERSION 0.1.0 LANGUAGES CXX)

# -- Third-party dependencies -------------------------------------------------
# In-tree builds: workspace/cmake/deps.cmake already declared these.
# Standalone: build from workspace root so deps.cmake is included automatically.
# NOTE: MuJoCo target may be 'mujoco' (FetchContent) or 'mujoco::mujoco'
# (find_package). Verify during M0 validation and update accordingly.
if(NOT TARGET mujoco)
    message(FATAL_ERROR
        "[simulation] Target 'mujoco' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()
if(NOT TARGET imgui::imgui)
    message(FATAL_ERROR
        "[simulation] Target 'imgui::imgui' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()
# ------------------------------------------------------------------------------

find_package(OpenGL REQUIRED)

# Collect sources — executable grows as features are added
file(GLOB_RECURSE SOURCES src/*.cpp)

if(NOT SOURCES)
    message(STATUS "[simulation] No source files yet — skipping executable target")
    return()
endif()

add_executable(simulation_app ${SOURCES})

target_include_directories(simulation_app PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(simulation_app PRIVATE
    # Physics engine (target name may be 'mujoco' or 'mujoco::mujoco' — verify in M0)
    mujoco

    # GUI overlay (GLFW + OpenGL3 backend, GLFW comes via MuJoCo)
    imgui::imgui
    OpenGL::GL

    # JSON (scenario configs, parameter export, telemetry)
    # nlohmann_json::nlohmann_json

    # Robotics modules consumed by the simulation at runtime.
    # Add as needed when implementing scenarios:
    # robotlib::common
    # robotlib::ekf
    # robotlib::occupancy_grid
    # robotlib::pid
    # robotlib::astar
    # robotlib::dwa
)

target_compile_features(simulation_app PRIVATE cxx_std_20)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
