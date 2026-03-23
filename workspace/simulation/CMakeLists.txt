cmake_minimum_required(VERSION 3.20)
project(simulation VERSION 0.1.0 LANGUAGES CXX)

# -- Third-party dependencies -------------------------------------------------
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
list(FILTER SOURCES EXCLUDE REGEX "src/main\\.cpp$")

if(NOT SOURCES)
    message(STATUS "[simulation] No source files yet — skipping executable target")
    return()
endif()

# Core simulation library (shared between app and tests)
add_library(simulation_core STATIC ${SOURCES})

target_include_directories(simulation_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(simulation_core PUBLIC
    mujoco
    common
    common_logging
    ekf
    occupancy_grid
    ray_casting
    lidar_processing
    pid
    astar
    dwa
    velocity_profiling
    pure_pursuit
    mpc
    cbf
)

# GUI/rendering deps only needed for interactive mode (linked via app_shell.cpp)
target_link_libraries(simulation_core PRIVATE
    imgui::imgui
    OpenGL::GL
    glfw
)

target_compile_features(simulation_core PUBLIC cxx_std_20)

# Main executable
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")
    add_executable(simulation_app src/main.cpp)
    target_link_libraries(simulation_app PRIVATE simulation_core imgui::imgui OpenGL::GL glfw)
endif()

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
