cmake_minimum_required(VERSION 3.20)
project(simulation VERSION 0.1.0 LANGUAGES CXX)

# ── Third-party dependencies ─────────────────────────────────────────────────
# In-tree builds: workspace/cmake/deps.cmake already declared these.
# Standalone: uncomment and fetch them inline (or run cmake from workspace root).
if(NOT TARGET Crow::Crow)
    message(FATAL_ERROR
        "[simulation] Target 'Crow::Crow' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
    message(FATAL_ERROR
        "[simulation] Target 'nlohmann_json::nlohmann_json' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()
# ─────────────────────────────────────────────────────────────────────────────

# Collect sources — executable grows as features are added
file(GLOB_RECURSE SOURCES src/*.cpp)

if(NOT SOURCES)
    message(STATUS "[simulation] No source files yet — skipping executable target")
    return()
endif()

add_executable(simulation_server ${SOURCES})

target_include_directories(simulation_server PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(simulation_server PRIVATE
    # HTTP/WebSocket server
    Crow::Crow
    nlohmann_json::nlohmann_json

    # Robotics modules consumed by the simulation at runtime.
    # Add as needed when implementing scenarios:
    # robotlib::common
    # robotlib::ekf
    # robotlib::occupancy_grid
    # robotlib::pid
    # robotlib::astar
    # robotlib::dwa
)

target_compile_features(simulation_server PRIVATE cxx_std_20)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
