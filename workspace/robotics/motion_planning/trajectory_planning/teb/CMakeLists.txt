cmake_minimum_required(VERSION 3.20)
project(teb VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[teb] Target 'common' not found. Build as part of the workspace.")
endif()

# ── g2o dependency (sparse nonlinear least-squares optimizer) ────────────────
# g2o requires system packages: libsuitesparse-dev, libcholmod-dev
#   sudo apt install libsuitesparse-dev
# Uncomment the block below when TEB implementation begins:
#
# if(NOT TARGET g2o::core)
#     include(FetchContent)
#     FetchContent_Declare(g2o
#         GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
#         GIT_TAG        20230223_git
#         GIT_SHALLOW    TRUE
#     )
#     set(G2O_BUILD_APPS     OFF CACHE BOOL "" FORCE)
#     set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
#     FetchContent_MakeAvailable(g2o)
# endif()
# ─────────────────────────────────────────────────────────────────────────────

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(teb STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(teb INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[teb] No source files yet — INTERFACE target")
endif()

target_include_directories(teb ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(teb ${_vis}
    common            # Eigen via common
    # g2o::core      # uncomment when sources exist
    # g2o::types_sba
)
target_compile_features(teb ${_vis} cxx_std_20)

add_library(robotlib::teb ALIAS teb)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
