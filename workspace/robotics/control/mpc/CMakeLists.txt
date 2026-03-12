cmake_minimum_required(VERSION 3.20)
project(mpc VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[mpc] Target 'common' not found. Build as part of the workspace.")
endif()

# OSQP: fetch standalone if not already available
if(NOT TARGET osqp::osqp)
    include(FetchContent)
    FetchContent_Declare(osqp
        GIT_REPOSITORY https://github.com/osqp/osqp.git
        GIT_TAG        v0.6.3
        GIT_SHALLOW    TRUE
    )
    set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(osqp)
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(mpc STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(mpc INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[mpc] No source files yet — INTERFACE target")
endif()

target_include_directories(mpc ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# common brings Eigen transitively; osqp for the QP solver
target_link_libraries(mpc ${_vis} common osqp::osqp)
target_compile_features(mpc ${_vis} cxx_std_20)

add_library(robotlib::mpc ALIAS mpc)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
