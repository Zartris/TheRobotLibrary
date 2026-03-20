cmake_minimum_required(VERSION 3.20)
project(common_logging VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[common_logging] Target 'common' not found. Build as part of the workspace.")
endif()

# spdlog is an optional dependency — only required when sources exist.
# During scaffold stage this is an INTERFACE target with no link requirement.
file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(common_logging STATIC ${SOURCES})
    set(_vis PUBLIC)
    # spdlog provided via FetchContent in deps.cmake (workspace builds)
    # or find_package for standalone builds
    if(NOT TARGET spdlog::spdlog)
        find_package(spdlog REQUIRED)
    endif()
    target_link_libraries(common_logging PUBLIC common spdlog::spdlog)
else()
    add_library(common_logging INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[common_logging] No source files yet — INTERFACE target")
    target_link_libraries(common_logging INTERFACE common)
endif()

target_include_directories(common_logging ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(common_logging ${_vis} cxx_std_20)

add_library(robotlib::common_logging ALIAS common_logging)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
