cmake_minimum_required(VERSION 3.20)
project(vda5050 VERSION 0.1.0 LANGUAGES CXX)

if(NOT TARGET common)
    message(FATAL_ERROR "[vda5050] Target 'common' not found. Build as part of the workspace.")
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
    message(FATAL_ERROR "[vda5050] Target 'nlohmann_json::nlohmann_json' not found.\n"
        "Build from the workspace root so deps.cmake is included automatically.")
endif()

file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(vda5050 STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(vda5050 INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[vda5050] No source files yet — INTERFACE target")
endif()

target_include_directories(vda5050 ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(vda5050 ${_vis} common nlohmann_json::nlohmann_json)
target_compile_features(vda5050 ${_vis} cxx_std_20)

add_library(robotlib::vda5050 ALIAS vda5050)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
