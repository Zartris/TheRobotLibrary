cmake_minimum_required(VERSION 3.20)
project(common VERSION 0.1.0 LANGUAGES CXX)

# ── Standalone build: fetch Eigen if not already available ──────────────────
if(NOT TARGET Eigen3::Eigen)
    include(FetchContent)
    FetchContent_Declare(Eigen3
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG        3.4.0
        GIT_SHALLOW    TRUE
    )
    set(EIGEN_BUILD_DOC       OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
    set(EIGEN_BUILD_TESTING   OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(Eigen3)
endif()

# ── Library target ───────────────────────────────────────────────────────────
# Switches between STATIC (when sources exist) and INTERFACE (scaffold stage).
file(GLOB_RECURSE SOURCES src/*.cpp)

if(SOURCES)
    add_library(common STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(common INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[common] No source files yet — INTERFACE target")
endif()

target_include_directories(common ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Eigen is PUBLIC so every consumer of common automatically gets Eigen headers
target_link_libraries(common ${_vis} Eigen3::Eigen)
target_compile_features(common ${_vis} cxx_std_20)

add_library(robotlib::common ALIAS common)

# ── Sub-modules ──────────────────────────────────────────────────────────────
add_subdirectory(logging)
add_subdirectory(transforms)
add_subdirectory(noise_models)

# ── Tests ────────────────────────────────────────────────────────────────────
if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
