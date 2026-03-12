cmake_minimum_required(VERSION 3.20)
project(native_frontend VERSION 0.1.0 LANGUAGES CXX)

# ── Third-party dependency checks ────────────────────────────────────────────
# In-tree builds: workspace/cmake/deps.cmake has already fetched all of these.
# For standalone builds, run cmake from the workspace root.
foreach(_dep IN ITEMS "imgui::imgui" "httplib::httplib" "ixwebsocket::ixwebsocket")
    if(NOT TARGET ${_dep})
        message(FATAL_ERROR
            "[native_frontend] Target '${_dep}' not found.\n"
            "Build from the workspace root so deps.cmake is included automatically.")
    endif()
endforeach()
# ─────────────────────────────────────────────────────────────────────────────

find_package(OpenGL REQUIRED)

file(GLOB_RECURSE SOURCES src/*.cpp)

if(NOT SOURCES)
    message(STATUS "[native_frontend] No source files yet — skipping executable target")
    return()
endif()

add_executable(native_frontend ${SOURCES})

target_include_directories(native_frontend PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(native_frontend PRIVATE
    imgui::imgui              # immediate-mode GUI (SDL2 + OpenGL3 backend)
    OpenGL::GL
    httplib::httplib          # REST calls to the simulation server
    ixwebsocket::ixwebsocket  # live telemetry stream from the simulation server
)

target_compile_features(native_frontend PRIVATE cxx_std_20)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
