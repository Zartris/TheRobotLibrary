# workspace/cmake/deps.cmake
#
# Central FetchContent declarations for all third-party dependencies.
# Every declaration is guarded so this file is safe to include() more than once
# from nested sub-directories.
#
# Usage (in-tree build):
#   The workspace root CMakeLists.txt calls include(cmake/deps.cmake) once.
#   All targets declared here are then visible to every subdirectory.
#
# Usage (standalone module build):
#   Copy the relevant block(s) into your module's own CMakeLists.txt,
#   wrapping each in:  if(NOT TARGET <alias>)  ...  endif()

include(FetchContent)

# ---------------------------------------------------------------------------
# Eigen 3.4.0 — header-only linear algebra
# Used by: common (re-exported), ekf, particle_filter, ekf_slam, lidar_slam,
#          mpc, dmpc, spline_fitting, teb, time_optimal, mader
# ---------------------------------------------------------------------------
if(NOT TARGET Eigen3::Eigen)
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

# ---------------------------------------------------------------------------
# Catch2 v3.5.2 — unit testing framework (all modules' tests/)
# ---------------------------------------------------------------------------
if(NOT TARGET Catch2::Catch2)
    FetchContent_Declare(Catch2
        GIT_REPOSITORY https://github.com/catchorg/Catch2.git
        GIT_TAG        v3.5.2
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(Catch2)
    # Make Catch2's cmake modules available for catch_discover_tests()
    list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
endif()

# ---------------------------------------------------------------------------
# nlohmann/json v3.11.3 — header-only JSON (simulation, REST payloads)
# ---------------------------------------------------------------------------
if(NOT TARGET nlohmann_json::nlohmann_json)
    FetchContent_Declare(nlohmann_json
        GIT_REPOSITORY https://github.com/nlohmann/json.git
        GIT_TAG        v3.11.3
        GIT_SHALLOW    TRUE
    )
    set(JSON_BuildTests     OFF CACHE BOOL "" FORCE)
    set(JSON_Install        OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# ---------------------------------------------------------------------------
# Crow v1.2 — C++ HTTP + WebSocket server (simulation backend)
# Crow internally fetches standalone Asio if not found on the system.
# ---------------------------------------------------------------------------
if(NOT TARGET Crow::Crow)
    FetchContent_Declare(Crow
        GIT_REPOSITORY https://github.com/CrowCpp/Crow.git
        GIT_TAG        v1.2
        GIT_SHALLOW    TRUE
    )
    set(CROW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(CROW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(Crow)
endif()

# ---------------------------------------------------------------------------
# OSQP v0.6.3 — Embedded QP solver (multi_robot/dmpc, general QP needs)
# Note: MPC uses acados (see below) for NMPC. OSQP retained for simpler QP
# problems (DMPC per-robot subproblems, trajectory optimization QPs).
# ---------------------------------------------------------------------------
if(NOT TARGET osqp::osqp)
    FetchContent_Declare(osqp
        GIT_REPOSITORY https://github.com/osqp/osqp.git
        GIT_TAG        v0.6.3
        GIT_SHALLOW    TRUE
    )
    set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(osqp)
endif()

# ---------------------------------------------------------------------------
# cpp-httplib v0.15.3 — header-only HTTP/HTTPS client (frontends/native)
# ---------------------------------------------------------------------------
if(NOT TARGET httplib::httplib)
    FetchContent_Declare(cpp-httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG        v0.15.3
        GIT_SHALLOW    TRUE
    )
    set(HTTPLIB_COMPILE OFF CACHE BOOL "" FORCE)   # stay header-only
    FetchContent_MakeAvailable(cpp-httplib)
endif()

# ---------------------------------------------------------------------------
# IXWebSocket v11.4.4 — WebSocket client (frontends/native)
# ---------------------------------------------------------------------------
if(NOT TARGET ixwebsocket::ixwebsocket)
    FetchContent_Declare(IXWebSocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
        GIT_TAG        v11.4.4
        GIT_SHALLOW    TRUE
    )
    set(USE_TLS OFF CACHE BOOL "" FORCE)   # set ON to enable SSL support
    FetchContent_MakeAvailable(IXWebSocket)
endif()

# ---------------------------------------------------------------------------
# SDL2 release-2.30.0 — window / input / OpenGL context (frontends/native)
# ---------------------------------------------------------------------------
if(NOT TARGET SDL2::SDL2)
    FetchContent_Declare(SDL2
        GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
        GIT_TAG        release-2.30.0
        GIT_SHALLOW    TRUE
    )
    set(SDL_STATIC  ON  CACHE BOOL "" FORCE)
    set(SDL_SHARED  OFF CACHE BOOL "" FORCE)
    set(SDL_TEST    OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(SDL2)
endif()

# ---------------------------------------------------------------------------
# Dear ImGui v1.90.4 — immediate-mode GUI (frontends/native)
# ImGui has no official CMakeLists; we create a STATIC target manually.
# ---------------------------------------------------------------------------
if(NOT TARGET imgui::imgui)
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.90.4
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(imgui)

    find_package(OpenGL REQUIRED)

    add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        # SDL2 + OpenGL3 back-end
        ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui PUBLIC SDL2::SDL2 OpenGL::GL)
    add_library(imgui::imgui ALIAS imgui)
endif()

# ---------------------------------------------------------------------------
# g2o — sparse nonlinear least-squares optimizer (trajectory_planning/teb)
#
# g2o requires system packages on Linux:
#   sudo apt install libsuitesparse-dev libcholmod-dev
#
# Uncomment this block when beginning TEB implementation:
# ---------------------------------------------------------------------------
# if(NOT TARGET g2o::core)
#     FetchContent_Declare(g2o
#         GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
#         GIT_TAG        20230223_git
#         GIT_SHALLOW    TRUE
#     )
#     set(G2O_BUILD_APPS     OFF CACHE BOOL "" FORCE)
#     set(G2O_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
#     FetchContent_MakeAvailable(g2o)
# endif()

# ---------------------------------------------------------------------------
# acados — real-time NMPC solver (control/mpc)
#
# acados uses CASAdi internally for OCP specification and HPIPM for QP solving.
# The workflow: define OCP in Python (CASAdi + acados API) → generate C solver
# code → link generated code into C++ module.
#
# Requires: Python 3 + pip install acados (for code generation only; runtime is pure C)
# The generated C code is committed to src/generated/ so building does NOT
# require Python.
#
# Uncomment this block when beginning MPC implementation (M3):
# ---------------------------------------------------------------------------
# if(NOT TARGET acados)
#     FetchContent_Declare(acados
#         GIT_REPOSITORY https://github.com/acados/acados.git
#         GIT_TAG        v0.4.1
#         GIT_SHALLOW    TRUE
#     )
#     set(ACADOS_WITH_QPOASES  OFF CACHE BOOL "" FORCE)
#     set(ACADOS_WITH_HPIPM    ON  CACHE BOOL "" FORCE)
#     set(BLASFEO_TARGET       GENERIC CACHE STRING "" FORCE)
#     FetchContent_MakeAvailable(acados)
# endif()
