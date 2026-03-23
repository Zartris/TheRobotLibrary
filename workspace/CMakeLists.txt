cmake_minimum_required(VERSION 3.20)
project(TheRobotLibrary VERSION 0.1.0 LANGUAGES C CXX)

# Several FetchContent dependencies (OSQP v0.6.3, qhull via MuJoCo) use
# cmake_minimum_required(VERSION < 3.5) which CMake 4.x rejects by default.
# This global override lets CMake 4.x configure them as if they require 3.5.
# See: https://cmake.org/cmake/help/latest/variable/CMAKE_POLICY_VERSION_MINIMUM.html
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

# ---------------------------------------------------------------------------
# Coverage build type (GCC/Clang gcov + lcov)
# ---------------------------------------------------------------------------
option(ENABLE_COVERAGE "Enable gcov/lcov coverage instrumentation" OFF)
if(ENABLE_COVERAGE)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(FATAL_ERROR "Coverage requires GCC or Clang")
  endif()
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

# ── C++ standard ────────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# ── Position-independent code (needed when linking static libs with shared) ──
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# ── Compile database for clangd / IDE tooling ───────────────────────────────
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Default build type ───────────────────────────────────────────────────────
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
        Debug Release RelWithDebInfo MinSizeRel)
endif()

# ── Testing ──────────────────────────────────────────────────────────────────
option(BUILD_TESTING "Build unit tests" ON)
include(CTest)

# ── Third-party dependencies (fetched once, shared by all sub-directories) ──
include(cmake/deps.cmake)

# ── Sub-directories ──────────────────────────────────────────────────────────
add_subdirectory(robotics)
add_subdirectory(simulation)

# ---------------------------------------------------------------------------
# Coverage report target
# ---------------------------------------------------------------------------
if(ENABLE_COVERAGE)
  find_program(LCOV_PATH lcov REQUIRED)
  find_program(GENHTML_PATH genhtml REQUIRED)

  add_custom_target(coverage
    COMMENT "Generating HTML coverage report"
    COMMAND ${LCOV_PATH} --directory ${CMAKE_BINARY_DIR}
            --capture --output-file ${CMAKE_BINARY_DIR}/coverage.info
            --gcov-tool gcov-13
            --ignore-errors mismatch
    COMMAND ${LCOV_PATH} --remove ${CMAKE_BINARY_DIR}/coverage.info
            '*/_deps/*' '/usr/*' '*/tests/*'
            --output-file ${CMAKE_BINARY_DIR}/coverage_filtered.info
            --ignore-errors unused
    COMMAND ${GENHTML_PATH} ${CMAKE_BINARY_DIR}/coverage_filtered.info
            --output-directory ${CMAKE_BINARY_DIR}/coverage_html
            --title "TheRobotLibrary Coverage"
            --legend --show-details
    COMMAND ${CMAKE_COMMAND} -E echo
            "Coverage report: ${CMAKE_BINARY_DIR}/coverage_html/index.html"
  )
endif()
