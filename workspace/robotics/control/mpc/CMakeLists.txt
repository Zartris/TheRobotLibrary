cmake_minimum_required(VERSION 3.20)
project(mpc VERSION 0.1.0 LANGUAGES CXX C)

if(NOT TARGET common)
    message(FATAL_ERROR "[mpc] Target 'common' not found. Build as part of the workspace.")
endif()

# --- acados detection (optional) ---
# acados must be installed separately. See docs/theory.md for setup instructions.
# If found, the MPC module uses acados for real-time NMPC.
# If not found, falls back to Eigen-based LTV-MPC.
set(MPC_HAS_ACADOS FALSE)
find_package(acados QUIET)
if(acados_FOUND)
    message(STATUS "[mpc] acados found -- NMPC backend enabled")
    set(MPC_HAS_ACADOS TRUE)
else()
    # Try manual detection via ACADOS_DIR environment variable
    if(DEFINED ENV{ACADOS_DIR})
        set(ACADOS_DIR "$ENV{ACADOS_DIR}")
        if(EXISTS "${ACADOS_DIR}/lib/libacados.so" OR EXISTS "${ACADOS_DIR}/lib/libacados.a")
            message(STATUS "[mpc] acados found at ${ACADOS_DIR} -- NMPC backend enabled")
            set(MPC_HAS_ACADOS TRUE)
            set(ACADOS_INCLUDE_DIR "${ACADOS_DIR}/include")
            set(ACADOS_LIB_DIR "${ACADOS_DIR}/lib")
        endif()
    endif()
    if(NOT MPC_HAS_ACADOS)
        message(STATUS "[mpc] acados not found -- using Eigen LTV-MPC fallback")
        message(STATUS "[mpc] To enable acados: set ACADOS_DIR or install acados system-wide")
    endif()
endif()

# --- Sources ---
file(GLOB_RECURSE SOURCES src/*.cpp)

# Include generated acados solver C files if they exist
if(MPC_HAS_ACADOS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/generated")
    file(GLOB GENERATED_SOURCES src/generated/*.c)
    list(APPEND SOURCES ${GENERATED_SOURCES})
endif()

if(SOURCES)
    add_library(mpc STATIC ${SOURCES})
    set(_vis PUBLIC)
else()
    add_library(mpc INTERFACE)
    set(_vis INTERFACE)
    message(STATUS "[mpc] No source files yet -- INTERFACE target")
endif()

target_include_directories(mpc ${_vis}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc ${_vis} common common_logging)
target_compile_features(mpc ${_vis} cxx_std_20)

# --- acados linking ---
if(MPC_HAS_ACADOS)
    target_compile_definitions(mpc ${_vis} ROBOTLIB_HAS_ACADOS)
    if(acados_FOUND)
        target_link_libraries(mpc ${_vis} acados)
    else()
        target_include_directories(mpc ${_vis}
            ${ACADOS_INCLUDE_DIR}
            ${ACADOS_INCLUDE_DIR}/blasfeo/include
            ${ACADOS_INCLUDE_DIR}/hpipm/include)
        target_link_directories(mpc ${_vis} ${ACADOS_LIB_DIR})
        target_link_libraries(mpc ${_vis} acados hpipm blasfeo m)
    endif()
    # Include generated solver headers
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/generated")
        target_include_directories(mpc ${_vis}
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>)
    endif()
endif()

add_library(robotlib::mpc ALIAS mpc)

if(BUILD_TESTING AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
    add_subdirectory(tests)
endif()
