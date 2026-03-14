cmake_minimum_required(VERSION 3.20)
project(lattice_planner)

add_library(lattice_planner STATIC
    src/lattice_planner.cpp
)
add_library(robotlib::lattice_planner ALIAS lattice_planner)

target_include_directories(lattice_planner
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(lattice_planner PUBLIC Eigen3::Eigen common)
target_compile_features(lattice_planner PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
