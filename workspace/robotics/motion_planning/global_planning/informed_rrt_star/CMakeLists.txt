cmake_minimum_required(VERSION 3.20)
project(informed_rrt_star)

add_library(informed_rrt_star STATIC
    src/informed_rrt_star_planner.cpp
)
add_library(robotlib::informed_rrt_star ALIAS informed_rrt_star)

target_include_directories(informed_rrt_star
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(informed_rrt_star PUBLIC Eigen3::Eigen common)
target_compile_features(informed_rrt_star PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
