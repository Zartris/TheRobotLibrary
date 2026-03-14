cmake_minimum_required(VERSION 3.20)
project(lqg)

add_library(lqg STATIC
    src/lqg_controller.cpp
)
add_library(robotlib::lqg ALIAS lqg)

target_include_directories(lqg
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(lqg PUBLIC Eigen3::Eigen common lqr ekf)
target_compile_features(lqg PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
