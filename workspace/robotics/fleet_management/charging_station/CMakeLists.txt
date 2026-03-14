cmake_minimum_required(VERSION 3.20)
project(charging_station)

add_library(charging_station STATIC
    src/charging_station_manager.cpp
)
add_library(robotlib::charging_station ALIAS charging_station)

target_include_directories(charging_station
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>
)
target_link_libraries(charging_station PUBLIC Eigen3::Eigen common)
target_compile_features(charging_station PUBLIC cxx_std_20)

if(BUILD_TESTING AND EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()
