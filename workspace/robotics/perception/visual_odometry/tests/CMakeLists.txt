include(CTest)

add_executable(visual_odometry_tests
    visual_odometry_tests.cpp
)
target_link_libraries(visual_odometry_tests
    PRIVATE robotlib::visual_odometry Catch2::Catch2WithMain
)

include(Catch2/extras/Catch.cmake OPTIONAL)
catch_discover_tests(visual_odometry_tests)
