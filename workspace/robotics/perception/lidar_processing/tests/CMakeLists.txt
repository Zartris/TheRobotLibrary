add_executable(lidar_processing_tests
    test_scan_filter.cpp
    test_line_extractor.cpp
)

target_link_libraries(lidar_processing_tests PRIVATE
    lidar_processing
    common_logging
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(lidar_processing_tests)
