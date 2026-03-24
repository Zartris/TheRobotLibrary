add_executable(occupancy_grid_tests
    test_occupancy_grid_map.cpp
    test_inflation.cpp
)

target_link_libraries(occupancy_grid_tests PRIVATE
    occupancy_grid
    common_logging
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(occupancy_grid_tests)
