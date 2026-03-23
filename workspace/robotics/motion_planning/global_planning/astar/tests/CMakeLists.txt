add_executable(astar_tests
    test_astar_planner.cpp
)

target_link_libraries(astar_tests PRIVATE
    astar
    common_logging
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(astar_tests)
