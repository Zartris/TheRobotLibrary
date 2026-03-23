add_executable(dwa_tests
    test_dwa_planner.cpp
)

target_link_libraries(dwa_tests PRIVATE
    dwa
    common_logging
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(dwa_tests)
