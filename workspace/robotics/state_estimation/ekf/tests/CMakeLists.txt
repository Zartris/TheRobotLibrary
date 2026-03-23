add_executable(ekf_tests
    test_ekf2d.cpp
)

target_link_libraries(ekf_tests PRIVATE
    ekf
    common_logging
    common_testing
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(ekf_tests)
