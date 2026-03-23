add_executable(cbf_tests test_cbf_safety_filter.cpp)
target_link_libraries(cbf_tests PRIVATE
    cbf
    pid
    common_testing
    Catch2::Catch2WithMain
)
include(Catch)
catch_discover_tests(cbf_tests)
