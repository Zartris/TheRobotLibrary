add_executable(common_tests
    test_types.cpp
    test_transform2d.cpp
    test_geometry.cpp
    test_differential_drive.cpp
)

target_link_libraries(common_tests PRIVATE
    common
    common_logging
    Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(common_tests)
