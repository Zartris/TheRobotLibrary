include(CTest)

add_executable(feature_extraction_tests
    feature_extraction_tests.cpp
)
target_link_libraries(feature_extraction_tests
    PRIVATE robotlib::feature_extraction Catch2::Catch2WithMain
)

include(Catch)
catch_discover_tests(feature_extraction_tests)
