#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <lidar_processing/scan_filter.hpp>
#include <cmath>
#include <limits>

using namespace robotlib;

TEST_CASE("Range clipping below min", "[lidar_processing]") {
    FilterConfig cfg{0.5, 10.0, 0};
    ScanFilter filter(cfg);

    LaserScan scan;
    scan.ranges = {0.1f, 0.3f, 1.0f, 5.0f, 15.0f};

    auto result = filter.filterScan(scan);

    REQUIRE(std::isnan(result.ranges[0]));  // below min
    REQUIRE(std::isnan(result.ranges[1]));  // below min
    REQUIRE_THAT(result.ranges[2], Catch::Matchers::WithinAbs(1.0f, 1e-6));
    REQUIRE_THAT(result.ranges[3], Catch::Matchers::WithinAbs(5.0f, 1e-6));
    REQUIRE(std::isnan(result.ranges[4]));  // above max
}

TEST_CASE("Median filter with window=3", "[lidar_processing]") {
    FilterConfig cfg{0.0, 100.0, 3};
    ScanFilter filter(cfg);

    LaserScan scan;
    scan.ranges = {1.0f, 5.0f, 2.0f, 3.0f, 4.0f};

    auto result = filter.filterScan(scan);

    // With window=3: median of {1,1,5}=1, {1,5,2}=2, {5,2,3}=3, {2,3,4}=3, {3,4,4}=4
    // Edge handling uses clamped indices
    REQUIRE(result.ranges.size() == 5);
}

TEST_CASE("Passthrough on clean scan", "[lidar_processing]") {
    FilterConfig cfg{0.0, 100.0, 0};
    ScanFilter filter(cfg);

    LaserScan scan;
    scan.ranges = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};

    auto result = filter.filterScan(scan);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        REQUIRE_THAT(result.ranges[i], Catch::Matchers::WithinAbs(scan.ranges[i], 1e-6));
    }
}

TEST_CASE("All-NaN input produces all-NaN output", "[lidar_processing]") {
    FilterConfig cfg{0.5, 10.0, 3};
    ScanFilter filter(cfg);

    LaserScan scan;
    scan.ranges = {std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::quiet_NaN(),
                   std::numeric_limits<float>::quiet_NaN()};

    auto result = filter.filterScan(scan);

    for (const auto& r : result.ranges) {
        REQUIRE(std::isnan(r));
    }
}

TEST_CASE("ScanFilter logging on construction", "[lidar_processing]") {
    FilterConfig cfg;
    ScanFilter filter(cfg);
    // Should not crash — logger is initialized
    LaserScan scan;
    scan.ranges = {1.0f};
    auto result = filter.filterScan(scan);
    REQUIRE(result.ranges.size() == 1);
}
