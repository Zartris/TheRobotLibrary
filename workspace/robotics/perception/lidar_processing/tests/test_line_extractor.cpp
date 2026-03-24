#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <lidar_processing/line_extractor.hpp>
#include <testing/recording_logger.hpp>
#include <logging/get_logger.hpp>
#include <cmath>
#include <numbers>

using namespace robotlib;

// Helper: create a scan simulating a straight wall
static LaserScan makeWallScan(double wallDist, double wallAngleStart, double wallAngleEnd,
                                int totalRays = 360) {
    LaserScan scan;
    scan.angleMin = -std::numbers::pi;
    scan.angleMax = std::numbers::pi;
    scan.angleIncrement = (scan.angleMax - scan.angleMin) / (totalRays - 1);
    scan.ranges.resize(totalRays, 50.0f);  // default far away

    for (int i = 0; i < totalRays; ++i) {
        double angle = scan.angleMin + i * scan.angleIncrement;
        if (angle >= wallAngleStart && angle <= wallAngleEnd) {
            // Wall at fixed distance perpendicular to angle
            scan.ranges[i] = static_cast<float>(wallDist / std::cos(angle - (wallAngleStart + wallAngleEnd) / 2.0));
            if (scan.ranges[i] < 0.1f || scan.ranges[i] > 10.0f) {
                scan.ranges[i] = 50.0f;
            }
        }
    }
    return scan;
}

TEST_CASE("Straight wall -> single line", "[lidar_processing][ransac]") {
    LineExtractorConfig cfg;
    cfg.distanceThreshold = 0.1;
    cfg.minInliers = 5;
    LineExtractor extractor(cfg);

    // Wall ahead at ~2m, spanning angles -0.3 to 0.3
    auto scan = makeWallScan(2.0, -0.3, 0.3);
    auto lines = extractor.extractLines(scan, {0, 0, 0});

    REQUIRE(lines.size() >= 1);
    CHECK(lines[0].inlierCount >= 5);
}

TEST_CASE("L-shaped wall -> two lines", "[lidar_processing][ransac]") {
    LineExtractorConfig cfg;
    cfg.distanceThreshold = 0.1;
    cfg.minInliers = 5;
    cfg.maxIterations = 200;
    LineExtractor extractor(cfg);

    // Two walls at different angles
    LaserScan scan;
    scan.angleMin = -std::numbers::pi;
    scan.angleMax = std::numbers::pi;
    int numRays = 360;
    scan.angleIncrement = (scan.angleMax - scan.angleMin) / (numRays - 1);
    scan.ranges.resize(numRays, 50.0f);

    // Wall 1: ahead at 2m (angles -0.2 to 0.2)
    for (int i = 170; i <= 190; ++i) scan.ranges[i] = 2.0f;
    // Wall 2: to the left at 1.5m (angles 1.3 to 1.7)
    for (int i = 250; i <= 270; ++i) scan.ranges[i] = 1.5f;

    auto lines = extractor.extractLines(scan, {0, 0, 0});
    REQUIRE(lines.size() >= 2);
}

TEST_CASE("No structure -> empty result", "[lidar_processing][ransac]") {
    LineExtractorConfig cfg;
    cfg.minInliers = 10;
    LineExtractor extractor(cfg);

    // All ranges at max (no walls nearby)
    LaserScan scan;
    scan.angleMin = -std::numbers::pi;
    scan.angleMax = std::numbers::pi;
    scan.angleIncrement = 0.1;
    scan.ranges.resize(63, 50.0f);  // all far

    auto lines = extractor.extractLines(scan, {0, 0, 0});
    REQUIRE(lines.empty());
}

TEST_CASE("LineExtractor logging and observability", "[lidar_processing][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("lidar_processing.line_extractor", mockLogger);

    LineExtractor extractor;

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));

    LaserScan scan;
    scan.angleMin = -1.0;
    scan.angleMax = 1.0;
    scan.angleIncrement = 0.1;
    scan.ranges.resize(20, 2.0f);
    extractor.extractLines(scan, {0, 0, 0});

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
    REQUIRE(mockLogger->hasNoErrors());
}
