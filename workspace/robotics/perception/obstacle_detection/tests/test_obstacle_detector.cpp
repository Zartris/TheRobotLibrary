#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <obstacle_detection/obstacle_detector.hpp>
#include <testing/recording_logger.hpp>
#include <logging/get_logger.hpp>
#include <cmath>
#include <numbers>

using namespace robotlib;

// Helper: create a LaserScan with uniform angular spacing
static LaserScan makeUniformScan(int numRays, double angleMin, double angleMax,
                                  const std::vector<float>& ranges) {
    LaserScan scan;
    scan.angleMin = angleMin;
    scan.angleMax = angleMax;
    scan.angleIncrement = (angleMax - angleMin) / std::max(1, numRays - 1);
    scan.ranges = ranges;
    return scan;
}

TEST_CASE("Cluster 3 distinct groups → 3 obstacles", "[obstacle_detection]") {
    ObstacleDetectorConfig cfg;
    cfg.clusterEps = 0.3;
    cfg.clusterMinPoints = 2;
    ObstacleDetector detector(cfg);

    // Create scan with 3 clusters of points
    // Cluster 1: points at ~1m ahead (angles near 0)
    // Cluster 2: points at ~2m to the left (angles near pi/2)
    // Cluster 3: points at ~1.5m to the right (angles near -pi/2)
    int numRays = 360;
    double angleMin = -std::numbers::pi;
    double angleMax = std::numbers::pi;
    std::vector<float> ranges(numRays, 50.0f);  // far away (no obstacle)

    // Cluster 1: rays near angle 0 (indices ~180) -> range 1.0
    for (int i = 175; i <= 185; ++i) ranges[i] = 1.0f;
    // Cluster 2: rays near angle pi/2 (indices ~270) -> range 2.0
    for (int i = 265; i <= 275; ++i) ranges[i] = 2.0f;
    // Cluster 3: rays near angle -pi/2 (indices ~90) -> range 1.5
    for (int i = 85; i <= 95; ++i) ranges[i] = 1.5f;

    auto scan = makeUniformScan(numRays, angleMin, angleMax, ranges);
    Pose2D pose{0.0, 0.0, 0.0};

    auto obstacles = detector.detect(scan, pose);
    REQUIRE(obstacles.size() == 3);
}

TEST_CASE("Track obstacle across frames -> stable ID + velocity", "[obstacle_detection]") {
    ObstacleDetectorConfig cfg;
    cfg.clusterEps = 0.5;
    cfg.clusterMinPoints = 2;
    cfg.associationThreshold = 1.0;
    ObstacleDetector detector(cfg);

    // Frame 1: obstacle at ~(2, 0)
    int numRays = 36;
    std::vector<float> ranges1(numRays, 50.0f);
    // Rays near angle 0 -> range 2.0
    for (int i = 17; i <= 19; ++i) ranges1[i] = 2.0f;
    auto scan1 = makeUniformScan(numRays, -std::numbers::pi, std::numbers::pi, ranges1);

    auto obs1 = detector.detect(scan1, {0, 0, 0});
    REQUIRE(obs1.size() >= 1);
    int firstId = obs1[0].id;
    REQUIRE(firstId >= 0);

    // Frame 2: obstacle moved slightly to (2.1, 0)
    std::vector<float> ranges2(numRays, 50.0f);
    for (int i = 17; i <= 19; ++i) ranges2[i] = 2.1f;
    auto scan2 = makeUniformScan(numRays, -std::numbers::pi, std::numbers::pi, ranges2);

    auto obs2 = detector.detect(scan2, {0, 0, 0});
    REQUIRE(obs2.size() >= 1);

    // Should maintain same ID
    bool foundSameId = false;
    for (const auto& o : obs2) {
        if (o.id == firstId) {
            foundSameId = true;
            // Velocity should be nonzero (obstacle moved)
            CHECK(o.velocity.norm() > 0.0);
        }
    }
    CHECK(foundSameId);
}

TEST_CASE("Empty scan -> no obstacles", "[obstacle_detection]") {
    ObstacleDetector detector;

    LaserScan scan;
    scan.angleMin = -std::numbers::pi;
    scan.angleMax = std::numbers::pi;
    scan.angleIncrement = 0.01;
    // No ranges -> empty

    auto obstacles = detector.detect(scan, {0, 0, 0});
    REQUIRE(obstacles.empty());
}

TEST_CASE("Single-point clusters filtered by minPoints", "[obstacle_detection]") {
    ObstacleDetectorConfig cfg;
    cfg.clusterMinPoints = 3;
    cfg.clusterEps = 0.2;
    ObstacleDetector detector(cfg);

    // Create scan with isolated single points (noise)
    int numRays = 36;
    std::vector<float> ranges(numRays, 50.0f);
    ranges[5] = 1.0f;   // single point
    ranges[15] = 2.0f;  // single point (far from others)
    ranges[25] = 3.0f;  // single point

    auto scan = makeUniformScan(numRays, -std::numbers::pi, std::numbers::pi, ranges);
    auto obstacles = detector.detect(scan, {0, 0, 0});

    // All points are isolated, none form clusters of 3+
    REQUIRE(obstacles.empty());
}

TEST_CASE("ObstacleDetector logging and observability", "[obstacle_detection][logging]") {
    auto cleanup = std::shared_ptr<void>(nullptr, [](void*) { robotlib::clearLoggerRegistry(); });
    auto mockLogger = std::make_shared<robotlib::testing::RecordingLogger>();
    robotlib::registerLogger("obstacle_detection", mockLogger);

    ObstacleDetector detector;

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::DEBUG, "initialized"));

    // Run detect
    LaserScan scan;
    scan.angleMin = -1.0;
    scan.angleMax = 1.0;
    scan.angleIncrement = 0.1;
    scan.ranges.resize(20, 1.0f);
    detector.detect(scan, {0, 0, 0});

    REQUIRE(mockLogger->hasMessageContaining(
        robotlib::testing::LogEntry::Level::TRACE, "us"));
    REQUIRE(mockLogger->hasNoErrors());
}
