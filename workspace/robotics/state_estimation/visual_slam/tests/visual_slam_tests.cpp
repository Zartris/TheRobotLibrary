/// @file visual_slam_tests.cpp
/// @brief Catch2 v3 unit tests for the visual_slam module.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <visual_slam/visual_slam.hpp>

#include <array>
#include <cstdint>
#include <random>
#include <vector>

using namespace visual_slam;

// ─────────────────────────────────────────────────────────────────────────────
// Test helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Build a minimal valid camera model (VGA, 60-degree horizontal FoV).
static CameraIntrinsics makeCamera() {
    CameraIntrinsics cam;
    cam.fx     = 525.0;
    cam.fy     = 525.0;
    cam.cx     = 320.0;
    cam.cy     = 240.0;
    cam.width  = 640;
    cam.height = 480;
    return cam;
}

/// Generate `count` synthetic FeatureObservations with random pixels and
/// random binary descriptors, using a seeded PRNG for reproducibility.
static VisualSlam::FrameInput makeSyntheticFrame(
    int          count,
    double       timestamp   = 0.0,
    uint32_t     seed        = 42u,
    float        depthValue  = 0.f)
{
    std::mt19937                     rng{seed};
    std::uniform_real_distribution<float> xDist{10.f, 630.f};
    std::uniform_real_distribution<float> yDist{10.f, 470.f};
    std::uniform_int_distribution<int>    byteDist{0, 255};

    VisualSlam::FrameInput frame;
    frame.timestamp = timestamp;
    frame.features.reserve(static_cast<std::size_t>(count));

    for (int i = 0; i < count; ++i) {
        VisualSlam::FeatureObservation feat;
        feat.x     = xDist(rng);
        feat.y     = yDist(rng);
        feat.depth = depthValue;
        for (auto& b : feat.descriptor) {
            b = static_cast<uint8_t>(byteDist(rng));
        }
        frame.features.push_back(feat);
    }
    return frame;
}

/// Make a frame that shares the first `shared` descriptors with a reference
/// frame (simulating tracked features from the previous frame).
static VisualSlam::FrameInput makeOverlappingFrame(
    const VisualSlam::FrameInput& reference,
    int                           shared,
    int                           newFeatures,
    double                        timestamp,
    uint32_t                      seed = 99u)
{
    VisualSlam::FrameInput frame;
    frame.timestamp = timestamp;

    // Copy first `shared` features from the reference frame (slightly jittered).
    std::mt19937                      rng{seed};
    std::uniform_real_distribution<float> jitter{-1.f, 1.f};

    const int copyCount = std::min(shared, static_cast<int>(reference.features.size()));
    for (int i = 0; i < copyCount; ++i) {
        VisualSlam::FeatureObservation feat = reference.features[static_cast<std::size_t>(i)];
        feat.x += jitter(rng);
        feat.y += jitter(rng);
        frame.features.push_back(feat);
    }

    // Append brand-new random features.
    const VisualSlam::FrameInput extra = makeSyntheticFrame(newFeatures, timestamp, seed + 1u);
    for (const auto& f : extra.features) {
        frame.features.push_back(f);
    }
    return frame;
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

TEST_CASE("VisualSlam construction", "[visual_slam]") {
    const VisualSlam slam{makeCamera()};

    REQUIRE_FALSE(slam.isInitialized());
    REQUIRE(slam.landmarks().empty());
    REQUIRE(slam.keyFrames().empty());
    REQUIRE_FALSE(slam.currentPose().has_value());
    REQUIRE(slam.camera().fx == Catch::Approx(525.0));
    REQUIRE(slam.config().minInitFeatures == 100);
}

TEST_CASE("VisualSlam empty frame returns Lost status", "[visual_slam]") {
    VisualSlam slam{makeCamera()};

    VisualSlam::FrameInput emptyFrame;
    emptyFrame.timestamp = 0.0;
    // No features added.

    const auto result = slam.processFrame(emptyFrame);
    REQUIRE(result.has_value());
    CHECK(result->status == TrackingResult::Status::Lost);
    CHECK(result->trackedLandmarks == 0);
    CHECK_FALSE(slam.isInitialized());
}

TEST_CASE("VisualSlam initialization with enough features", "[visual_slam]") {
    VisualSlam slam{makeCamera()};

    // Feed a frame with 150 features — above the default minInitFeatures of 100.
    const auto frame  = makeSyntheticFrame(150, 0.0);
    const auto result = slam.processFrame(frame);

    REQUIRE(result.has_value());
    CHECK(result->status == TrackingResult::Status::Tracking);
    CHECK(result->trackedLandmarks == 150);
    CHECK(result->isKeyFrame);  // First frame must always be a keyframe

    // Map should now be bootstrapped.
    CHECK(slam.isInitialized());
    CHECK(slam.landmarks().size() == 150);
    CHECK_FALSE(slam.keyFrames().empty());
    CHECK(slam.currentPose().has_value());
}

TEST_CASE("VisualSlam tracks multiple frames", "[visual_slam]") {
    VisualSlamConfig cfg;
    cfg.minInitFeatures    = 50;
    cfg.minTrackedFeatures = 20;
    cfg.keyFrameInterval   = 3;

    VisualSlam slam{makeCamera(), cfg};

    // Frame 0 — initialise.
    const auto frame0  = makeSyntheticFrame(80, 0.0, 1u);
    const auto result0 = slam.processFrame(frame0);
    REQUIRE(result0.has_value());
    REQUIRE(result0->status == TrackingResult::Status::Tracking);

    // Frames 1–4 — tracking with overlapping features.
    for (int i = 1; i <= 4; ++i) {
        const auto frameN  = makeOverlappingFrame(frame0, 60, 20,
                                                  static_cast<double>(i),
                                                  static_cast<uint32_t>(100 + i));
        const auto resultN = slam.processFrame(frameN);
        REQUIRE(resultN.has_value());
        CHECK(resultN->status == TrackingResult::Status::Tracking);
    }

    // After 5 frames the system should still be initialised with keyframes.
    CHECK(slam.isInitialized());
    CHECK_FALSE(slam.keyFrames().empty());
}

TEST_CASE("VisualSlam reset clears state", "[visual_slam]") {
    VisualSlam slam{makeCamera()};

    // Initialise the map.
    const auto frame = makeSyntheticFrame(150, 0.0);
    slam.processFrame(frame);
    REQUIRE(slam.isInitialized());
    REQUIRE_FALSE(slam.landmarks().empty());

    // Reset.
    slam.reset();

    CHECK_FALSE(slam.isInitialized());
    CHECK(slam.landmarks().empty());
    CHECK(slam.keyFrames().empty());
    CHECK_FALSE(slam.currentPose().has_value());
}

TEST_CASE("VisualSlam insufficient features for initialisation", "[visual_slam]") {
    VisualSlam slam{makeCamera()};  // default minInitFeatures = 100

    // Feed only 5 features — well below the threshold.
    const auto frame  = makeSyntheticFrame(5, 0.0);
    const auto result = slam.processFrame(frame);

    REQUIRE(result.has_value());
    CHECK(result->status == TrackingResult::Status::Lost);
    CHECK_FALSE(slam.isInitialized());
    CHECK(slam.landmarks().empty());
}

TEST_CASE("VisualSlam custom config respected", "[visual_slam]") {
    VisualSlamConfig cfg;
    cfg.minInitFeatures    = 10;
    cfg.minTrackedFeatures = 5;
    cfg.enableLoopClosure  = false;

    VisualSlam slam{makeCamera(), cfg};

    // 15 features should be enough to initialise with the custom config.
    const auto frame  = makeSyntheticFrame(15, 1.0);
    const auto result = slam.processFrame(frame);

    REQUIRE(result.has_value());
    CHECK(result->status == TrackingResult::Status::Tracking);
    CHECK(slam.isInitialized());
    CHECK(slam.config().enableLoopClosure == false);
}

TEST_CASE("VisualSlam camera accessor returns correct intrinsics", "[visual_slam]") {
    CameraIntrinsics cam = makeCamera();
    cam.fx = 600.0;
    cam.fy = 601.0;
    cam.cx = 319.5;
    cam.cy = 239.5;

    const VisualSlam slam{cam};

    CHECK(slam.camera().fx == Catch::Approx(600.0));
    CHECK(slam.camera().fy == Catch::Approx(601.0));
    CHECK(slam.camera().cx == Catch::Approx(319.5));
    CHECK(slam.camera().cy == Catch::Approx(239.5));
}

TEST_CASE("VisualSlam reinitialise after reset", "[visual_slam]") {
    VisualSlam slam{makeCamera()};

    slam.processFrame(makeSyntheticFrame(150, 0.0));
    REQUIRE(slam.isInitialized());

    slam.reset();
    CHECK_FALSE(slam.isInitialized());

    // Reinitialise in a second session.
    const auto result = slam.processFrame(makeSyntheticFrame(120, 1.0, 77u));
    REQUIRE(result.has_value());
    CHECK(result->status == TrackingResult::Status::Tracking);
    CHECK(slam.isInitialized());
    CHECK(slam.landmarks().size() == 120);
}
