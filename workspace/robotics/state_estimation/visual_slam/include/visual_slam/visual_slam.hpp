#pragma once

/// @file visual_slam.hpp
/// @brief Feature-based Visual SLAM public API.
///
/// Tracks visual features across camera frames (front-end visual odometry),
/// builds a 3-D landmark map, and detects loop closures to correct accumulated
/// drift.  This module is the capstone of the visual perception pipeline.
///
/// Architecture note: this module depends only on `common` (which transitively
/// provides Eigen).  It deliberately does NOT depend on `visual_odometry` or
/// any other robotics module, keeping all tiers cleanly separated.
///
/// For the mathematical background see docs/theory.md.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <expected>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace visual_slam {

// ─────────────────────────────────────────────────────────────────────────────
// Camera model
// ─────────────────────────────────────────────────────────────────────────────

/// Pinhole camera intrinsic parameters.
/// A self-contained copy — intentionally not re-using visual_odometry types so
/// this module remains a leaf with no sibling dependencies.
struct CameraIntrinsics {
    double fx{0.0};   ///< Focal length in pixels, x-axis
    double fy{0.0};   ///< Focal length in pixels, y-axis
    double cx{0.0};   ///< Principal point x (pixels)
    double cy{0.0};   ///< Principal point y (pixels)
    int    width{0};  ///< Image width in pixels
    int    height{0}; ///< Image height in pixels
};

// ─────────────────────────────────────────────────────────────────────────────
// Map primitives
// ─────────────────────────────────────────────────────────────────────────────

/// A single 2-D observation of a 3-D landmark in one camera frame.
struct Observation {
    uint64_t               landmarkId{0};    ///< ID of the observed 3-D landmark
    Eigen::Vector2d        pixel;            ///< Pixel coordinates (u, v)
    std::array<uint8_t, 32> descriptor{};   ///< Binary descriptor (e.g. ORB)
};

/// A 3-D point maintained in the persistent map.
struct Landmark {
    uint64_t               id{0};
    Eigen::Vector3d        position;               ///< World-frame 3-D position
    std::array<uint8_t, 32> descriptor{};          ///< Representative descriptor (running mean)
    int                    observationCount{0};    ///< Total times this landmark was observed
    bool                   isActive{true};         ///< False when culled from the map
};

/// A keyframe: a camera frame selected for long-term retention in the map.
/// Keyframes anchor the pose graph and are the basis for loop closure queries.
struct KeyFrame {
    uint64_t                id{0};
    Eigen::Isometry3d       pose{Eigen::Isometry3d::Identity()}; ///< T_wc (world-from-camera)
    std::vector<Observation> observations;                        ///< All landmarks visible here
    double                  timestamp{0.0};                      ///< Seconds (arbitrary epoch)
};

// ─────────────────────────────────────────────────────────────────────────────
// Per-frame output
// ─────────────────────────────────────────────────────────────────────────────

/// Result returned by VisualSlam::processFrame() on success.
struct TrackingResult {
    /// High-level tracking state machine state.
    enum class Status {
        Tracking,    ///< Pose estimated successfully
        Lost,        ///< Too few feature matches — pose unreliable
        Relocalized  ///< Tracking recovered after a lost episode (via loop closure)
    };

    Status            status{Status::Lost};
    Eigen::Isometry3d cameraPose{Eigen::Isometry3d::Identity()}; ///< T_wc estimate
    int               trackedLandmarks{0};   ///< Number of landmarks matched this frame
    bool              isKeyFrame{false};     ///< True when this frame was inserted as a keyframe
};

// ─────────────────────────────────────────────────────────────────────────────
// Error type
// ─────────────────────────────────────────────────────────────────────────────

/// Error descriptor returned inside std::expected on failure paths.
struct SlamError {
    /// Enumerated failure reason.
    enum class Code {
        NotInitialized,  ///< processFrame called before the map is bootstrapped
        TrackingLost,    ///< Insufficient feature matches to estimate pose
        MapEmpty,        ///< Operation requires at least one landmark
        NumericalFailure ///< Linear algebra / optimisation failed to converge
    };

    Code        code{Code::NotInitialized};
    std::string message;
};

// ─────────────────────────────────────────────────────────────────────────────
// Configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Tunable parameters controlling SLAM behaviour.
struct VisualSlamConfig {
    /// Minimum number of features required in the first frame to bootstrap the map.
    int minInitFeatures{100};

    /// If tracked landmarks fall below this threshold the system declares tracking lost.
    int minTrackedFeatures{30};

    /// Force-insert a keyframe at least every N frames (independent of feature ratio).
    int keyFrameInterval{5};

    /// Insert a keyframe when tracked features drop below this fraction of the
    /// feature count at the last keyframe (captures large viewpoint change).
    float keyFrameFeatureRatio{0.8f};

    /// Similarity score (0–1) above which a loop closure attempt is triggered.
    double loopClosureScoreThreshold{0.3};

    /// Master switch for the loop closure subsystem.
    bool enableLoopClosure{true};
};

// ─────────────────────────────────────────────────────────────────────────────
// Main SLAM class
// ─────────────────────────────────────────────────────────────────────────────

/// Feature-based Visual SLAM system.
///
/// Lifecycle:
///   1. Construct with a CameraIntrinsics and optional VisualSlamConfig.
///   2. Call processFrame() with each incoming set of detected features.
///      The system bootstraps automatically once minInitFeatures is met.
///   3. Query landmarks(), keyFrames(), and currentPose() at any time.
///   4. Call reset() to start a new session.
///
/// Thread safety: not thread-safe — callers must serialise access externally.
class VisualSlam {
public:
    // ── Construction / destruction ───────────────────────────────────────────

    /// Construct a VisualSlam instance.
    /// @param camera  Pinhole camera model; must have fx, fy > 0.
    /// @param config  Optional tuning parameters; defaults are reasonable for
    ///                VGA-resolution cameras at 30 Hz.
    explicit VisualSlam(const CameraIntrinsics& camera,
                        const VisualSlamConfig&  config = {});

    ~VisualSlam();

    // Non-copyable (owns internal mutable state).
    VisualSlam(const VisualSlam&)            = delete;
    VisualSlam& operator=(const VisualSlam&) = delete;

    // Movable.
    VisualSlam(VisualSlam&&)            noexcept = default;
    VisualSlam& operator=(VisualSlam&&) noexcept = default;

    // ── Primary interface ────────────────────────────────────────────────────

    /// A single detected feature within an input frame.
    struct FeatureObservation {
        float x{0.f};                        ///< Pixel column (sub-pixel OK)
        float y{0.f};                        ///< Pixel row   (sub-pixel OK)
        std::array<uint8_t, 32> descriptor{}; ///< Binary descriptor (e.g. ORB-256)
        float depth{0.f};                    ///< Metric depth in metres; 0 = unknown (monocular)
    };

    /// A complete input frame: timestamp + list of detected features.
    struct FrameInput {
        double                        timestamp{0.0}; ///< Seconds (arbitrary epoch)
        std::vector<FeatureObservation> features;     ///< Detected features for this frame
    };

    /// Process one camera frame and update the SLAM state.
    ///
    /// On the first call(s), the system attempts map initialisation.  Once
    /// initialised, subsequent calls perform tracking (feature matching +
    /// PnP pose estimation) and optional loop closure detection.
    ///
    /// @param frame  Detected features and timestamp for this frame.
    /// @returns      TrackingResult on success, SlamError on unrecoverable failure.
    [[nodiscard]] std::expected<TrackingResult, SlamError>
    processFrame(const FrameInput& frame);

    // ── Map accessors ────────────────────────────────────────────────────────

    /// All landmarks currently in the map (including culled ones with isActive=false).
    [[nodiscard]] const std::vector<Landmark>&  landmarks()  const noexcept;

    /// All keyframes retained in the map, in insertion order.
    [[nodiscard]] const std::vector<KeyFrame>&  keyFrames()  const noexcept;

    /// Most recently estimated camera pose T_wc, or std::nullopt if not yet initialised.
    [[nodiscard]] std::optional<Eigen::Isometry3d> currentPose() const noexcept;

    /// True once the map has been successfully bootstrapped.
    [[nodiscard]] bool isInitialized() const noexcept;

    // ── Session control ──────────────────────────────────────────────────────

    /// Discard all map state and return to the uninitialised condition.
    /// Useful for restarting a mapping session without reallocating the object.
    void reset();

    // ── Configuration accessors ──────────────────────────────────────────────

    [[nodiscard]] const CameraIntrinsics& camera() const noexcept;
    [[nodiscard]] const VisualSlamConfig& config() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

}  // namespace visual_slam
