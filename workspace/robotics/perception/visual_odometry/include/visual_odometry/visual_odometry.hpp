#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <expected>
#include <string>
#include <vector>
#include <optional>

namespace visual_odometry {

// ── Camera model ──────────────────────────────────────────────────────────

/// Pinhole camera intrinsic parameters.
struct CameraIntrinsics {
    double fx{0.0};  ///< Focal length x (pixels)
    double fy{0.0};  ///< Focal length y (pixels)
    double cx{0.0};  ///< Principal point x (pixels)
    double cy{0.0};  ///< Principal point y (pixels)
    int    width{0};
    int    height{0};

    /// Project a 3D point (camera frame) to pixel coordinates.
    /// Returns nullopt if point is behind camera (z <= 0).
    std::optional<Eigen::Vector2d> project(const Eigen::Vector3d& point3d) const noexcept;

    /// Unproject a 2D pixel to a unit-bearing vector.
    Eigen::Vector3d unproject(const Eigen::Vector2d& pixel) const noexcept;

    /// Build 3x3 intrinsic matrix K.
    Eigen::Matrix3d K() const noexcept;
};

// ── Point correspondences ─────────────────────────────────────────────────

/// A 2D–2D point correspondence between two frames.
struct PointMatch {
    Eigen::Vector2d point1;  ///< Pixel in frame 1
    Eigen::Vector2d point2;  ///< Pixel in frame 2
};

// ── VO result ─────────────────────────────────────────────────────────────

/// Result of a visual odometry step.
struct OdometryResult {
    Eigen::Isometry3d relativePose{Eigen::Isometry3d::Identity()};  ///< T_{2<-1}: frame2-from-frame1
    int               inlierCount{0};
    int               matchCount{0};
    bool              isValid{false};  ///< false if not enough inliers or degenerate
};

/// Error type.
struct VoError {
    enum class Code { InsufficientMatches, DegenerateConfiguration, NumericalFailure };
    Code        code;
    std::string message;
};

// ── Config ────────────────────────────────────────────────────────────────

struct VisualOdometryConfig {
    int    minMatchesRequired{8};     ///< Minimum feature matches to attempt estimation
    double ransacThresholdPx{2.0};    ///< RANSAC reprojection error threshold (pixels) for essential matrix
    double ransacConfidence{0.999};   ///< RANSAC success probability
    int    maxRansacIterations{1000};
    double minInlierRatio{0.3};       ///< Reject if inlier ratio below this
    bool   useDepth{false};           ///< If true, use depth data for scale-resolved pose (RGB-D mode)
};

// ── VisualOdometry ────────────────────────────────────────────────────────

/// Frame-to-frame visual odometry via essential matrix decomposition.
///
/// Monocular mode: recovers rotation + direction of translation (scale-ambiguous).
/// RGB-D mode (useDepth=true): recovers metric translation using depth at matched points.
class VisualOdometry {
public:
    explicit VisualOdometry(const CameraIntrinsics& camera,
                            const VisualOdometryConfig& config = {});
    ~VisualOdometry();

    /// Estimate the relative pose T_{2<-1} from 2D–2D point correspondences.
    /// Input: matched pixel coordinates in frame1 and frame2.
    std::expected<OdometryResult, VoError>
    estimatePose(const std::vector<PointMatch>& matches) const;

    /// Estimate relative pose using 2D matches + per-match depth in frame1 (RGB-D / stereo mode).
    /// depths[i] is the metric depth for matches[i].point1. 0 or negative means unknown.
    std::expected<OdometryResult, VoError>
    estimatePoseWithDepth(const std::vector<PointMatch>& matches,
                          const std::vector<double>& depths) const;

    const CameraIntrinsics&       camera() const noexcept;
    const VisualOdometryConfig&   config() const noexcept;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace visual_odometry
