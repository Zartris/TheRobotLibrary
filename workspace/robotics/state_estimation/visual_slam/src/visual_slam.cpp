/// @file visual_slam.cpp
/// @brief Feature-based Visual SLAM — implementation stub.
///
/// Current status: Phase 1 scaffold.
///   - Map initialisation (identity pose + initial landmark set).
///   - Tracking state machine skeleton (feature count gating).
///   - Keyframe insertion decision stub.
///   - Loop closure detection stub.
///
/// TODO (Phase 2+):
///   - Descriptor-based feature matching (Hamming distance for binary descriptors).
///   - PnP pose estimation (EPnP / DLT + RANSAC) for frame-to-map tracking.
///   - Triangulation for new landmark creation.
///   - Local bundle adjustment (Gauss-Newton / LM over sliding window).
///   - Global pose graph optimisation triggered by loop closures.
///   - Place recognition (bag-of-words or descriptor index) for loop closure.

#include <visual_slam/visual_slam.hpp>
#include <common/logging/logger.hpp>

#include <format>

namespace visual_slam {

// ─────────────────────────────────────────────────────────────────────────────
// Internal implementation structure
// ─────────────────────────────────────────────────────────────────────────────

struct VisualSlam::Impl {
    // Configuration (immutable after construction)
    CameraIntrinsics m_camera;
    VisualSlamConfig m_config;

    // Logger (Phase 4.5 observability gate)
    std::shared_ptr<common::ILogger> m_logger;

    // State
    bool                    m_initialized{false};
    std::vector<Landmark>   m_landmarks;
    std::vector<KeyFrame>   m_keyFrames;
    Eigen::Isometry3d       m_currentPose{Eigen::Isometry3d::Identity()};

    // Monotonic ID counters
    uint64_t m_nextLandmarkId{1};
    uint64_t m_nextKeyFrameId{1};

    // Tracking bookkeeping
    int     m_framesSinceLastKeyFrame{0};
    int     m_lastKeyFrameFeatureCount{0};

    explicit Impl(const CameraIntrinsics& camera, const VisualSlamConfig& config)
        : m_camera{camera}
        , m_config{config}
        , m_logger{common::getLogger("visual_slam")}
    {}

    // ── Helpers ──────────────────────────────────────────────────────────────

    /// Decide whether the current frame should be promoted to a keyframe.
    /// Criteria (either triggers insertion):
    ///   1. It has been at least keyFrameInterval frames since the last keyframe.
    ///   2. The tracked feature count has fallen below keyFrameFeatureRatio of
    ///      the last keyframe's feature count (large viewpoint change).
    [[nodiscard]] bool shouldInsertKeyFrame(int trackedFeatures) const noexcept {
        if (m_framesSinceLastKeyFrame >= m_config.keyFrameInterval) {
            return true;
        }
        const int threshold = static_cast<int>(
            static_cast<float>(m_lastKeyFrameFeatureCount) * m_config.keyFrameFeatureRatio);
        return trackedFeatures < threshold;
    }

    /// Create the initial landmark set from the first frame's features.
    /// In a monocular system depth is unknown — landmarks are placed along the
    /// camera ray at an assumed unit depth (scale-ambiguous initialisation).
    /// If metric depth is provided it is used directly.
    void initializeMap(const FrameInput& frame) {
        m_landmarks.clear();
        m_keyFrames.clear();
        m_currentPose = Eigen::Isometry3d::Identity();
        m_framesSinceLastKeyFrame = 0;
        m_lastKeyFrameFeatureCount = static_cast<int>(frame.features.size());

        // Create one landmark per input feature.
        KeyFrame kf;
        kf.id        = m_nextKeyFrameId++;
        kf.pose      = m_currentPose;
        kf.timestamp = frame.timestamp;

        for (const auto& feat : frame.features) {
            Landmark lm;
            lm.id         = m_nextLandmarkId++;
            lm.descriptor = feat.descriptor;
            lm.observationCount = 1;
            lm.isActive   = true;

            // Back-project: p_c = depth * K^{-1} * [u, v, 1]^T
            const double depth = (feat.depth > 0.f) ? static_cast<double>(feat.depth) : 1.0;
            const double xc    = (static_cast<double>(feat.x) - m_camera.cx) / m_camera.fx;
            const double yc    = (static_cast<double>(feat.y) - m_camera.cy) / m_camera.fy;
            lm.position = Eigen::Vector3d{xc * depth, yc * depth, depth};

            Observation obs;
            obs.landmarkId = lm.id;
            obs.pixel      = Eigen::Vector2d{feat.x, feat.y};
            obs.descriptor = feat.descriptor;

            kf.observations.push_back(obs);
            m_landmarks.push_back(std::move(lm));
        }

        m_keyFrames.push_back(std::move(kf));
        m_initialized = true;

        m_logger->info(std::format(
            "[visual_slam] map initialized landmarks={}",
            m_landmarks.size()));
    }

    /// Stub: track features from the current frame against the existing map.
    /// Full implementation would:
    ///   1. Project all active landmarks into the current camera frame.
    ///   2. Match projected landmarks to input features (Hamming distance).
    ///   3. Run EPnP + RANSAC to estimate the camera pose.
    ///   4. Refine with a local bundle adjustment window.
    ///   5. Triangulate unmatched features to create new landmarks.
    ///
    /// @returns Number of landmarks "tracked" (stub: returns feature count).
    [[nodiscard]] int trackFeatures(const FrameInput& frame) {
        // TODO(Phase 2): implement descriptor matching + PnP pose estimation.
        const int featureCount = static_cast<int>(frame.features.size());
        m_logger->debug(std::format(
            "[visual_slam] tracking features={}",
            featureCount));
        return featureCount;
    }

    /// Stub: attempt loop closure detection against all keyframes.
    /// Full implementation would:
    ///   1. Compute a visual place descriptor (bag-of-words or VLAD) for the
    ///      current keyframe's descriptor set.
    ///   2. Query the descriptor index for the highest-similarity past keyframe.
    ///   3. If score > loopClosureScoreThreshold, verify geometrically (PnP).
    ///   4. Add a loop constraint to the pose graph and run global optimisation.
    void attemptLoopClosure(uint64_t /*currentKeyFrameId*/) {
        if (!m_config.enableLoopClosure) {
            m_logger->debug("[visual_slam] loop closure check disabled");
            return;
        }
        // TODO(Phase 3): implement bag-of-words place recognition + pose graph optimisation.
        m_logger->debug("[visual_slam] loop closure check enabled (stub — not yet implemented)");
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// VisualSlam public method implementations
// ─────────────────────────────────────────────────────────────────────────────

VisualSlam::VisualSlam(const CameraIntrinsics& camera, const VisualSlamConfig& config)
    : m_impl{std::make_unique<Impl>(camera, config)}
{}

VisualSlam::~VisualSlam() = default;

std::expected<TrackingResult, SlamError>
VisualSlam::processFrame(const FrameInput& frame)
{
    const int featureCount = static_cast<int>(frame.features.size());

    // ── Branch 1: not yet initialised ────────────────────────────────────────
    if (!m_impl->m_initialized) {
        if (featureCount < m_impl->m_config.minInitFeatures) {
            // Not enough features to bootstrap the map yet.
            m_impl->m_logger->warn(std::format(
                "[visual_slam] insufficient features for initialisation: "
                "have={} need={}",
                featureCount,
                m_impl->m_config.minInitFeatures));

            TrackingResult result;
            result.status           = TrackingResult::Status::Lost;
            result.trackedLandmarks = 0;
            result.isKeyFrame       = false;
            return result;
        }

        // Enough features — initialise the map.
        m_impl->initializeMap(frame);

        TrackingResult result;
        result.status           = TrackingResult::Status::Tracking;
        result.cameraPose       = m_impl->m_currentPose;
        result.trackedLandmarks = featureCount;
        result.isKeyFrame       = true;  // first frame is always a keyframe
        return result;
    }

    // ── Branch 2: tracking ───────────────────────────────────────────────────
    const int tracked = m_impl->trackFeatures(frame);

    if (tracked < m_impl->m_config.minTrackedFeatures) {
        m_impl->m_logger->warn(std::format(
            "[visual_slam] tracking lost features={} threshold={}",
            tracked,
            m_impl->m_config.minTrackedFeatures));

        TrackingResult result;
        result.status           = TrackingResult::Status::Lost;
        result.cameraPose       = m_impl->m_currentPose;
        result.trackedLandmarks = tracked;
        result.isKeyFrame       = false;
        return result;
    }

    // TODO(Phase 2): update m_impl->m_currentPose from PnP result here.

    m_impl->m_framesSinceLastKeyFrame++;

    // ── Keyframe decision ─────────────────────────────────────────────────────
    bool insertedKeyFrame = false;
    if (m_impl->shouldInsertKeyFrame(tracked)) {
        KeyFrame kf;
        kf.id        = m_impl->m_nextKeyFrameId++;
        kf.pose      = m_impl->m_currentPose;
        kf.timestamp = frame.timestamp;

        // Populate observations from matched features (stub: record as new obs).
        for (const auto& feat : frame.features) {
            Observation obs;
            obs.landmarkId = 0;  // TODO(Phase 2): set from feature matching result
            obs.pixel      = Eigen::Vector2d{feat.x, feat.y};
            obs.descriptor = feat.descriptor;
            kf.observations.push_back(obs);
        }

        m_impl->m_lastKeyFrameFeatureCount = tracked;
        m_impl->m_framesSinceLastKeyFrame  = 0;
        const uint64_t kfId = kf.id;
        m_impl->m_keyFrames.push_back(std::move(kf));
        insertedKeyFrame = true;

        m_impl->m_logger->debug(std::format(
            "[visual_slam] keyframe inserted id={} total_keyframes={}",
            kfId,
            m_impl->m_keyFrames.size()));

        m_impl->attemptLoopClosure(kfId);
    }

    TrackingResult result;
    result.status           = TrackingResult::Status::Tracking;
    result.cameraPose       = m_impl->m_currentPose;
    result.trackedLandmarks = tracked;
    result.isKeyFrame       = insertedKeyFrame;
    return result;
}

const std::vector<Landmark>& VisualSlam::landmarks() const noexcept {
    return m_impl->m_landmarks;
}

const std::vector<KeyFrame>& VisualSlam::keyFrames() const noexcept {
    return m_impl->m_keyFrames;
}

std::optional<Eigen::Isometry3d> VisualSlam::currentPose() const noexcept {
    if (!m_impl->m_initialized) {
        return std::nullopt;
    }
    return m_impl->m_currentPose;
}

bool VisualSlam::isInitialized() const noexcept {
    return m_impl->m_initialized;
}

void VisualSlam::reset() {
    m_impl->m_initialized              = false;
    m_impl->m_landmarks.clear();
    m_impl->m_keyFrames.clear();
    m_impl->m_currentPose              = Eigen::Isometry3d::Identity();
    m_impl->m_nextLandmarkId           = 1;
    m_impl->m_nextKeyFrameId           = 1;
    m_impl->m_framesSinceLastKeyFrame  = 0;
    m_impl->m_lastKeyFrameFeatureCount = 0;

    m_impl->m_logger->info("[visual_slam] state reset — all map data cleared");
}

const CameraIntrinsics& VisualSlam::camera() const noexcept {
    return m_impl->m_camera;
}

const VisualSlamConfig& VisualSlam::config() const noexcept {
    return m_impl->m_config;
}

}  // namespace visual_slam
