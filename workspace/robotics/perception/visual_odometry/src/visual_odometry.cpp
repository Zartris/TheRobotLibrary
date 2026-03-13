#include <visual_odometry/visual_odometry.hpp>
#include <common/logging/logger.hpp>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <format>
#include <memory>
#include <numbers>
#include <random>

namespace visual_odometry {

// ── Helpers ───────────────────────────────────────────────────────────────

namespace {

/// Normalise a set of 2D points: translate centroid to origin and scale so
/// mean distance to origin is sqrt(2).  Returns the 3x3 normalisation matrix T
/// such that  x_normalised = T * x_homogeneous.
Eigen::Matrix3d computeNormalisationMatrix(const std::vector<Eigen::Vector2d>& pts) {
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const auto& p : pts) {
        centroid += p;
    }
    centroid /= static_cast<double>(pts.size());

    double meanDist = 0.0;
    for (const auto& p : pts) {
        meanDist += (p - centroid).norm();
    }
    meanDist /= static_cast<double>(pts.size());

    const double scale = (meanDist > 1e-10) ? (std::numbers::sqrt2 / meanDist) : 1.0;

    Eigen::Matrix3d T = Eigen::Matrix3d::Zero();
    T(0, 0) = scale;
    T(1, 1) = scale;
    T(2, 2) = 1.0;
    T(0, 2) = -scale * centroid.x();
    T(1, 2) = -scale * centroid.y();
    return T;
}

/// Build the 9×9 (or Nx9) design matrix A for the 8-point algorithm.
/// Each correspondence contributes one row:  x2^T * F * x1 = 0
/// Row = [ x2*x1,  x2*y1,  x2,  y2*x1,  y2*y1,  y2,  x1,  y1,  1 ]
Eigen::MatrixXd buildDesignMatrix(const std::vector<PointMatch>& matches,
                                  const Eigen::Matrix3d& T1,
                                  const Eigen::Matrix3d& T2) {
    const int N = static_cast<int>(matches.size());
    Eigen::MatrixXd A(N, 9);

    for (int i = 0; i < N; ++i) {
        const Eigen::Vector3d p1 = T1 * Eigen::Vector3d{matches[static_cast<size_t>(i)].point1.x(),
                                                         matches[static_cast<size_t>(i)].point1.y(),
                                                         1.0};
        const Eigen::Vector3d p2 = T2 * Eigen::Vector3d{matches[static_cast<size_t>(i)].point2.x(),
                                                         matches[static_cast<size_t>(i)].point2.y(),
                                                         1.0};

        A(i, 0) = p2.x() * p1.x();
        A(i, 1) = p2.x() * p1.y();
        A(i, 2) = p2.x();
        A(i, 3) = p2.y() * p1.x();
        A(i, 4) = p2.y() * p1.y();
        A(i, 5) = p2.y();
        A(i, 6) = p1.x();
        A(i, 7) = p1.y();
        A(i, 8) = 1.0;
    }
    return A;
}

/// Enforce rank-2 constraint on fundamental matrix by zeroing the smallest
/// singular value.
Eigen::Matrix3d enforceRank2(const Eigen::Matrix3d& F) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d s = svd.singularValues();
    s(2) = 0.0;
    return svd.matrixU() * s.asDiagonal() * svd.matrixV().transpose();
}

/// Compute the Sampson distance for a single correspondence (x2^T F x1)^2 / denom.
/// Used as the RANSAC inlier test for the fundamental matrix.
double sampsonDistance(const Eigen::Matrix3d& F,
                       const Eigen::Vector3d& x1,
                       const Eigen::Vector3d& x2) {
    const double num   = x2.dot(F * x1);
    const Eigen::Vector3d Fx1  = F * x1;
    const Eigen::Vector3d Ftx2 = F.transpose() * x2;
    const double denom = Fx1(0) * Fx1(0) + Fx1(1) * Fx1(1)
                       + Ftx2(0) * Ftx2(0) + Ftx2(1) * Ftx2(1);
    if (std::abs(denom) < 1e-12) { return std::numeric_limits<double>::max(); }
    return (num * num) / denom;
}

/// Decompose essential matrix E into four (R, t) candidate solutions.
/// E = U * diag(1,1,0) * V^T
/// Returns vector of four (R, t) pairs.
std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>>
decomposeEssentialMatrix(const Eigen::Matrix3d& E) {
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);

    const Eigen::Matrix3d U = svd.matrixU();
    const Eigen::Matrix3d V = svd.matrixV();

    // W = [[0,-1,0],[1,0,0],[0,0,1]]
    Eigen::Matrix3d W;
    W <<  0.0, -1.0,  0.0,
          1.0,  0.0,  0.0,
          0.0,  0.0,  1.0;

    // Ensure valid rotation matrices: det(U), det(V) must be +1.
    const Eigen::Matrix3d U_fixed = (U.determinant() < 0.0) ? (-U).eval() : U;
    const Eigen::Matrix3d V_fixed = (V.determinant() < 0.0) ? (-V).eval() : V;

    const Eigen::Matrix3d R1 = U_fixed * W       * V_fixed.transpose();
    const Eigen::Matrix3d R2 = U_fixed * W.transpose() * V_fixed.transpose();
    const Eigen::Vector3d t  = U_fixed.col(2);

    return {
        {R1,  t},
        {R1, -t},
        {R2,  t},
        {R2, -t}
    };
}

/// Cheirality test: count how many of the given correspondences triangulate to
/// a point in front of both cameras for the candidate (R, t).
/// Camera 1 is at origin [I|0], camera 2 is at [R|t].
int cheiralityCount(const Eigen::Matrix3d& R,
                    const Eigen::Vector3d& t,
                    const std::vector<PointMatch>& matches,
                    const CameraIntrinsics& cam) {
    int count = 0;
    const Eigen::Matrix3d Kinv = cam.K().inverse();

    for (const auto& m : matches) {
        const Eigen::Vector3d x1 = Kinv * Eigen::Vector3d{m.point1.x(), m.point1.y(), 1.0};
        const Eigen::Vector3d x2 = Kinv * Eigen::Vector3d{m.point2.x(), m.point2.y(), 1.0};

        // Linear triangulation (DLT) for this pair.
        // P1 = [I | 0],  P2 = [R | t]
        Eigen::Matrix4d A_tri = Eigen::Matrix4d::Zero();
        A_tri.row(0) = x1.x() * Eigen::Vector4d{0, 0, 1, 0}.transpose()
                     - Eigen::Vector4d{1, 0, 0, 0}.transpose();
        A_tri.row(1) = x1.y() * Eigen::Vector4d{0, 0, 1, 0}.transpose()
                     - Eigen::Vector4d{0, 1, 0, 0}.transpose();

        Eigen::Matrix<double, 3, 4> P2;
        P2.leftCols(3)  = R;
        P2.rightCols(1) = t;

        A_tri.row(2) = x2.x() * P2.row(2) - P2.row(0);
        A_tri.row(3) = x2.y() * P2.row(2) - P2.row(1);

        Eigen::JacobiSVD<Eigen::Matrix4d> svd_tri(A_tri, Eigen::ComputeFullV);
        const Eigen::Vector4d X = svd_tri.matrixV().col(3);

        if (std::abs(X(3)) < 1e-10) { continue; }

        const double depth1 = X(2) / X(3);

        // Depth in camera 2
        const Eigen::Vector3d X3D = X.head<3>() / X(3);
        const Eigen::Vector3d X_cam2 = R * X3D + t;
        const double depth2 = X_cam2.z();

        if (depth1 > 0.0 && depth2 > 0.0) {
            ++count;
        }
    }
    return count;
}

} // anonymous namespace

// ── VisualOdometry::Impl ──────────────────────────────────────────────────

struct VisualOdometry::Impl {
    CameraIntrinsics          camera;
    VisualOdometryConfig      config;
    std::shared_ptr<common::ILogger> logger;

    explicit Impl(const CameraIntrinsics& cam, const VisualOdometryConfig& cfg)
        : camera{cam}
        , config{cfg}
        , logger{common::getLogger("visual_odometry")}
    {}
};

// ── CameraIntrinsics ──────────────────────────────────────────────────────

std::optional<Eigen::Vector2d>
CameraIntrinsics::project(const Eigen::Vector3d& point3d) const noexcept {
    if (point3d.z() <= 0.0) {
        return std::nullopt;
    }
    const double u = fx * point3d.x() / point3d.z() + cx;
    const double v = fy * point3d.y() / point3d.z() + cy;
    return Eigen::Vector2d{u, v};
}

Eigen::Vector3d
CameraIntrinsics::unproject(const Eigen::Vector2d& pixel) const noexcept {
    return Eigen::Vector3d{(pixel.x() - cx) / fx,
                           (pixel.y() - cy) / fy,
                           1.0}.normalized();
}

Eigen::Matrix3d CameraIntrinsics::K() const noexcept {
    Eigen::Matrix3d k = Eigen::Matrix3d::Zero();
    k(0, 0) = fx;
    k(1, 1) = fy;
    k(0, 2) = cx;
    k(1, 2) = cy;
    k(2, 2) = 1.0;
    return k;
}

// ── VisualOdometry ────────────────────────────────────────────────────────

VisualOdometry::VisualOdometry(const CameraIntrinsics& camera,
                               const VisualOdometryConfig& config)
    : m_impl{std::make_unique<Impl>(camera, config)}
{}

VisualOdometry::~VisualOdometry() = default;

const CameraIntrinsics& VisualOdometry::camera() const noexcept {
    return m_impl->camera;
}

const VisualOdometryConfig& VisualOdometry::config() const noexcept {
    return m_impl->config;
}

std::expected<OdometryResult, VoError>
VisualOdometry::estimatePose(const std::vector<PointMatch>& matches) const {
    m_impl->logger->info(std::format("estimatePose called with {} matches", matches.size()));

    // ── Guard: insufficient matches ──────────────────────────────────────
    if (static_cast<int>(matches.size()) < m_impl->config.minMatchesRequired) {
        const std::string msg = std::format(
            "insufficient matches: got {} but require {}",
            matches.size(), m_impl->config.minMatchesRequired);
        m_impl->logger->warn(msg);
        return std::unexpected(VoError{VoError::Code::InsufficientMatches, msg});
    }

    const int N = static_cast<int>(matches.size());
    const Eigen::Matrix3d K = m_impl->camera.K();

    // ── Normalise pixel coordinates for better numerical conditioning ────
    std::vector<Eigen::Vector2d> pts1, pts2;
    pts1.reserve(static_cast<size_t>(N));
    pts2.reserve(static_cast<size_t>(N));
    for (const auto& m : matches) {
        pts1.push_back(m.point1);
        pts2.push_back(m.point2);
    }

    const Eigen::Matrix3d T1 = computeNormalisationMatrix(pts1);
    const Eigen::Matrix3d T2 = computeNormalisationMatrix(pts2);

    // ── RANSAC loop: robust fundamental matrix estimation ─────────────────
    // TODO: Replace with dedicated 5-point (Nister 2004) essential matrix solver
    //       for improved minimal-sample efficiency.

    const double sampsonThreshold = m_impl->config.ransacThresholdPx
                                  * m_impl->config.ransacThresholdPx;

    std::mt19937 rng{42};
    std::uniform_int_distribution<int> dist(0, N - 1);

    Eigen::Matrix3d bestF = Eigen::Matrix3d::Zero();
    int bestInlierCount   = 0;

    for (int iter = 0; iter < m_impl->config.maxRansacIterations; ++iter) {
        // Sample 8 random correspondences.
        std::vector<PointMatch> sample;
        sample.reserve(8);
        std::vector<int> used;
        used.reserve(8);
        int attempts = 0;
        while (static_cast<int>(sample.size()) < 8 && attempts < 100) {
            const int idx = dist(rng);
            if (std::find(used.begin(), used.end(), idx) == used.end()) {
                sample.push_back(matches[static_cast<size_t>(idx)]);
                used.push_back(idx);
            }
            ++attempts;
        }
        if (static_cast<int>(sample.size()) < 8) { continue; }

        // Build design matrix for this 8-point sample.
        const Eigen::MatrixXd A = buildDesignMatrix(sample, T1, T2);

        // Solve A * f = 0 via SVD; solution is last column of V.
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
        const Eigen::VectorXd fvec = svd.matrixV().col(8);

        Eigen::Matrix3d F;
        F << fvec(0), fvec(1), fvec(2),
             fvec(3), fvec(4), fvec(5),
             fvec(6), fvec(7), fvec(8);

        // Enforce rank-2 and denormalise: F_pixel = T2^T * F_norm * T1
        F = enforceRank2(F);
        F = T2.transpose() * F * T1;

        // Guard against degenerate F.
        if (!F.allFinite()) { continue; }

        // Count inliers.
        int inliers = 0;
        for (const auto& m : matches) {
            const Eigen::Vector3d x1{m.point1.x(), m.point1.y(), 1.0};
            const Eigen::Vector3d x2{m.point2.x(), m.point2.y(), 1.0};
            if (sampsonDistance(F, x1, x2) < sampsonThreshold) {
                ++inliers;
            }
        }

        if (inliers > bestInlierCount) {
            bestInlierCount = inliers;
            bestF           = F;
        }

        // Early termination: high inlier fraction.
        if (static_cast<double>(bestInlierCount) / static_cast<double>(N) > 0.95) {
            break;
        }
    }

    m_impl->logger->debug(std::format("RANSAC best inlier count: {}/{}", bestInlierCount, N));

    // ── Inlier ratio check ───────────────────────────────────────────────
    const double inlierRatio = static_cast<double>(bestInlierCount) / static_cast<double>(N);
    if (inlierRatio < m_impl->config.minInlierRatio || bestInlierCount < 5) {
        const std::string msg = std::format(
            "degenerate configuration: inlier ratio {:.3f} below threshold {:.3f}",
            inlierRatio, m_impl->config.minInlierRatio);
        m_impl->logger->warn(msg);
        return std::unexpected(VoError{VoError::Code::DegenerateConfiguration, msg});
    }

    // ── Convert fundamental matrix to essential matrix: E = K^T * F * K ─
    const Eigen::Matrix3d E = K.transpose() * bestF * K;

    // Guard against non-finite E.
    if (!E.allFinite()) {
        return std::unexpected(VoError{VoError::Code::NumericalFailure,
                                       "essential matrix contains non-finite values"});
    }

    // ── Decompose E into four candidate (R, t) poses ─────────────────────
    const auto candidates = decomposeEssentialMatrix(E);

    // ── Cheirality test: pick candidate with most points in front ────────
    // Use only inlier matches for the cheirality test.
    std::vector<PointMatch> inlierMatches;
    inlierMatches.reserve(static_cast<size_t>(bestInlierCount));
    for (const auto& m : matches) {
        const Eigen::Vector3d x1{m.point1.x(), m.point1.y(), 1.0};
        const Eigen::Vector3d x2{m.point2.x(), m.point2.y(), 1.0};
        if (sampsonDistance(bestF, x1, x2) < sampsonThreshold) {
            inlierMatches.push_back(m);
        }
    }

    int bestCandidateIdx   = 0;
    int bestCandidateScore = -1;
    for (int c = 0; c < 4; ++c) {
        const auto& [R, t] = candidates[static_cast<size_t>(c)];
        const int score = cheiralityCount(R, t, inlierMatches, m_impl->camera);
        if (score > bestCandidateScore) {
            bestCandidateScore = score;
            bestCandidateIdx   = c;
        }
    }

    const auto& [R_best, t_best] = candidates[static_cast<size_t>(bestCandidateIdx)];

    m_impl->logger->debug(std::format(
        "selected candidate {} with cheirality score {}/{}",
        bestCandidateIdx, bestCandidateScore, static_cast<int>(inlierMatches.size())));

    // ── Build result ──────────────────────────────────────────────────────
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear()      = R_best;
    pose.translation() = t_best;  // unit-norm; scale is ambiguous in monocular VO

    OdometryResult result;
    result.relativePose  = pose;
    result.inlierCount   = bestInlierCount;
    result.matchCount    = N;
    result.isValid       = true;

    m_impl->logger->info(std::format(
        "estimatePose succeeded: inliers={}/{} isValid={}",
        result.inlierCount, result.matchCount, result.isValid));

    return result;
}

std::expected<OdometryResult, VoError>
VisualOdometry::estimatePoseWithDepth(const std::vector<PointMatch>& matches,
                                      const std::vector<double>& depths) const {
    m_impl->logger->info(std::format(
        "estimatePoseWithDepth called with {} matches and {} depths",
        matches.size(), depths.size()));

    if (static_cast<int>(matches.size()) < m_impl->config.minMatchesRequired) {
        const std::string msg = std::format(
            "insufficient matches: got {} but require {}",
            matches.size(), m_impl->config.minMatchesRequired);
        m_impl->logger->warn(msg);
        return std::unexpected(VoError{VoError::Code::InsufficientMatches, msg});
    }

    // TODO: Implement depth-aided pose estimation (RGB-D / stereo mode).
    //   1. Filter matches to those with valid (positive) depth values.
    //   2. Lift 2D points in frame 1 to 3D using depth: X = depth * K^{-1} * x1_homogeneous.
    //   3. Solve 3D-2D PnP (perspective-n-point) for the pose T_{2<-1}:
    //      - Minimal solver: P3P (3-point algorithm by Ke & Tang or Kneip et al.).
    //      - Over-determined: DLT (Direct Linear Transform) or non-linear refinement.
    //   4. Wrap in RANSAC to handle outliers.
    //   5. Return metric pose (translation has physical scale because depth is known).

    m_impl->logger->warn(
        "estimatePoseWithDepth: depth-based pose estimation not yet implemented; "
        "returning invalid result");

    OdometryResult result;
    result.relativePose = Eigen::Isometry3d::Identity();
    result.inlierCount  = 0;
    result.matchCount   = static_cast<int>(matches.size());
    result.isValid      = false;
    return result;
}

} // namespace visual_odometry
