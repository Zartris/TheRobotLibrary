#pragma once
#include <common/laser_scan.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace robotlib {

struct ExtractedLine {
    Eigen::Vector2d start;     ///< Line segment start (world frame)
    Eigen::Vector2d end;       ///< Line segment end (world frame)
    double a{0}, b{0}, c{0};  ///< Line equation: ax + by + c = 0 (normalized)
    int inlierCount{0};
};

struct LineExtractorConfig {
    double distanceThreshold{0.05};  ///< RANSAC inlier distance threshold (m)
    int maxIterations{100};           ///< RANSAC iterations per line
    int minInliers{10};               ///< Min points to accept a line
    int maxLines{10};                 ///< Max lines to extract
    double maxRange{10.0};            ///< Ignore scan points beyond this
};

class LineExtractor {
public:
    explicit LineExtractor(const LineExtractorConfig& config = {});

    /// Extract lines from a laser scan.
    /// @param scan  Laser scan
    /// @param pose  Robot pose (transforms points to world frame)
    /// @return Extracted line segments in world frame
    std::vector<ExtractedLine> extractLines(const LaserScan& scan, const Pose2D& pose) const;

    const LineExtractorConfig& config() const { return m_config; }

private:
    LineExtractorConfig m_config;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
