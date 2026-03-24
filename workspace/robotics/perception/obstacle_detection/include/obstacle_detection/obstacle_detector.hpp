#pragma once
#include <common/laser_scan.hpp>
#include <common/types.hpp>
#include <logging/get_logger.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>

namespace robotlib {

struct DetectedObstacle {
    Eigen::Vector2d position{0.0, 0.0};  ///< Center in world frame
    Eigen::Vector2d velocity{0.0, 0.0};  ///< Estimated velocity (m/s)
    double radius{0.1};                   ///< Bounding radius
    int id{-1};                           ///< Stable tracking ID (-1 = untracked)
    int pointCount{0};                    ///< Number of scan points in cluster
};

struct ObstacleDetectorConfig {
    // DBSCAN parameters
    double clusterEps{0.3};           ///< Max distance between points in cluster (m)
    int clusterMinPoints{3};          ///< Min points to form a cluster
    double maxRange{10.0};            ///< Ignore scan points beyond this range

    // Tracking parameters
    double associationThreshold{0.5}; ///< Max distance to associate detection with track
    int maxMissedFrames{5};           ///< Remove track after this many missed frames
};

class ObstacleDetector {
public:
    explicit ObstacleDetector(const ObstacleDetectorConfig& config = {});

    /// Detect and track obstacles from a laser scan.
    /// @param scan  Laser scan data
    /// @param pose  Robot pose (to transform scan points to world frame)
    /// @return Detected and tracked obstacles in world frame
    std::vector<DetectedObstacle> detect(const LaserScan& scan, const Pose2D& pose);

    /// Get currently tracked obstacles (without new detection)
    const std::vector<DetectedObstacle>& trackedObstacles() const { return m_tracks; }

    /// Reset all tracks
    void reset();

    const ObstacleDetectorConfig& config() const { return m_config; }

private:
    /// Convert scan to 2D points in world frame, filtering invalid ranges
    std::vector<Eigen::Vector2d> scanToPoints(const LaserScan& scan, const Pose2D& pose) const;

    /// DBSCAN clustering on 2D points
    /// Returns cluster labels (-1 = noise, 0+ = cluster index)
    std::vector<int> dbscan(const std::vector<Eigen::Vector2d>& points) const;

    /// Convert clusters to obstacle detections
    std::vector<DetectedObstacle> clustersToObstacles(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<int>& labels) const;

    /// Associate detections with existing tracks (nearest-neighbor)
    void associateAndUpdate(const std::vector<DetectedObstacle>& detections, double dt);

    ObstacleDetectorConfig m_config;
    std::vector<DetectedObstacle> m_tracks;
    int m_nextId{0};
    double m_lastTimestamp{-1.0};
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
