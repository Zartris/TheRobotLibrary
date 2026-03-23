#pragma once
#include <common/laser_scan.hpp>
#include <Eigen/Core>
#include <vector>

namespace robotlib {

/// Feature point extracted from a sensor scan (used in M4 obstacle tracking).
struct Feature {
    Eigen::Vector2d position;  ///< In sensor frame (metres)
    double intensity{0.0};
    int id{-1};  ///< -1 = unassociated
};

/// Interface for extracting geometric features from raw sensor data.
///
/// Implementations: line extraction, corner detection, reflector detection.
///
/// Pre-conditions:
///   - scan.ranges must not be empty
/// Post-conditions:
///   - Returned features are in the sensor frame (not map frame)
///
/// Thread safety: implementations must be re-entrant (no shared mutable state).
class IFeatureExtractor {
public:
    virtual ~IFeatureExtractor() = default;
    virtual std::vector<Feature> extract(const LaserScan& scan) = 0;
};

}  // namespace robotlib
