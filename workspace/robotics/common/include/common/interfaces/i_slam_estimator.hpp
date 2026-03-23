#pragma once
#include <common/interfaces/i_state_estimator.hpp>
#include <common/occupancy_grid.hpp>
#include <Eigen/Core>
#include <vector>

namespace robotlib {

struct Landmark {
    Eigen::Vector2d position;
    int id{-1};
};

/// Interface for simultaneous localisation and mapping (SLAM) estimators.
///
/// Extends IStateEstimator with access to the incrementally-built map and
/// extracted landmarks.
///
/// Pre-conditions:
///   - Same predict/update pre-conditions as IStateEstimator
///
/// Post-conditions:
///   - getMap() returns the current best-estimate occupancy grid (may be sparse)
///   - getLandmarks() returns the current landmark set; ids are stable once assigned
///
/// Thread safety: not thread-safe; map and landmark data must be read from the
///   same thread that drives predict/update.
class ISlamEstimator : public IStateEstimator {
public:
    virtual const OccupancyGrid& getMap() const = 0;
    virtual std::vector<Landmark> getLandmarks() const = 0;
};

}  // namespace robotlib
