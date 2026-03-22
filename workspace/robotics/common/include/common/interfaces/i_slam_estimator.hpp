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

class ISlamEstimator : public IStateEstimator {
public:
    virtual const OccupancyGrid& getMap() const = 0;
    virtual std::vector<Landmark> getLandmarks() const = 0;
};

}  // namespace robotlib
