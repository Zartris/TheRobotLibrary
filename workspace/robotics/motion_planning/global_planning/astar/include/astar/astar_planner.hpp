#pragma once
#include <common/interfaces/i_global_planner.hpp>
#include <logging/get_logger.hpp>
#include <memory>

namespace robotlib {

class AStarPlanner : public IGlobalPlanner {
public:
    AStarPlanner();

    std::optional<Path> plan(const Pose2D& start, const Pose2D& goal,
                             const OccupancyGrid& grid) override;

private:
    struct Node {
        int x, y;
        double g, f;
        int parentX{-1}, parentY{-1};

        bool operator>(const Node& other) const { return f > other.f; }
    };

    [[nodiscard]] double heuristic(int x1, int y1, int x2, int y2) const;
    std::shared_ptr<ILogger> m_logger;
};

}  // namespace robotlib
