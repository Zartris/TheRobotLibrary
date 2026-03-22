#include <astar/astar_planner.hpp>
#include <common/geometry.hpp>
#include <queue>
#include <unordered_set>
#include <sstream>
#include <chrono>
#include <cmath>

namespace robotlib {

AStarPlanner::AStarPlanner() : m_logger(getLogger("astar")) {
    m_logger->debug("AStarPlanner initialized");
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) const {
    // Octile distance
    const int dx = std::abs(x1 - x2);
    const int dy = std::abs(y1 - y2);
    constexpr double kSqrt2 = 1.41421356237;
    return std::max(dx, dy) + (kSqrt2 - 1.0) * std::min(dx, dy);
}

std::optional<Path> AStarPlanner::plan(const Pose2D& start, const Pose2D& goal,
                                        const OccupancyGrid& grid) {
    auto startTime = std::chrono::high_resolution_clock::now();

    auto [sx, sy] = grid.toGrid(start.x, start.y);
    auto [gx, gy] = grid.toGrid(goal.x, goal.y);

    // Check start and goal validity
    if (!grid.isValid(sx, sy) || !grid.isValid(gx, gy)) {
        m_logger->debug("A* failed: start or goal out of bounds");
        return std::nullopt;
    }
    if (grid.cellState(sx, sy) == CellState::Occupied ||
        grid.cellState(gx, gy) == CellState::Occupied) {
        m_logger->debug("A* failed: start or goal in occupied cell");
        return std::nullopt;
    }

    // Start == goal
    if (sx == gx && sy == gy) {
        return Path{Pose2D{goal.x, goal.y, goal.theta}};
    }

    // 8-connected neighbors
    constexpr int DX[] = {1, -1, 0, 0, 1, -1, 1, -1};
    constexpr int DY[] = {0, 0, 1, -1, 1, 1, -1, -1};
    constexpr double COST[] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;

    const int W = grid.width;
    const int H = grid.height;
    std::vector<double> gScore(W * H, std::numeric_limits<double>::infinity());
    std::vector<int> parentX(W * H, -1);
    std::vector<int> parentY(W * H, -1);
    std::vector<bool> closed(W * H, false);

    auto idx = [W](int x, int y) { return y * W + x; };

    gScore[idx(sx, sy)] = 0.0;
    openSet.push(Node{sx, sy, 0.0, heuristic(sx, sy, gx, gy)});

    int nodesExpanded = 0;

    while (!openSet.empty()) {
        auto current = openSet.top();
        openSet.pop();

        if (current.x == gx && current.y == gy) {
            // Reconstruct path
            Path path;
            int cx = gx, cy = gy;
            while (cx != -1) {
                auto [wx, wy] = grid.toWorld(cx, cy);
                path.push_back(Pose2D{wx, wy, 0.0});
                int px = parentX[idx(cx, cy)];
                int py = parentY[idx(cx, cy)];
                cx = px;
                cy = py;
            }
            std::reverse(path.begin(), path.end());

            // Set heading along path
            for (size_t i = 0; i + 1 < path.size(); ++i) {
                path[i].theta = std::atan2(path[i + 1].y - path[i].y,
                                           path[i + 1].x - path[i].x);
            }
            if (path.size() > 1) {
                path.back().theta = path[path.size() - 2].theta;
            }

            auto endTime = std::chrono::high_resolution_clock::now();
            auto us = std::chrono::duration_cast<std::chrono::microseconds>(
                          endTime - startTime)
                          .count();
            std::ostringstream oss;
            oss << "A* found path: " << path.size() << " waypoints, "
                << nodesExpanded << " nodes expanded in " << us << " us";
            m_logger->debug(oss.str());

            return path;
        }

        int ci = idx(current.x, current.y);
        if (closed[ci]) continue;
        closed[ci] = true;
        ++nodesExpanded;

        for (int d = 0; d < 8; ++d) {
            int nx = current.x + DX[d];
            int ny = current.y + DY[d];

            if (!grid.isValid(nx, ny)) continue;
            if (grid.cellState(nx, ny) == CellState::Occupied) continue;

            int ni = idx(nx, ny);
            if (closed[ni]) continue;

            double tentG = gScore[ci] + COST[d];
            if (tentG < gScore[ni]) {
                gScore[ni] = tentG;
                parentX[ni] = current.x;
                parentY[ni] = current.y;
                openSet.push(
                    Node{nx, ny, tentG, tentG + heuristic(nx, ny, gx, gy)});
            }
        }
    }

    m_logger->debug("A* failed: no path found");
    return std::nullopt;
}

}  // namespace robotlib
