#include <lidar_processing/line_extractor.hpp>
#include <cmath>
#include <algorithm>
#include <limits>
#include <random>
#include <sstream>
#include <chrono>

namespace robotlib {

LineExtractor::LineExtractor(const LineExtractorConfig& config)
    : m_config(config), m_logger(getLogger("lidar_processing.line_extractor")) {
    m_logger->debug("LineExtractor initialized");
}

std::vector<ExtractedLine> LineExtractor::extractLines(
    const LaserScan& scan, const Pose2D& pose) const {
    auto start = std::chrono::high_resolution_clock::now();

    // Convert scan to 2D world-frame points
    std::vector<Eigen::Vector2d> points;
    double cosT = std::cos(pose.theta);
    double sinT = std::sin(pose.theta);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < 0.01f || r > static_cast<float>(m_config.maxRange)) continue;
        double angle = scan.angleMin + static_cast<double>(i) * scan.angleIncrement;
        double lx = static_cast<double>(r) * std::cos(angle);
        double ly = static_cast<double>(r) * std::sin(angle);
        points.push_back({pose.x + cosT * lx - sinT * ly,
                          pose.y + sinT * lx + cosT * ly});
    }

    std::vector<ExtractedLine> lines;
    std::vector<bool> used(points.size(), false);
    std::mt19937 rng(42);  // deterministic for reproducibility

    for (int lineIdx = 0; lineIdx < m_config.maxLines; ++lineIdx) {
        // Collect available (unused) point indices
        std::vector<int> available;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!used[i]) available.push_back(static_cast<int>(i));
        }
        if (static_cast<int>(available.size()) < m_config.minInliers) break;

        // RANSAC: find best line
        double bestA = 0, bestB = 0, bestC = 0;
        std::vector<int> bestInliers;

        for (int iter = 0; iter < m_config.maxIterations; ++iter) {
            // Pick 2 random points
            std::uniform_int_distribution<int> dist(0, static_cast<int>(available.size()) - 1);
            int i1 = dist(rng);
            int i2 = dist(rng);
            if (i1 == i2) continue;

            const auto& p1 = points[available[i1]];
            const auto& p2 = points[available[i2]];

            // Compute line ax + by + c = 0
            double dx = p2.x() - p1.x();
            double dy = p2.y() - p1.y();
            double len = std::sqrt(dx * dx + dy * dy);
            if (len < 1e-9) continue;

            double a = -dy / len;
            double b = dx / len;
            double c = -(a * p1.x() + b * p1.y());

            // Count inliers
            std::vector<int> inliers;
            for (int idx : available) {
                double d = std::abs(a * points[idx].x() + b * points[idx].y() + c);
                if (d <= m_config.distanceThreshold) {
                    inliers.push_back(idx);
                }
            }

            if (inliers.size() > bestInliers.size()) {
                bestInliers = inliers;
                bestA = a; bestB = b; bestC = c;
            }
        }

        if (static_cast<int>(bestInliers.size()) < m_config.minInliers) break;

        // Point on line closest to origin: p0 = (-a*c, -b*c) (since a^2+b^2=1)
        Eigen::Vector2d p0{-bestA * bestC, -bestB * bestC};

        // Find line segment endpoints (project inliers onto line)
        double minProj = std::numeric_limits<double>::max();
        double maxProj = std::numeric_limits<double>::lowest();
        Eigen::Vector2d lineDir{bestB, -bestA};  // direction along line

        for (int idx : bestInliers) {
            double proj = lineDir.dot(points[idx] - p0);
            minProj = std::min(minProj, proj);
            maxProj = std::max(maxProj, proj);
        }

        ExtractedLine line;
        line.a = bestA;
        line.b = bestB;
        line.c = bestC;
        line.inlierCount = static_cast<int>(bestInliers.size());
        // Reconstruct endpoints from projection
        line.start = p0 + minProj * lineDir;
        line.end = p0 + maxProj * lineDir;

        lines.push_back(line);

        // Mark inliers as used
        for (int idx : bestInliers) used[idx] = true;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "LineExtractor: " << us << " us, " << points.size() << " points, "
        << lines.size() << " lines";
    m_logger->trace(oss.str());

    return lines;
}

}  // namespace robotlib
