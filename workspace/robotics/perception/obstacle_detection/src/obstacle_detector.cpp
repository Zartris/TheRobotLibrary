#include <obstacle_detection/obstacle_detector.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <sstream>
#include <unordered_set>

namespace robotlib {

ObstacleDetector::ObstacleDetector(const ObstacleDetectorConfig& config)
    : m_config(config), m_logger(getLogger("obstacle_detection")) {
    m_logger->debug("ObstacleDetector initialized");
}

void ObstacleDetector::reset() {
    m_tracks.clear();
    m_nextId = 0;
    m_lastTimestamp = -1.0;
    m_logger->debug("ObstacleDetector reset");
}

std::vector<Eigen::Vector2d> ObstacleDetector::scanToPoints(
    const LaserScan& scan, const Pose2D& pose) const {
    std::vector<Eigen::Vector2d> points;
    double cosT = std::cos(pose.theta);
    double sinT = std::sin(pose.theta);

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        if (!std::isfinite(r) || r < 0.01f || r > static_cast<float>(m_config.maxRange)) {
            continue;
        }
        double angle = scan.angleMin + static_cast<double>(i) * scan.angleIncrement;
        // Transform to world frame
        double lx = static_cast<double>(r) * std::cos(angle);
        double ly = static_cast<double>(r) * std::sin(angle);
        double wx = pose.x + cosT * lx - sinT * ly;
        double wy = pose.y + sinT * lx + cosT * ly;
        points.push_back({wx, wy});
    }
    return points;
}

std::vector<int> ObstacleDetector::dbscan(const std::vector<Eigen::Vector2d>& points) const {
    const int n = static_cast<int>(points.size());
    std::vector<int> labels(n, -1);  // -1 = unvisited/noise
    int clusterId = 0;

    for (int i = 0; i < n; ++i) {
        if (labels[i] != -1) continue;  // already assigned

        // Find neighbors
        std::vector<int> neighbors;
        for (int j = 0; j < n; ++j) {
            if ((points[i] - points[j]).norm() <= m_config.clusterEps) {
                neighbors.push_back(j);
            }
        }

        if (static_cast<int>(neighbors.size()) < m_config.clusterMinPoints) {
            continue;  // noise point
        }

        // Start new cluster
        labels[i] = clusterId;
        std::unordered_set<int> visited;
        visited.insert(i);

        // Expand cluster
        size_t k = 0;
        while (k < neighbors.size()) {
            int q = neighbors[k];
            if (!visited.count(q)) {
                visited.insert(q);
                labels[q] = clusterId;

                // Find q's neighbors
                std::vector<int> qNeighbors;
                for (int j = 0; j < n; ++j) {
                    if ((points[q] - points[j]).norm() <= m_config.clusterEps) {
                        qNeighbors.push_back(j);
                    }
                }
                if (static_cast<int>(qNeighbors.size()) >= m_config.clusterMinPoints) {
                    for (int nn : qNeighbors) {
                        if (!visited.count(nn)) {
                            neighbors.push_back(nn);
                        }
                    }
                }
            }
            ++k;
        }
        ++clusterId;
    }
    return labels;
}

std::vector<DetectedObstacle> ObstacleDetector::clustersToObstacles(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<int>& labels) const {

    // Find max cluster ID
    int maxLabel = -1;
    for (int l : labels) maxLabel = std::max(maxLabel, l);
    if (maxLabel < 0) return {};

    std::vector<DetectedObstacle> obstacles;
    for (int c = 0; c <= maxLabel; ++c) {
        Eigen::Vector2d sum{0.0, 0.0};
        int count = 0;

        // Compute centroid
        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] == c) {
                sum += points[i];
                ++count;
            }
        }
        if (count == 0) continue;

        Eigen::Vector2d center = sum / static_cast<double>(count);

        // Compute bounding radius
        double maxDist = 0.0;
        for (size_t i = 0; i < points.size(); ++i) {
            if (labels[i] == c) {
                double d = (points[i] - center).norm();
                maxDist = std::max(maxDist, d);
            }
        }

        DetectedObstacle obs;
        obs.position = center;
        obs.radius = maxDist + 0.05;  // small margin
        obs.pointCount = count;
        obstacles.push_back(obs);
    }
    return obstacles;
}

void ObstacleDetector::associateAndUpdate(
    const std::vector<DetectedObstacle>& detections, double dt) {

    // Simple nearest-neighbor association
    std::vector<bool> detUsed(detections.size(), false);
    std::vector<bool> trackMatched(m_tracks.size(), false);

    for (size_t t = 0; t < m_tracks.size(); ++t) {
        double bestDist = m_config.associationThreshold;
        int bestDet = -1;

        for (size_t d = 0; d < detections.size(); ++d) {
            if (detUsed[d]) continue;
            double dist = (m_tracks[t].position - detections[d].position).norm();
            if (dist < bestDist) {
                bestDist = dist;
                bestDet = static_cast<int>(d);
            }
        }

        if (bestDet >= 0) {
            // Update track with new detection
            detUsed[bestDet] = true;
            trackMatched[t] = true;

            // Simple velocity estimate
            if (dt > 0.0) {
                m_tracks[t].velocity =
                    (detections[bestDet].position - m_tracks[t].position) / dt;
            }
            m_tracks[t].position = detections[bestDet].position;
            m_tracks[t].radius = detections[bestDet].radius;
            m_tracks[t].pointCount = detections[bestDet].pointCount;
        }
    }

    // Keep matched tracks, drop unmatched ones
    std::vector<DetectedObstacle> activeTracks;
    for (size_t t = 0; t < m_tracks.size(); ++t) {
        if (trackMatched[t]) {
            activeTracks.push_back(m_tracks[t]);
        }
        // Unmatched tracks are dropped immediately (simplified: no prediction window)
    }

    // Create new tracks for unassociated detections
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!detUsed[d]) {
            DetectedObstacle newTrack = detections[d];
            newTrack.id = m_nextId++;
            activeTracks.push_back(newTrack);
        }
    }

    m_tracks = activeTracks;
}

std::vector<DetectedObstacle> ObstacleDetector::detect(
    const LaserScan& scan, const Pose2D& pose, double timestamp) {
    auto start = std::chrono::high_resolution_clock::now();

    // 1. Convert scan to world-frame points
    auto points = scanToPoints(scan, pose);

    if (points.empty()) {
        auto end = std::chrono::high_resolution_clock::now();
        auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        std::ostringstream oss;
        oss << "ObstacleDetector: " << us << " us, 0 points, 0 obstacles";
        m_logger->trace(oss.str());
        return {};
    }

    // 2. DBSCAN clustering
    auto labels = dbscan(points);

    // 3. Convert clusters to obstacle detections
    auto detections = clustersToObstacles(points, labels);

    // 4. Compute dt from timestamps
    double dt = 0.1;  // default if no timestamp
    if (timestamp >= 0.0 && m_lastTimestamp >= 0.0) {
        dt = timestamp - m_lastTimestamp;
        if (dt <= 0.0) dt = 0.1;
    }
    if (timestamp >= 0.0) m_lastTimestamp = timestamp;

    // 5. Associate with tracks and update
    associateAndUpdate(detections, dt);

    auto end = std::chrono::high_resolution_clock::now();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    std::ostringstream oss;
    oss << "ObstacleDetector: " << us << " us, " << points.size() << " points, "
        << m_tracks.size() << " obstacles";
    m_logger->trace(oss.str());

    return m_tracks;
}

}  // namespace robotlib
