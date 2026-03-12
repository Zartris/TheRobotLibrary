# lidar_processing

Scan filtering, segmentation, and feature extraction from 2D lidar and sonar sensors.

**Standalone module.** Depends only on `common`. Copy this folder to any C++ project.

---

## What's in this module

- **Scan filtering** — median/mean filtering, range clamping, outlier rejection
- **Scan segmentation** — split a scan into segments (groups of points belonging to the same surface)
- **Feature extraction** — extract line segments, corners, and circular landmarks from a scan
- **Scan matching** — ICP-lite for scan-to-scan alignment (useful for odometry)

---

## Integration

```cmake
add_subdirectory(common)
add_subdirectory(lidar_processing)
target_link_libraries(your_target PRIVATE lidar_processing)
```

```cpp
#include <lidar_processing/scan.hpp>
#include <lidar_processing/scan_filter.hpp>
#include <lidar_processing/segmentation.hpp>

lidar_processing::Scan raw_scan = /* from sensor */;
auto filtered = lidar_processing::filter(raw_scan, {.max_range = 8.0, .median_window = 3});
auto segments = lidar_processing::segment(filtered, {.gap_threshold = 0.2});
```

---

## Theory

See [`docs/theory.md`](docs/theory.md) for the beam sensor model, scan segmentation
algorithms, and line extraction techniques.
