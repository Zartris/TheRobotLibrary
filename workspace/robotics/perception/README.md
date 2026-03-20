# perception

Sensor models, sensor data processing, and environment representation for mobile robots.

Each sub-folder is a **standalone C++ library** — copy only what you need. All sub-modules
depend only on `common`.

---

## Sub-modules

| Sub-module | Description |
|---|---|
| [`lidar_processing/`](lidar_processing/) | Scan filtering, segmentation, feature extraction from lidar/sonar |
| [`occupancy_grid/`](occupancy_grid/) | 2D probabilistic grid map updated from range sensor readings |
| [`ray_casting/`](ray_casting/) | Synthetic sensor simulation — cast rays through a known map |
| [`obstacle_detection/`](obstacle_detection/) | Unified obstacle detection interface; camera, lidar, mono detection |

---

## Extending this domain

Perception is intentionally designed for growth. Future sub-modules may include:
- `camera_processing/` — image-based perception, optical flow
- `feature_extraction/` — corners, lines, SIFT/ORB-style features for SLAM
- `semantic_segmentation/` — object classification from sensor data

Follow the standard sub-module layout and add to this table.

---

## Domain overview

See [`docs/theory.md`](docs/theory.md) for the probabilistic sensor model framework
before diving into a specific sub-module.

---

## Dependency rule

All sub-modules depend only on `common`. No sub-module may depend on another perception
sub-module, on `simulation`, or on the simulation app.
