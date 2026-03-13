# visual_slam

Feature-based Visual SLAM for TheRobotLibrary.

Tracks features across camera frames (visual odometry front-end), builds a persistent
3-D landmark map, and detects loop closures to correct accumulated drift.  This module
is the capstone of the visual perception pipeline.

---

## Pipeline

```
  ┌─────────────────────────────────────────────────────────────────────┐
  │  FrameInput                                                          │
  │  { timestamp, features[ {x, y, descriptor[32], depth} ] }          │
  └───────────────────────────────┬─────────────────────────────────────┘
                                  │ processFrame()
  ┌───────────────────────────────▼──────────────────────────┐
  │  FRONT-END                                                │
  │                                                           │
  │  1. Feature count gate                                    │
  │     ├─ < minInitFeatures → Lost (not yet initialised)    │
  │     └─ >= minInitFeatures (first time) → init map        │
  │                                                           │
  │  2. Frame-to-map matching (descriptor NN + ratio test)   │  ← TODO Phase 2
  │     └─ PnP + RANSAC → camera pose T_wc                   │  ← TODO Phase 2
  │                                                           │
  │  3. Triangulate new landmarks                             │  ← TODO Phase 2
  │                                                           │
  │  4. Keyframe decision                                     │
  │     ├─ interval criterion  (every N frames)              │
  │     └─ feature-ratio criterion  (< α × last KF count)   │
  └───────────────────────────────┬──────────────────────────┘
                                  │ on keyframe insert
  ┌───────────────────────────────▼──────────────────────────┐
  │  BACK-END (async, future phases)                          │
  │                                                           │
  │  Local Bundle Adjustment (sliding window)                 │  ← TODO Phase 3
  │  Loop Closure Detection  (BoW place recognition)         │  ← TODO Phase 3
  │  Pose Graph Optimisation (on accepted loop)              │  ← TODO Phase 3
  └───────────────────────────────┬──────────────────────────┘
                                  │
  ┌───────────────────────────────▼──────────────────────────┐
  │  TrackingResult                                           │
  │  { Status, cameraPose T_wc, trackedLandmarks, isKF }     │
  └───────────────────────────────────────────────────────────┘
```

---

## Public API

### Core types

| Type | Description |
|---|---|
| `CameraIntrinsics` | Pinhole camera model: `fx`, `fy`, `cx`, `cy`, `width`, `height` |
| `VisualSlamConfig` | Tunable parameters (feature thresholds, keyframe policy, loop closure) |
| `FrameInput` | Input frame: timestamp + list of `FeatureObservation` |
| `FeatureObservation` | Single feature: pixel `(x, y)`, binary `descriptor[32]`, optional `depth` |
| `TrackingResult` | Per-frame output: `Status`, `cameraPose`, `trackedLandmarks`, `isKeyFrame` |
| `Landmark` | 3-D map point: `id`, `position`, `descriptor`, `observationCount`, `isActive` |
| `KeyFrame` | Retained frame: `id`, `pose`, `observations`, `timestamp` |
| `SlamError` | Error returned in `std::expected` failure path |

### `VisualSlam` methods

| Method | Returns | Description |
|---|---|---|
| `VisualSlam(camera, config)` | — | Construct with camera model and optional config |
| `processFrame(frame)` | `std::expected<TrackingResult, SlamError>` | Feed one frame; drives the full SLAM pipeline |
| `landmarks()` | `const std::vector<Landmark>&` | All landmarks in the current map |
| `keyFrames()` | `const std::vector<KeyFrame>&` | All retained keyframes, in insertion order |
| `currentPose()` | `std::optional<Eigen::Isometry3d>` | Latest camera pose $T_{wc}$, or `nullopt` if not initialised |
| `isInitialized()` | `bool` | True once the map has been bootstrapped |
| `reset()` | `void` | Discard all state and return to uninitialised condition |
| `camera()` | `const CameraIntrinsics&` | Camera model accessor |
| `config()` | `const VisualSlamConfig&` | Configuration accessor |

### `TrackingResult::Status` values

| Value | Meaning |
|---|---|
| `Tracking` | Pose successfully estimated |
| `Lost` | Too few matched features — pose unreliable |
| `Relocalized` | Tracking recovered after a lost episode via loop closure |

---

## CMake Integration

This module is built as part of the workspace:

```bash
cmake -B build -S workspace -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

To link against `visual_slam` in another module (workspace only):

```cmake
target_link_libraries(my_target PRIVATE robotlib::visual_slam)
```

To build and run tests:

```bash
cmake -B build -S workspace -DBUILD_TESTING=ON
cmake --build build --target visual_slam_tests
ctest --test-dir build -R visual_slam --output-on-failure
```

---

## Usage Example

```cpp
#include <visual_slam/visual_slam.hpp>
#include <iostream>

int main() {
    // 1. Configure camera (VGA, f ≈ 525 px).
    visual_slam::CameraIntrinsics cam;
    cam.fx = 525.0; cam.fy = 525.0;
    cam.cx = 320.0; cam.cy = 240.0;
    cam.width = 640; cam.height = 480;

    visual_slam::VisualSlamConfig cfg;
    cfg.minInitFeatures    = 100;
    cfg.minTrackedFeatures = 30;
    cfg.enableLoopClosure  = true;

    visual_slam::VisualSlam slam{cam, cfg};

    // 2. Main loop — feed one FrameInput per camera frame.
    for (int frameIdx = 0; frameIdx < 300; ++frameIdx) {
        visual_slam::VisualSlam::FrameInput frame;
        frame.timestamp = frameIdx / 30.0;  // 30 Hz

        // --- populate frame.features from your feature detector ---
        // visual_slam::VisualSlam::FeatureObservation feat;
        // feat.x = u; feat.y = v; feat.descriptor = desc; feat.depth = d;
        // frame.features.push_back(feat);

        const auto result = slam.processFrame(frame);

        if (!result) {
            std::cerr << "SLAM error: " << result.error().message << "\n";
            continue;
        }

        switch (result->status) {
            case visual_slam::TrackingResult::Status::Tracking:
                std::cout << "Frame " << frameIdx
                          << " tracked=" << result->trackedLandmarks
                          << " kf=" << result->isKeyFrame << "\n";
                break;
            case visual_slam::TrackingResult::Status::Lost:
                std::cerr << "Frame " << frameIdx << ": tracking lost\n";
                break;
            case visual_slam::TrackingResult::Status::Relocalized:
                std::cout << "Frame " << frameIdx << ": relocalized\n";
                break;
        }
    }

    std::cout << "Map contains "
              << slam.landmarks().size() << " landmarks and "
              << slam.keyFrames().size() << " keyframes.\n";

    return 0;
}
```

---

## Theory

See [docs/theory.md](docs/theory.md) for a full treatment of:
- SLAM problem formulation (MAP estimation over poses and landmarks)
- Feature detection and binary descriptor matching
- PnP pose estimation (EPnP + RANSAC)
- Bundle adjustment (sparse nonlinear least squares with Schur complement)
- Loop closure detection (Bag-of-Words) and pose graph optimisation
- Map initialisation: stereo vs. monocular (homography / essential matrix)
- Scale ambiguity in monocular SLAM
- System comparison: ORB-SLAM2/3, PTAM, LSD-SLAM, DSO

---

## Current Status

| Phase | Feature | Status |
|---|---|---|
| 1 | Module scaffold, types, CMake integration | Done |
| 1 | Map initialisation (identity pose + landmark set) | Done |
| 1 | Tracking state machine (feature-count gating) | Done |
| 1 | Keyframe insertion (interval + feature-ratio criteria) | Done |
| 1 | ILogger observability gate | Done |
| 2 | Descriptor matching + PnP pose estimation | TODO |
| 2 | New landmark triangulation | TODO |
| 2 | Local bundle adjustment | TODO |
| 3 | Bag-of-Words place recognition | TODO |
| 3 | Pose graph optimisation | TODO |
