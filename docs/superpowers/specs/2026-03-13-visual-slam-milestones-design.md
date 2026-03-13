# Design: Visual SLAM Milestones

**Date:** 2026-03-13
**Status:** Approved
**Author:** Brainstorming session

---

## Context

TheRobotLibrary's milestone roadmap (M0–M12) lacked dedicated milestones for the visual SLAM pipeline. The gap analysis (2026-03-13) identified `common/transforms`, `perception/feature_extraction`, `perception/visual_odometry`, and `state_estimation/visual_slam` as missing modules. This spec defines how they are integrated into the milestone sequence.

---

## Architecture Rule (enforced throughout)

> **Robotics modules may only depend on `common`.** Never on each other, simulation, or frontends.

This means `visual_slam` does **not** link against `feature_extraction` or `visual_odometry`. Instead, the pipeline is assembled by the caller (simulation or application): `FeatureExtractor` → `VisualOdometry` → `VisualSlam`. Data flows via shared types in `common/` (`CameraIntrinsics`, `CameraFrame`, feature observation vectors). Each module is independently testable and reusable.

---

## Decisions

### 1. `common/transforms` → M1 (extension of M1-A)

`common/transforms` is a header-only INTERFACE module providing `SE2`, `SE3`, and `SO3` rigid-body math types built on Eigen. It extends — not replaces — the existing `Transform2D` type already in M1-A's `common`. `Transform2D` covers the simple 2D pose needs of M1 modules (EKF, DWA, A*). `common/transforms` adds the full Lie-group API (SE3, SO3, quaternion round-trips, `toSE2()` projection) needed by M6+.

**Scope note:** No M1 module requires SE3. Adding `common/transforms` to M1 expands M1-A's scope. This is acceptable because Eigen is already a `common` dependency in M1; the module is header-only and does not affect M1 runtime behaviour; and placing it in M1 avoids introducing Eigen for the first time mid-chain at M6.

**Deliverables:**
- `workspace/robotics/common/transforms/` INTERFACE target ✓ (pre-scaffolded)
- `add_subdirectory(transforms)` in `workspace/robotics/common/CMakeLists.txt` ✓ (already present)
- `common/transforms` tasks added to `repo-plans/modules/common.md` under M1-A

### 2. `CameraIntrinsics` and `CameraFrame` live in `common/`

Both `CameraIntrinsics` (fx, fy, cx, cy, width, height) and `CameraFrame` (timestamp, grayscale pixel buffer) are placed in `common/camera.hpp`. This is the only way for `visual_odometry`, `visual_slam`, and the simulation camera sensor to share these types without creating inter-module dependencies.

Note: the existing `M6-slam.md` uses the name `CameraImage`. This is renamed to `CameraFrame` for consistency. The M6.5 targeted-edits section below includes this rename.

**Deliverable:** `workspace/robotics/common/include/common/camera.hpp` — added as part of M6, tracked inside `M6-visual-perception.md`.

### 3. New M6 — "Visual Perception Building Blocks"

A new milestone M6 is inserted between M4 and the former M6 (SLAM). It delivers the perception infrastructure required by visual SLAM, independently verifiable before committing to full SLAM complexity.

#### File-system actions required when implementing this milestone

1. Rename `repo-plans/milestones/M6-slam.md` → `repo-plans/milestones/M6.5-slam.md`
2. Create `repo-plans/milestones/M6-visual-perception.md` (content defined by this spec)
3. Update `repo-plans/README.md`: milestone table and dependency graph (see Section 5); preserve the `M5 → M6.5` dependency edge (the old M6 declared M4+M5 as dependencies — M6.5 inherits M5)
4. Update `repo-plans/modules/common.md`: add `common/transforms` tasks under M1-A and `common/camera.hpp` tasks under M6

#### Modules

| Module | Domain | Description |
|--------|--------|-------------|
| `common/camera.hpp` | `common/` | `CameraIntrinsics` + `CameraFrame` shared types. Added to `common` target. |
| `feature_extraction` | `perception/` | FAST keypoints + BRIEF descriptors, no OpenCV. `FeatureExtractor` + `DescriptorMatcher` (Hamming distance, Lowe's ratio test). Depends on `common` only. |
| `visual_odometry` | `perception/` | Essential matrix decomposition (8-point + RANSAC + cheirality check). Monocular and depth-aided modes. `VisualOdometry`; uses `CameraIntrinsics` from `common/camera.hpp`. Depends on `common` only. |
| 2.5D camera renderer | `simulation/` | Wolfenstein-style per-column raycaster in `workspace/simulation/`. Wall material IDs in occupancy grid cells. Procedural texture set (brick, concrete, checker). Output: `320×240` grayscale `CameraFrame` (from `common/camera.hpp`) streamed over WebSocket at 30 Hz alongside existing lidar state. Camera configured via scenario JSON: `"camera": {"fov": 90, "resolution": [320, 240], "height_offset": 0.5}`. |

#### Dependencies

```
M4 (occupancy_grid + ray_casting — used by 2.5D camera renderer) → M6
```

M2 is implicitly in the chain (M4 depends on M2 which freezes `IStateEstimator`). M5 (particle filter) is not a direct dependency of M6.

#### Exit Criteria

1. `feature_extraction` unit tests pass — `hammingDistance` correct on identical/flipped/single-bit descriptors; `DescriptorMatcher` cross-check and ratio test verified with synthetic descriptors.
2. `visual_odometry` unit tests pass — `CameraIntrinsics::project` round-trip; essential matrix recovers known relative pose from synthetic correspondences within 5° rotation tolerance.
3. Simulation streams `CameraFrame` at 30 Hz alongside lidar state (verified by integration test).
4. Headless integration test: robot translates 5 m along a straight textured corridor; camera noise σ = 1 px; fixed RNG seed; VO pipeline (`feature_extraction` → `visual_odometry`) run over 20 frames at 0.25 m/frame; trajectory drift < 0.5 m. Test binary lives in `workspace/simulation/tests/` or a dedicated integration target — not in a robotics module test binary (sim links modules, not the reverse).
5. All modules pass Phase 4.5 Observability gate (`ILogger` injected, state transitions at DEBUG, hot-loop metrics at TRACE).

### 4. M6.5 — "SLAM" (renamed from old M6)

The existing `M6-slam.md` is renumbered `M6.5-slam.md`. Content is carried forward with the following targeted edits:

**In `visual_slam` task list — remove:**
- "Feature extraction: ORB features (implement or lightweight wrapper)"
- "Feature matching: brute-force or FLANN-like nearest-neighbor"
- "Visual odometry: essential matrix → relative pose from matched features"
- `CameraImage` type references → rename to `CameraFrame` throughout

**Also remove from `M6.5-slam.md` camera sim section:** the `include/common/camera.hpp` task (`CameraIntrinsics` / `CameraImage`) — it is now owned by M6-visual-perception.md. Do not duplicate it.

**In `visual_slam` task list — add:**
- `VisualSlam` accepts pre-computed feature observations (pixel + descriptor pairs) and pre-computed `OdometryResult` from `VisualOdometry` as inputs — these are provided by the caller, not linked internally
- `VisualSlam` is responsible for: landmark map management, keyframe selection, landmark triangulation, loop closure detection (descriptor-similarity bag-of-words implemented within `visual_slam`), pose graph correction
- `common/camera.hpp` types (`CameraIntrinsics`, `CameraFrame`) used for camera model

**Architecture note:** `visual_slam` does not link against `feature_extraction` or `visual_odometry`. The pipeline is assembled in the simulation (or user application) by wiring the three modules through `common/` types.

**Dependency note:** `ekf_slam` and `lidar_slam` do not depend on M6 — they require only M4/M5 and can be started when M6.5 begins. Only `visual_slam` strictly waits on M6.

---

## Updated Milestone Sequence

| # | Name | Key Changes vs Previous Roadmap |
|---|------|----------------------------------|
| M1 | Minimum Viable Robot | + `common/transforms` INTERFACE sub-module |
| M5 | State Estimation Upgrades | (unchanged) |
| **M6** | **Visual Perception Building Blocks** | **NEW** — `common/camera.hpp`, `feature_extraction`, `visual_odometry`, 2.5D sim renderer |
| **M6.5** | **SLAM** | Renamed from old M6; `visual_slam` inputs updated; `CameraImage` → `CameraFrame` |
| M7+ | Advanced Planning, Multi-Robot, etc. | (unchanged — M7+ numbering unaffected) |

---

## Updated Dependency Graph

```
M0 (infra)
 └→ M1 (MVR + common/transforms)
      └→ M2 (hardening)
           ├→ M3 (control)            → M11 (advanced control)
           ├→ M4 (perception)
           │    └→ M6 (visual perception)
           │         └→ M6.5 (SLAM)
           ├→ M5 (state estimation)
           │    └→ M6.5 (SLAM)
           ├→ M7 (advanced planning)  ← M4
           ├→ M8 (multi-robot)        ← M3, M7  → M12 (fleet)
           ├→ M9 (web frontend)
           └→ M10 (polish)            ← M8, M9
```

---

## Scaffold Notes

The following workspace directories were pre-scaffolded during gap analysis (ahead of this milestone workflow):

- `workspace/robotics/common/transforms/` — header-only INTERFACE, SE2/SE3/SO3 + tests ✓
- `workspace/robotics/perception/feature_extraction/` — FAST+BRIEF stub + hammingDistance + tests ✓
- `workspace/robotics/perception/visual_odometry/` — 8-point RANSAC + cheirality + tests ✓
- `workspace/robotics/state_estimation/visual_slam/` — pimpl stub + tests ✓

These scaffolds contain implementation stubs beyond what a pure scaffold phase produces. The milestone docs still need to be written to formally gate the work. The `visual_slam` scaffold's CMakeLists.txt must **not** be updated to link against `feature_extraction` or `visual_odometry` — the architecture rule prohibits this.

---

## Out of Scope

- Full 3D rendering — 2.5D raycaster only
- OpenCV dependency — all feature extraction is custom C++
- GPU acceleration — CPU-only
- VIO (visual-inertial odometry) — tracked in `todos.md` as P3
- Loop closure as a separate module — implemented as a sub-task within `visual_slam` (bag-of-words similarity), not a standalone `place_recognition` module
