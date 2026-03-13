# TheRobotLibrary — Roadmap

This folder tracks the big picture: milestones define **what** gets built and in what order; module task files track **per-module** progress within each milestone.

## Philosophy

Get the simplest version of every layer running end-to-end first (M1), then upgrade one module at a time. Each upgrade is independently testable because the full robot loop already works. The simulator lets users pick **any combination** of modules and test them live.

## Milestone Sequence

| # | Name | Focus | Status |
|---|------|-------|--------|
| **M0** | [Dev Infrastructure](milestones/M0-dev-infrastructure.md) | AI agent configs, devcontainer, CI/CD, linting, fleet_management scaffold | Not Started |
| **M1** | [Minimum Viable Robot](milestones/M1-minimum-viable-robot.md) | common + lidar + grid + A* + DWA + PID + EKF + sim + frontend | Not Started |
| **M2** | [Hardening & Testing](milestones/M2-hardening-testing.md) | ≥80% coverage, 3+ integration scenarios, interface freeze | Not Started |
| **M3** | [Control Upgrades](milestones/M3-control-upgrades.md) | pure_pursuit, MPC (acados), CBF safety filter, additional kinematics | Not Started |
| **M4** | [Perception Upgrades](milestones/M4-perception-upgrades.md) | obstacle_detection, RANSAC, inflation, dynamic obstacles | Not Started |
| **M5** | [State Estimation Upgrades](milestones/M5-state-estimation-upgrades.md) | Particle filter (MCL) as EKF alternative | Not Started |
| **M6** | [SLAM](milestones/M6-slam.md) | EKF-SLAM + lidar SLAM + camera-based visual SLAM | Not Started |
| **M7** | [Advanced Planning](milestones/M7-advanced-planning.md) | Dijkstra, RRT*, spline fitting, TEB, time-optimal | Not Started |
| **M8** | [Multi-Robot](milestones/M8-multi-robot.md) | N-robot sim + ORCA → priority → CBS → DMPC → MADER | Not Started |
| **M9** | [Web Frontend](milestones/M9-web-frontend.md) | TypeScript/React Canvas 2D app, deployable static site | Not Started |
| **M10** | [Polish & Showcase](milestones/M10-polish-showcase.md) | Docs site, examples, packaging, demo GIFs | Not Started |
| **M11** | [Advanced Control](milestones/M11-advanced-control.md) | Adaptive PID, RLS parameter estimator, Frenet controller | Not Started |
| **M12** | [Fleet Management](milestones/M12-fleet-management.md) | VDA 5050, task allocation, fleet monitor, battery management | Not Started |

## Dependency Graph

```
M0 (infra)
 └→ M1 (minimum viable robot)
      └→ M2 (hardening & testing)
           ├→ M3 (control upgrades) → M11 (advanced control)
           ├→ M4 (perception upgrades)
           ├→ M5 (state estimation upgrades)
           │    └→ M6 (SLAM) ←── M4
           ├→ M7 (advanced planning) ←── M4
           ├→ M8 (multi-robot) ←── M3, M7 → M12 (fleet management)
           ├→ M9 (web frontend)
           └→ M10 (polish & showcase) ←── M8, M9
```

After M2, milestones M3–M5 and M9 can be developed **in parallel** since they touch independent domains. M7 can start after M4 (needs costmaps). M8 needs M3 + M7. M11 requires M3 (stable control interfaces). M12 requires M8 (N-robot sim infrastructure). M10 can begin partially after M8; full completion requires all milestones.

## Folder Structure

```
repo-plans/
├── README.md                     ← This file (roadmap overview)
├── todos.md                      ← Quick scratchpad for ideas
├── module-task-template.md       ← Standard per-module workflow
├── milestones/                   ← One doc per milestone
│   ├── M0-dev-infrastructure.md
│   ├── M1-minimum-viable-robot.md
│   ├── ...
│   └── done/                     ← Completed milestones moved here
├── modules/                      ← Per-module task tracking
│   ├── common.md                 ← (created when work begins)
│   ├── pid.md
│   ├── astar.md
│   ├── ...
│   └── done/                     ← Completed module files moved here
└── plans/                        ← Misc planning artifacts
```

## Workflow

1. **Starting a milestone:** Read the milestone doc, create module task files from `module-task-template.md` for each module in scope. Milestone docs contain preliminary task breakdowns that get refined into module task files when work begins.
2. **Working on a module:** Check off tasks in `modules/<name>.md` as you go.
3. **Completing a module:** Move `modules/<name>.md` → `modules/done/<name>.md`.
4. **Completing a milestone:** Verify all exit criteria, move `milestones/M<N>-*.md` → `milestones/done/`.

> **Note:** Module task files for M2+ modules are created when starting that milestone, not upfront. Only M1 module files exist initially.

