# Design: TheRobotLibrary Expansions
**Date:** 2026-03-13  
**Status:** Approved  
**Topics:** Observability Standard · Advanced Control (CBF + M11) · Fleet Management (M12) · Updated Milestone Table

---

## 1. Observability Standard

### Architecture

`common/logging/` exposes `ILogger` — modules call `common::getLogger("ekf")`, returning `ILogger&`. No `#include <spdlog/...>` in module headers.

- **Default implementation:** `SpdlogLogger` wraps spdlog named loggers (spdlog is a dev/test dependency, not shipped in public headers)
- **Log line format:** `[LEVEL][MODULE] message key=value key=value`  
  Example: `[DEBUG][ekf] predict_step dt=0.020 cycle_us=142`
- **Format rationale:** grep-friendly tagged plain-text; AI agents parse via stdout in `ctest --output-on-failure`

### Phase 4.5 — Observability (GATED)

Inserted between Phase 4 (Passing Tests) and Phase 5 (Simulation Integration). **Blocks module completion.**

```markdown
### Phase 4.5 — Observability

- [ ] `ILogger` injected into module constructor (testable via mock logger)
- [ ] State transitions logged at `DEBUG` (init, reset, error paths)
- [ ] Hot loop metrics logged at `TRACE` (cycle time per iteration, iteration count)
- [ ] Tests assert expected log lines appear via stdout capture
- [ ] Zero `ERROR`-level logs during nominal test runs

\`\`\`bash
cmake --build build --target <module>_tests
cd build && ctest -R <module> --output-on-failure 2>&1 | grep "\\[DEBUG\\]\\|\\[TRACE\\]"
\`\`\`
```

### Files to Update

- `common/` — add `logging/` sub-directory with `ILogger`, `SpdlogLogger`, `getLogger()`
- `repo-plans/module-task-template.md` — insert Phase 4.5 between Phase 4 and Phase 5
- `repo-plans/milestones/M0-M10/*.md` — append to each Exit Criteria: *"All modules pass Phase 4.5 — Observability gate."*

---

## 2. Advanced Control

### CBF → Added to M3

`CbfSafetyFilter` is a **decorator** wrapping any `IController`.

```
control/cbf/
  include/cbf/cbf_safety_filter.hpp   ← CbfSafetyFilter : IController
  src/cbf_safety_filter.cpp
  tests/test_cbf_safety_filter.cpp
```

**Interface:**
```cpp
class CbfSafetyFilter : public IController {
public:
    CbfSafetyFilter(std::unique_ptr<IController> nominal, CbfConfig config);
    // compute() calls nominal, then projects via QP into safe set
    std::expected<ControlOutput, ControlError> compute(const RobotState&, const Pose2D& goal) override;
    void updateObstacles(std::span<const Obstacle> obstacles);
};
```

**Safety set:** `h(x) = ||p_robot − p_obstacle||² − r_safe²` per obstacle  
**QP solver:** Eigen-based (no additional dependency)  
**REST:** `PUT /api/robot/controller {"type":"cbf","wraps":"pid"}` — CBF wraps active controller

### M11 — Advanced Control (new milestone, depends on M3)

```
control/adaptive/
  gain_scheduling/   ← AdaptivePidController : IController
  rls/               ← RlsEstimator (feeds params to MPC/PID, not IController itself)
control/frenet/
  include/frenet/frenet_controller.hpp   ← FrenetController : IController
  src/frenet_controller.cpp
  tests/test_frenet_controller.cpp
```

**AdaptivePidController:** adjusts K_p/K_i/K_d online from tracking error history (gain scheduling).

**RlsEstimator:** Recursive Least Squares estimates mass, friction, drag; exposes `getEstimatedParams()` for consumption by MPC or adaptive PID.

**FrenetController:** decomposes path tracking into lateral error (curvature-based, like pure_pursuit) + longitudinal speed; consumes reference trajectory from `IVelocityProfiler`; implements `IController` (no new interface needed).

---

## 3. Fleet Management → M12

### Directory Layout

```
workspace/robotics/fleet_management/
├── CMakeLists.txt
├── README.md
├── docs/
│   └── theory.md
├── vda5050/           ← VDA 5050 types + nlohmann/json serialization
│   ├── CMakeLists.txt
│   ├── include/vda5050/
│   │   ├── order.hpp
│   │   ├── instant_action.hpp
│   │   ├── state.hpp
│   │   ├── connection.hpp
│   │   └── visualization.hpp
│   └── src/
├── task_allocation/   ← ITaskAllocator + AuctionAllocator / GreedyAllocator
│   ├── CMakeLists.txt
│   ├── include/task_allocation/
│   └── src/
├── fleet_monitor/     ← IFleetMonitor + FleetMonitor (state aggregation)
│   ├── CMakeLists.txt
│   ├── include/fleet_monitor/
│   └── src/
└── battery_management/ ← IBatteryManager + BatteryManager (charge routing)
    ├── CMakeLists.txt
    ├── include/battery_management/
    └── src/
```

### C++ Interfaces

```cpp
// ITaskAllocator
std::expected<RobotId, AllocationError>
assignTask(const FleetState&, const Vda5050Order&);

// IFleetMonitor
void update(RobotId, const Vda5050State&);
FleetState getFleetState() const;

// IBatteryManager
bool needsCharging(RobotId, const BatteryState&) const;
std::expected<Pose2D, BatteryError> routeToCharger(RobotId, const FleetState&);
```

### Sim Integration

Existing REST/WS server extended (no separate fleet server process):

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/fleet/task` | POST | Submit VDA 5050 Order |
| `/api/fleet/state` | GET | Full fleet state JSON |
| WS broadcast | — | Add `fleet_state` field to existing state message |

### Scaffold in M0

`fleet_management/` directory tree + `CMakeLists.txt` stubs added during M0 Dev Infrastructure scaffold phase — consistent with how all other modules are scaffolded.

---

## 4. Updated Milestone Table

| # | Name | Status | Changes | Depends On |
|---|------|--------|---------|-----------|
| M0 | Dev Infrastructure | Not Started | **+fleet_management scaffold** | — |
| M1 | Minimum Viable Robot | Not Started | +Observability exit criterion | M0 |
| M2 | Hardening & Testing | Not Started | +Observability exit criterion | M1 |
| M3 | Control Upgrades | Not Started | **+CBF sub-module**; +Observability exit criterion | M2 |
| M4 | Perception Upgrades | Not Started | +Observability exit criterion | M2 |
| M5 | State Estimation | Not Started | +Observability exit criterion | M2 |
| M6 | SLAM | Not Started | +Observability exit criterion | M4, M5 |
| M7 | Advanced Planning | Not Started | +Observability exit criterion | M4 |
| M8 | Multi-Robot | Not Started | +Observability exit criterion | M3, M7 |
| M9 | Web Frontend | Not Started | +Observability exit criterion | M2 |
| M10 | Polish & Showcase | Not Started | +Observability exit criterion | M8, M9 |
| **M11** | **Advanced Control** | Not Started | **NEW** | M3 |
| **M12** | **Fleet Management** | Not Started | **NEW** | M8 |

### Dependency Graph

```
M0 → M1 → M2 → ┬─ M3 → ┬─ M11 (Advanced Control)
                │        └─ M8 ← M7 ← M4 → M6 ← M5
                ├─ M4            ↑
                ├─ M5 ───────────┘
                └─ M9
                
M8 → M12 (Fleet Management)
M8 + M9 → M10
```

---

## File Changes Needed

### New Files
| File | Reason |
|------|--------|
| `common/logging/` (ILogger, SpdlogLogger, getLogger) | Observability architecture |
| `control/cbf/` (full sub-module scaffold) | CBF added to M3 |
| `control/adaptive/gain_scheduling/` | M11 Advanced Control |
| `control/adaptive/rls/` | M11 Advanced Control |
| `control/frenet/` | M11 Advanced Control |
| `robotics/fleet_management/` (full tree) | M12 Fleet Management |
| `repo-plans/milestones/M11-advanced-control.md` | New milestone doc |
| `repo-plans/milestones/M12-fleet-management.md` | New milestone doc |

### Modified Files
| File | Change |
|------|--------|
| `repo-plans/module-task-template.md` | Insert Phase 4.5 Observability |
| `repo-plans/milestones/M0-M10.md` (all 11) | Add Observability exit criterion + CBF to M3 |
| `workspace/robotics/CMakeLists.txt` | Add fleet_management subdirectory |
| `workspace/robotics/control/CMakeLists.txt` | Add cbf, adaptive, frenet subdirectories |
| `repo-plans/todos.md` | Move items to fleshed-out section, mark as designed |

---

## Open Questions

None — all design decisions were approved. Implementation can begin.
