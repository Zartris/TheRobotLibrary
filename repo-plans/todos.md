Description and intent of the file:
This file is listing down tasks and features that we want before making it into a concrete milestone. There is two  list. The quick todo list is for us to quickly jot down ideas and tasks, while the fleshed out todo list is for us to have a more detailed view of what we want to do and how we want to do it.

---
Quick todo list:
---
Fleshed out todo list:

### ✅ Fleet management scaffold + M12 milestone
- [x] Add fleet management as a module (just the scaffolding), which will include features such as:
  - [x] Standard protocols like VDA 5050.
  - [x] Fleet management features such as task assignment, monitoring, and battery management.
- [x] Add milestone for populating the fleet management module with features and protocols.
- **Resolution:** `workspace/robotics/fleet_management/` scaffolded (vda5050, task_allocation, fleet_monitor, battery_management); added to M0 exit criteria; M12 milestone created (depends on M8). Fleet REST endpoints added to `workspace/architecture.md`. Brainstorm design doc at `.github/superpower/brainstorm/2026-03-13-robot-library-expansions-design.md`.

### ✅ Advanced control algorithms + M11 milestone
- [x] Add more to the control section, such as:
  - [x] Adaptive Control: Learning in Real Time → `control/adaptive/gain_scheduling/` + `control/adaptive/rls/`
  - [x] Control Barrier Functions for Safety → `control/cbf/`
  - [x] Frenet-Serret Frame Controller for Path Following → `control/frenet/`
- **Resolution:** CBF added to M3 (safety filter decorator); M11 milestone created for adaptive + Frenet (depends on M3). All scaffold files created with theory docs.

### ✅ Agent/Developer Observability
- [x] Focus on both human and agent feedback — everything observable through logging and metrics.
  - [x] Add observability criteria to all milestones.
- **Resolution:** Phase 4.5 (Observability) inserted into `module-task-template.md`; `ILogger` / `SpdlogLogger` scaffolded under `common/logging/`; observability exit criterion added to every milestone (M0–M12).
