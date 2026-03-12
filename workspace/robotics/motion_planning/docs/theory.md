# Theory: Motion Planning — Overview

This document introduces the planning problem and explains how the four sub-areas
fit together. Each sub-area has its own `docs/theory.md` with detailed algorithms.

---

## 1. What is Motion Planning?

Motion planning computes **how a robot should move** to achieve a goal. Depending on what
aspect of "how" you care about, you operate at different levels:

| Level | Cares about | Ignores |
|---|---|---|
| Global planning | Collision-free path geometry | Time, speed, dynamics |
| Local planning | Immediate velocity, dynamic obstacles | Long-horizon geometry |
| Trajectory planning | Time, speed, acceleration, smoothness | Online replanning |
| Multi-robot planning | Avoiding other robots | Single-robot constraints (per module) |

---

## 2. Configuration Space (C-space)

The **configuration space** $\mathcal{C}$ is the space of all robot configurations.
For a 2D mobile robot: $\mathcal{C} = \{(x, y, \theta)\}$.

An obstacle in physical space maps to a **C-space obstacle** via the Minkowski sum of
the obstacle with the robot's footprint. This reduces planning to **point navigation** in
C-space: find a path from $q_{\text{start}}$ to $q_{\text{goal}}$ in $\mathcal{C}_{\text{free}}$.

---

## 3. Planning Hierarchy in Practice

```
Environment map + goal
        │
   Global Planner          ── runs once (or on map change)
        │  geometric path
        ▼
  Trajectory Planner       ── runs when global path changes
        │  time-indexed trajectory
        ▼
   Local Planner           ── runs every 50–100 ms
        │  velocity command
        ▼
   Controller              ── runs every 10–20 ms
```

Global planning is slow (ms to seconds); local planning is fast (< 10 ms).
Trajectory planning sits between them, re-running when the global path changes.

---

## 4. Multi-Robot Planning: Reactive vs. Coordinated

**Reactive (e.g. ORCA):**
Each robot plans independently; avoidance happens locally at runtime. No communication
needed. Scales to large fleets. Cannot guarantee globally optimal routing.

**Coordinated (e.g. CBS, DMPC, MADER):**
Robots share plans or a central authority resolves conflicts. Can guarantee
collision-freedom and minimize total completion time. Requires communication.

The choice depends on the application: reactive for open-world large fleets,
coordinated for warehouse-style environments with defined lanes and tasks.

---

## Where to go next

- Need a path through a map? → [`../global_planning/docs/theory.md`](../global_planning/docs/theory.md)
- Need to add speed and timing to a path? → [`../trajectory_planning/docs/theory.md`](../trajectory_planning/docs/theory.md)
- Need real-time obstacle avoidance? → [`../local_planning/docs/theory.md`](../local_planning/docs/theory.md)
- Coordinating multiple robots? → [`../multi_robot/docs/theory.md`](../multi_robot/docs/theory.md)

---

## Further Reading

- LaValle — *Planning Algorithms* (free at planning.cs.uiuc.edu — comprehensive)
- Choset et al. — *Principles of Robot Motion*
- Siciliano et al. — *Robotics: Modelling, Planning and Control*
