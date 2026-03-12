# Theory: Priority-Based Planning

---

## 1. Algorithm

Given $N$ robots and a priority ordering $\pi = (\pi_1, \pi_2, \ldots, \pi_N)$
(index 1 is highest priority):

```
for k = 1 to N:
    robot = π[k]
    moving_obstacles = {trajectory(π[j]) : j < k}
    trajectory(robot) = single_robot_plan(robot, moving_obstacles)
    if no solution found:
        fail (or retry with different priority)
```

**Space-time obstacles:** the trajectory of robot $j$ defines, at each time $t$, a
disc of radius $r_j + r_{\text{robot}}$ that the current planner must avoid.
These discs are sampled and added as time-indexed obstacles to the A\* or RRT planner.

---

## 2. Completeness

Priority planning is **not complete** in general: there exist configurations where
a solution exists but no fixed priority ordering finds it [Erdmann & Lozano-Pérez 1987].

However, it is complete if the **priority dependency graph** has no cycles:
if an ordering exists such that robot $i$ only ever needs to wait for or yield to robot $j$
when $j$ has strictly higher priority, priority planning finds the solution.

**Practical mitigation:**
- Randomize priorities and restart on failure (randomized priority planning).
- Use the robot with the largest $\|q_i^{\text{start}} - q_i^{\text{goal}}\|$ as the
  highest-priority robot (heuristic, works well in many warehouses).

---

## 3. Optimality

The solution found is optimal **per robot given earlier robots' plans**, but the global
solution (sum of all path lengths or travel times) is not optimal in general.

**Example:** in a narrow bidirectional corridor, if the high-priority robot goes through
first, the low-priority robot must wait. Swapping priorities might produce a strictly
shorter total time.

---

## 4. Conflict-Free Speed Adjustment

Instead of replanning the path entirely, a simpler variant adjusts **speed** only:
higher-priority robots keep their planned path and speed; lower-priority robots slow
down or wait at waypoints to let higher-priority robots pass.

Algorithm:
1. Plan paths (ignoring others) for all robots simultaneously.
2. For each pair $(i, j)$ with $i$ higher priority: detect path-crossing segments.
3. Add a temporal delay to robot $j$'s departure or waypoint wait time.
4. Iterate until no conflicts remain.

This is much faster to compute and works well in warehouse gridworlds where paths cross
at fixed intersection points.

---

## 5. When Priority Planning Succeeds vs. Fails

| Scenario | Result |
|---|---|
| Sparse environment, few crossings | Typically fast and near-optimal |
| All robots going to same narrow corridor | High chance of indefinite wait |
| Symmetric group (robots swapping positions) | Deadlock — need priority to break symmetry |
| Random priorities, retry on failure | Probabilistically complete |

---

## Further Reading

- Erdmann & Lozano-Pérez — *On Multiple Moving Objects*, Algorithmica 1987
- Silver — *Cooperative Pathfinding*, AIIDE 2005 (speed adjustment variant)
- van den Berg & Overmars — *Prioritized Motion Planning for Multiple Robots*, IROS 2005
