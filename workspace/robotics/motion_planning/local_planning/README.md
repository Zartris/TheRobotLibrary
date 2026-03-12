# local_planning

Real-time velocity command generation that follows a global path while reacting to
dynamic obstacles not present in the static map.

---

## Sub-modules

| Sub-module | Description |
|---|---|
| [`dwa/`](dwa/) | Dynamic Window Approach — scores candidate velocities in real time |

---

## Role in the planning stack

The local planner receives:
- A reference trajectory or path from the global planner / trajectory planner
- Current robot state (pose, velocity)
- Current sensor data (obstacles detected right now)

And produces a single velocity command $(v, \omega)$ every 50–100 ms.

It does **not** replan the global path — it only steers within the current plan.
If the local planner detects the path is blocked, it signals the global planner to replan.

---

## See also

[`docs/theory.md`](docs/theory.md) for local planning theory.
