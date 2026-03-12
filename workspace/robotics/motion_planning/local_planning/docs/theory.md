# Theory: Local Planning

---

## 1. The Local Planning Problem

The global planner finds a collision-free path through the *static* map. The local
planner must handle the reality that the world is not static: other robots move, people
walk, doors open. The local planner operates at high frequency on current sensor data.

**Inputs:** current robot state, current obstacles (from sensor), reference path/goal  
**Output:** velocity command $(v, \omega)$ to be executed for the next timestep

---

## 2. Dynamic Window Approach (DWA)

DWA restricts velocity search to commands that are:
1. **Feasible:** within actuator limits $v \in [v_{\min}, v_{\max}]$, $\omega \in [\omega_{\min}, \omega_{\max}]$
2. **Reachable** within one timestep given max acceleration — the **dynamic window**:

$$V_d = [v_c - a_v \Delta t,\; v_c + a_v \Delta t] \times [\omega_c - a_\omega \Delta t,\; \omega_c + a_\omega \Delta t]$$

For each candidate $(v, \omega) \in V_d$:
- Simulate the arc trajectory forward for horizon $T$ (typically 2–3 s)
- Check if the trajectory is collision-free
- Score with objective $G$

### Objective function

$$G(v, \omega) = \sigma\bigl(w_h \cdot \text{heading}(v,\omega) + w_c \cdot \text{clearance}(v,\omega) + w_v \cdot \text{velocity}(v,\omega)\bigr)$$

- **Heading:** $\pi - |\text{angle to goal} - \text{robot heading after arc}|$ — prefer arcs pointing toward goal
- **Clearance:** minimum distance to any obstacle on the arc — prefer safe arcs
- **Velocity:** $v / v_{\max}$ — prefer faster arcs (progress)
- $\sigma$ is a smoothing/normalization function

Select $(v^*, \omega^*) = \arg\max G$.

---

## 3. DWA vs. TEB

DWA is **reactive** — it only optimizes one step into the future (the arc for the next $T$ seconds).
TEB (Timed Elastic Band, in `trajectory_planning/teb/`) optimizes a full time-indexed trajectory
over a longer horizon, making it smoother and more principled at the cost of higher computation.

| | DWA | TEB |
|---|---|---|
| Method | Score discrete arc candidates | Optimize a continuous trajectory |
| Horizon | Short (current velocity window) | Full path |
| Obstacles | Geometric clearance per arc | Repulsive constraints |
| Dynamics | Velocity constraints only | Full kinematic constraints |
| Computation | Very fast (ms) | Moderate (10–50 ms) |

Use DWA for straightforward environments; TEB when smooth, dynamically consistent trajectories matter.

---

## Further Reading

- Fox, Burgard, Thrun — *The Dynamic Window Approach to Collision Avoidance*, RA&M 1997
- Marder-Eppstein et al. — *The Office Marathon: Robust Navigation in an Indoor Office Environment*, ICRA 2010
