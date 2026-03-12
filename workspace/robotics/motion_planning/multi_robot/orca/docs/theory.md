# Theory: Velocity Obstacles → RVO → ORCA

---

## 1. Velocity Obstacles (VO)

Robot $A$ has position $\mathbf{p}_A$, velocity $\mathbf{v}_A$.
Robot $B$ has position $\mathbf{p}_B$, velocity $\mathbf{v}_B$.
Combined radius: $r = r_A + r_B$.

The **velocity obstacle** $\text{VO}_{A|B}^\tau$ is the set of velocities that robot $A$
could choose that would cause a collision with $B$ within time $\tau$, assuming $B$
continues with $\mathbf{v}_B$:

$$\text{VO}_{A|B}^\tau = \left\{ \mathbf{v} \;\middle|\; \exists\, t \in [0,\tau] : \mathbf{p}_A + \mathbf{v}\,t \in D(\mathbf{p}_B + \mathbf{v}_B\,t,\, r) \right\}$$

where $D(\mathbf{c}, r)$ is the disk of radius $r$ centered at $\mathbf{c}$.

Expanding: $\text{VO}_{A|B}^\tau$ is a **truncated cone** in velocity space:
- Apex (for $\tau \to \infty$): $\mathbf{v}_B$
- Opening angle: $\sin\alpha = r / \|\mathbf{p}_B - \mathbf{p}_A\|$
- Axis direction: $(\mathbf{p}_B - \mathbf{p}_A) / \|\mathbf{p}_B - \mathbf{p}_A\|$
- Truncated at $\tau$: velocities less than $\|\mathbf{p}_B - \mathbf{p}_A\|/\tau$ are safe

**Safe velocity selection:** choose any $\mathbf{v}_A \notin \bigcup_B \text{VO}_{A|B}^\tau$.

**Problem:** VO does not account for the fact that $B$ is also attempting to avoid $A$.
Both agents make large avoidance maneuvers simultaneously → oscillations.

---

## 2. Reciprocal Velocity Obstacles (RVO)

**Insight:** if both $A$ and $B$ are trying to avoid each other, each only needs to
deviate by *half* of the required avoidance.

$\text{RVO}_{A|B}^\tau$ is the VO cone **shifted to be centered at**
$\frac{\mathbf{v}_A + \mathbf{v}_B}{2}$ rather than $\mathbf{v}_B$:

$$\text{RVO}_{A|B}^\tau = \mathbf{v}_B + \tfrac{1}{2}(\text{VO}_{A|B}^\tau - \mathbf{v}_B)
= \tfrac{\mathbf{v}_A + \mathbf{v}_B}{2} + \tfrac{1}{2}(\text{VO}_{A|B}^\tau - \mathbf{v}_A)$$

**Proof of collision avoidance:** if $\mathbf{v}_A \notin \text{RVO}_{A|B}^\tau$ and
$\mathbf{v}_B \notin \text{RVO}_{B|A}^\tau$ simultaneously, then $\mathbf{v}_A - \mathbf{v}_B$ lies
outside $\text{VO}_{A|B}^\tau$ — so $A$ and $B$ do not collide.

**Drawback:** the VO cone still has to be computed at each step, and for multiple agents
the problem of finding a velocity **outside all** $\text{RVO}$ cones may be complex.

---

## 3. ORCA (Optimal Reciprocal Collision Avoidance)

ORCA replaces the VO cone geometry with **half-planes** in velocity space, making the
multi-agent problem a clean Linear Program.

**Step 1: Compute the velocity obstacle vector**

$$\mathbf{u}_{A|B} = \arg\min_{\mathbf{u}} \|\mathbf{u}\| \quad \text{s.t.} \quad
(\mathbf{v}_A - \mathbf{v}_B) + \mathbf{u} \notin \text{VO}_{A|B}^\tau$$

This is the **minimum change** in relative velocity to just escape the VO cone.
It points from the current relative velocity $\mathbf{v}_A - \mathbf{v}_B$ to the
nearest point on the boundary of the cone.

**Step 2: Each agent takes half the responsibility**

$$\text{ORCA}_{A|B}^\tau = \left\{ \mathbf{v} \;\middle|\; (\mathbf{v} - \mathbf{v}_A - \tfrac{1}{2}\mathbf{u}_{A|B}) \cdot \hat{\mathbf{n}}_{A|B} \ge 0 \right\}$$

This is a **half-plane** in velocity space, where $\hat{\mathbf{n}}_{A|B}$ is the outward
normal of the VO cone boundary at the nearest point.

**Step 3: Solve the LP**

Agent $A$ finds the velocity closest to its preferred velocity while lying in all ORCA
half-planes:

$$\mathbf{v}_A^* = \arg\min_{\mathbf{v} \in \bigcap_B \text{ORCA}_{A|B}^\tau \cap \mathcal{D}(0, v_{\max})} \|\mathbf{v} - \mathbf{v}_A^{\text{pref}}\|$$

This is a 2D LP with $N-1$ half-plane constraints and a disk constraint from $v_{\max}$.
It is solvable in $O(N)$ using the randomized incremental LP algorithm.

If the LP is infeasible (agents in a deadlock), fall back to minimum-penalized-velocity
selection using the linear programming 3 (LP3) heuristic.

---

## 4. Non-Holonomic Extension (NH-ORCA)

For diff-drive robots, the feasible velocity space is not a disk but a subset constrained by:
- Maximum linear speed: $v \le v_{\max}$
- Maximum angular speed: $\omega \le \omega_{\max}$
- Acceleration limits: $|v - v_{\text{curr}}| \le a_{\max}\,\Delta t$

**Approach:** approximate the non-holonomic reachable set as a convex polygon in $(v, \omega)$
or map velocities from $(v_x, v_y)$ to $(v, \omega)$ via the robot's kinematic model, then
project the ORCA solution back into the feasible $(v, \omega)$ space.

---

## 5. Deadlock Detection

ORCA can produce deadlocks when $N$ agents all block each other's preferred direction
(e.g., symmetric 4-way crossing). Common mitigations:
- Add small random perturbation to preferred velocity
- Use a global planner to route agents around each other at high level
- Apply priority ordering as a tie-breaker

---

## Further Reading

- Fiorini & Shiller — *Motion Planning in Dynamic Environments using Velocity Obstacles*, IJRR 1998
- van den Berg et al. — *Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation*, ICRA 2008
- van den Berg et al. — *Reciprocal n-Body Collision Avoidance*, ISRR 2011 (full ORCA proof)
- Snape et al. — *The Hybrid Reciprocal Velocity Obstacle*, T-RO 2011 (NH extension)
