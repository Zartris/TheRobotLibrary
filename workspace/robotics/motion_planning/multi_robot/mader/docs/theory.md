# Theory: MADER

---

## 1. Background: FASTER (Single-Agent Predecessor)

MADER is the multi-agent extension of **FASTER** (Tordesillas et al. 2019), which plans
fast single-agent trajectories through unknown 3D environments. Key ideas carried forward:

- Decompose free space into a sequence of **convex polyhedra** (flying corridors)
- Plan a **piecewise polynomial** that stays inside each polyhedron → guaranteed collision-free
- Replan at high frequency (10–20 Hz) as new obstacles are perceived

---

## 2. Polynomial Trajectory Representation

Trajectories are piecewise Bézier curves of degree $p$ (typically 7):

$$\mathbf{B}(t) = \sum_{i=0}^{p} b_i \cdot N_{i,p}(t), \quad t \in [0, T_k]$$

for each segment $k$. A Bézier curve lies within the **convex hull** of its $p+1$
control points $\{b_i\}$. This is the key property that MADER exploits.

**Continuity across segments:** enforce $C^3$ (snap-continuous) by matching the last
$3$ control points of segment $k$ with the first $3$ of segment $k+1$ via affine constraints.

---

## 3. Convex Free-Space Decomposition

To guarantee a trajectory is collision-free, MADER maps the environment to a sequence of
convex polyhedra $\{\mathcal{P}_k\}$ such that segment $k$ of the polynomial lies entirely
within $\mathcal{P}_k$:

**Inclusion condition (from Bézier convex hull property):**
$$\text{all control points } b_i^k \in \mathcal{P}_k \implies \mathbf{B}^k(t) \in \mathcal{P}_k \;\; \forall t$$

**Polyhedra construction:**
1. Run 3D JPS (Jump Point Search) on the inflated occupancy grid to find a waypoint sequence.
2. For each waypoint segment, grow a convex polyhedron via iterative plane-pushing or
   convex hull of a sphere swept along the segment.
3. Shrink by robot radius to get a margin-safe polyhedron.

---

## 4. MINVO (Minimum Volume Outer Approximation)

To check whether two Bézier trajectory segments intersect, MADER uses **MINVO polytopes**:
tight convex polytopes that contain the Bézier curve with minimum volume.

For a degree-$p$ Bézier curve, MINVO computes a set of $p+1$ polytope vertices $\{v_i\}$
via a linear change of basis from the Bernstein basis:
$$\mathbf{v} = M \cdot \mathbf{b}$$

where $M$ is a precomputed matrix that minimizes $\text{vol}(\text{ConvexHull}(v_0, \ldots, v_p))$.

**Intersection test:** check if $\text{ConvexHull}(\{v_i^A\}) \cap \text{ConvexHull}(\{v_j^B\}) \ne \emptyset$
using the GJK algorithm or separating hyperplane test.

MINVO polytopes are $\sim$15× tighter than the standard Bézier convex hull, enabling fast
intersection tests with many neighbors.

---

## 5. Check-Before-Commit Loop

**Invariant:** at time $t$, every committed trajectory segment of every agent is
collision-free with respect to all other agents' committed segments.

**Proof (two agents):**
- Agent $A$ commits $\tau_A$ only after verifying $\tau_A \cap \tau_B^{\text{committed}} = \emptyset$.
- Agent $B$ commits $\tau_B$ only after verifying $\tau_B \cap \tau_A^{\text{committed}} = \emptyset$.
- Once committed, $\tau_A$ is immutable: no retroactive conflict can arise.

**N agents:** the check is pairwise, so each agent checks against all $N-1$ neighbors.
Complexity: $O(N \cdot S \cdot H)$ per replan, where $S$ = number of segments per trajectory
and $H$ = horizon length.

**Liveness:** an agent that repeatedly fails the check (neighbors keep updating and
invalidating its plan) will fall back to a safe stop trajectory. In practice, with 10 Hz
replanning and 2-second horizons, liveness issues are rare.

---

## 6. Asynchronous Operation

Unlike DMPC (where agents implicitly assume synchronous communication), MADER tolerates:
- **Delayed messages:** the check uses the most recently received committed trajectory;
  if stale, add a safety buffer to the committed region.
- **Out-of-order messages:** each trajectory is timestamped; only apply it if it represents
  a newer commitment for that agent.

This makes MADER compatible with mesh networks and intermittent WiFi, which is essential
for real drone deployments.

---

## 7. Comparison with DMPC

| | DMPC | MADER |
|---|---|---|
| Trajectory type | Linear MPC (short horizon) | Polynomial (Bézier) |
| Collision guarantee | Convergence-based | Hard (check-before-commit) |
| Replanning | Synchronized step | Asynchronous |
| Avoidance constraint | Linearized half-plane | MINVO intersection test |
| Space representation | Obstacle list | Convex polyhedra |
| Best for | Slow ground robots | Fast drones, 3D |

---

## Further Reading

- Tordesillas & How — *MADER: Trajectory Planner in Multiagent and Dynamic Environments*, T-RO 2022
- Tordesillas et al. — *FASTER: Fast and Safe Trajectory Planner for Navigation in Unknown Environments*, IROS 2019
- Tordesillas et al. — *MINVO Basis: Finding Simplexes with Minimum Volume Enclosing Polynomial Curves*, T-RO 2022
- [mit-acl/mader GitHub](https://github.com/mit-acl/mader)
