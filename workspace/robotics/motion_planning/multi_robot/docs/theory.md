# Theory: Multi-Robot Planning

---

## 1. Problem Statement

$N$ robots share a workspace. Given start configurations $\{q_i^0\}$ and goals $\{q_i^g\}$, find
trajectories $\{\mathbf{x}_i(t)\}$ for all $i = 1, \ldots, N$ such that:

1. Each robot reaches its goal: $\mathbf{x}_i(T_i) = q_i^g$
2. No two robots collide: $\|\mathbf{x}_i(t) - \mathbf{x}_j(t)\| \ge r_i + r_j \;\; \forall i \ne j, t$
3. Each robot respects its kinematic constraints

**Multi-agent pathfinding (MAPF):** discrete version (grid, time-steps).
**Multi-robot motion planning (MRMP):** continuous version (metric space, continuous time).

---

## 2. Velocity Obstacles — The Foundational Concept

A **velocity obstacle** $\text{VO}_{i|j}$ is the set of velocities for robot $i$ that
will lead to a collision with robot $j$ at some future time $\tau$, assuming $j$ maintains
its current velocity:

$$\text{VO}_{i|j}^\tau = \left\{ \mathbf{v} \;\middle|\; \exists t \in [0,\tau] : \mathbf{p}_i + \mathbf{v}\,t \in \mathcal{D}({\mathbf{p}_j + \mathbf{v}_j\,t},\, r_i+r_j) \right\}$$

Geometrically: a cone in velocity space, apex at the origin, aligned along $\mathbf{p}_j - \mathbf{p}_i$,
with opening angle $\sin^{-1}\!\left(\frac{r_i+r_j}{\|\mathbf{p}_j-\mathbf{p}_i\|}\right)$.

**Reciprocal Velocity Obstacle (RVO):** each robot assumes the other will also deviate by
half. The cone is shifted: $\text{RVO}_{i|j} = \tfrac{1}{2}\mathbf{v}_j + \text{VO}_{i|j}$.
Robots avoid oscillation because they split the avoidance work equally.

---

## 3. ORCA (Optimal Reciprocal Collision Avoidance)

ORCA extends RVO to multiple agents and formulates each agent's velocity selection as a
**linear program (LP)**:

1. For each neighbor $j$, compute a half-plane $\text{ORCA}_{i|j}$ in velocity space: the set of
   velocities for $i$ that are safe with respect to $j$.
2. Find the velocity $\mathbf{v}_i^*$ closest to the preferred velocity $\mathbf{v}_i^{\text{pref}}$
   (toward the goal) while lying in all ORCA half-planes: $\mathbf{v}_i^* \in \bigcap_j \text{ORCA}_{i|j}$.

The LP is:
$$\mathbf{v}_i^* = \arg\min_{\mathbf{v}} \|\mathbf{v} - \mathbf{v}_i^{\text{pref}}\|^2$$
$$\text{subject to: } \mathbf{u}_{ij}^\top \mathbf{v} \ge c_{ij} \;\; \forall j \ne i$$

Solvable in $O(N)$ per agent per timestep (using RVO2 library techniques).

---

## 4. Priority-Based Planning

Assign a priority ordering $\pi: \{1, \ldots, N\} \to \{1, \ldots, N\}$.

For $k = 1, 2, \ldots, N$:
- Robot $\pi(k)$ plans its trajectory treating all higher-priority robots' trajectories
  as moving obstacles (space-time obstacles).

Each robot uses a standard single-robot planner (e.g., A\*, RRT). The space-time
obstacle for robot $j$ (higher priority) is:
$$\mathcal{O}_j(t) = \{\mathbf{x} : \|\mathbf{x} - \mathbf{x}_j(t)\| \le r_j + r_{\text{robot}}\}$$

**Properties:**
- Complete only if priorities are well-assigned (topological ordering of dependency graph).
- Not globally optimal — early priority assignment heavily influences solution quality.
- Very fast: $O(N)$ single-robot plans, each one-shot.

---

## 5. Conflict-Based Search (CBS)

Optimal (under MAPF assumptions), two-level algorithm:

**High level** — constraint tree (CT):
- Each CT node holds a set of constraints (robot $i$ cannot be at cell $v$ at time $t$)
  and a solution (per-robot paths).
- Start: no constraints, each robot plans optimally independently.
- Detect first conflict: $(i, j, v, t)$ — robots $i$ and $j$ both at $v$ at time $t$.
- Branch: create two children: one constrains robot $i$, one constrains robot $j$.
- Select node with lowest cost from priority queue (best-first search).

**Low level:** single-agent pathfinding (A\* in space-time), producing shortest path
consistent with the node's constraints.

CBS is **complete and optimal** under the MAPF formulation. It is practical for $N \le 30$
robots; enhanced variants (ECBS with focal search, CBSH with heuristics) scale to hundreds.

---

## 6. Distributed MPC (DMPC)

Each robot solves a **finite-horizon optimal control problem** at each timestep:

$$\min_{u_i} \sum_{k=0}^{H-1} \ell(x_i^k, u_i^k) + V_f(x_i^H)$$

subject to:
- Robot $i$'s dynamics: $x_i^{k+1} = f(x_i^k, u_i^k)$
- Kinematic limits: $x_i^k \in \mathcal{X}$, $u_i^k \in \mathcal{U}$
- **Inter-robot avoidance:** $\|x_i^k - x_j^k\| \ge d_{\min}$ for all $j \ne i$

The inter-robot constraints couple the problems. In distributed MPC, each robot uses
the **last communicated predicted trajectory** of neighbors to form the avoidance constraints,
then optimizes independently. This decouples the QPs — each robot solves $O(H \cdot n)$ —
with one communication step per timestep.

Convergence to a collision-free set of trajectories is guaranteed under mild assumptions
(sufficient communication rate, feasible initial trajectories).

---

## 7. MADER (MIT ACL)

MADER (Multi-Agent Decentralized trajectory Replanning) — Tordesillas & How, 2021.

Key ideas:
1. **Convex decomposition:** free space around each agent is decomposed into overlapping
   convex polyhedra (JPS + convex hull). The robot plans within these polyhedra,
   guaranteeing collision-free trajectories without checking each point explicitly.
2. **Decentralized and asynchronous:** robots share only their **committed trajectory**
   (piecewise polynomial), no synchronized replanning round needed.
3. **Check-replan loop:** before committing a new trajectory, each robot checks that it
   does not intersect any neighbor's committed trajectory. If it does, it waits and tries again.
4. **Polynomial trajectory:** MADER uses MINVO (Minimum Volume Outer Approximation)
   polytopes to efficiently check if a Bézier segment intersects the committed region.

**Formal guarantee:** if a robot commits trajectory $\tau_i$, it only commits when
$\tau_i$ does not intersect any committed trajectory of any neighbor within the
communication horizon — so committed segments are permanently collision-free.

---

## 8. Method Comparison

| Method | Centralised? | Trajectory-level | Optimal | Scales to $N$= |
|---|---|---|---|---|
| ORCA | No | No (reactive) | Local | 100s |
| Priority-based | Semi | Yes | No | 50s |
| CBS | Yes | No (paths) | Yes | 30s |
| DMPC | No | Yes | Near | 20s |
| MADER | No | Yes | No (local) | 10s real-time |

---

## Further Reading

- Fiorini & Shiller — *Motion Planning in Dynamic Environments using Velocity Obstacles*, IJRR 1998
- van den Berg et al. — *Reciprocal n-Body Collision Avoidance*, ISRR 2011 (ORCA)
- Sharon et al. — *Conflict-Based Search for Optimal Multi-Agent Pathfinding*, AIJ 2015 (CBS)
- Tordesillas & How — *MADER: Trajectory Planner in Multi-Agent and Dynamic Environments*, T-RO 2022
- [mit-acl/mader](https://github.com/mit-acl/mader)
