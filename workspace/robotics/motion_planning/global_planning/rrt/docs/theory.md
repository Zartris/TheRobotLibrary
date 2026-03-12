# Theory: RRT and RRT*

See [`global_planning/docs/theory.md`](../docs/theory.md) for an overview of all
global planners and where sampling-based methods fit. This file covers the RRT
and RRT\* algorithms in detail, including convergence properties and practical extensions.

---

## 1. RRT (Rapidly-exploring Random Tree)

RRT incrementally builds a tree by sampling configuration space randomly:

```
Tree T ← {q_start}

for i = 1 to N:
    q_rand ← random_config()                     // sample C-space uniformly
    q_near ← nearest_neighbor(T, q_rand)          // closest node in T
    q_new  ← steer(q_near, q_rand, step_size)     // move toward q_rand
    if collision_free(q_near, q_new):
        T.add_node(q_new)
        T.add_edge(q_near, q_new)
        if reached(q_new, q_goal):
            return path_to_root(q_new)
```

**`steer`:** moves from $q_{\text{near}}$ toward $q_{\text{rand}}$ by at most `step_size`.

**Probabilistic completeness:** as $N \to \infty$, the probability that RRT finds a solution
(if one exists) approaches 1. The tree's Voronoi bias — larger cells attract more samples —
causes rapid exploration.

**Not optimal:** RRT finds *a* solution, not the shortest one. The solution path is jagged
and significantly longer than the true shortest path.

---

## 2. RRT* (Asymptotically Optimal)

RRT\* adds two operations to RRT that ensure the solution improves toward optimal as
$N \to \infty$:

**Near-neighbor set:**
$$Q_{\text{near}} = \{q \in T : \|q - q_{\text{new}}\| \le r(N)\}$$

where $r(N) = \gamma \left(\frac{\log N}{N}\right)^{1/d}$ shrinks with $N$.

**Choose parent (rewire incoming):**
$$q_{\text{parent}} = \arg\min_{q \in Q_{\text{near}}} \left[c(q) + c(q, q_{\text{new}})\right]$$

Choose the neighbor that minimizes the cost-from-root through $q_{\text{new}}$.

**Rewire tree (fix outgoing):**
For each $q_{\text{near}} \in Q_{\text{near}}$:
$$\text{if } c(q_{\text{new}}) + c(q_{\text{new}}, q_{\text{near}}) < c(q_{\text{near}}):$$
$$\quad \text{replace parent}(q_{\text{near}}) \leftarrow q_{\text{new}}$$

This propagates cost improvements through the tree.

**Asymptotic optimality:** $c^*(N) \to c^*_{\text{true}}$ as $N \to \infty$ — the path
cost converges to the true shortest path.

---

## 3. Goal Biasing

Uniform sampling is inefficient near the goal. A fraction $p_g$ (typically 5%) of
samples are set to $q_{\text{goal}}$ directly:

```cpp
if (uniform_real(0, 1) < p_goal)
    q_rand = q_goal;
```

This Biases the tree toward the goal for faster convergence in practice.

---

## 4. Bidirectional RRT

Grow two trees simultaneously: one from $q_{\text{start}}$, one from $q_{\text{goal}}$.
At each step, attempt to connect the two trees. When the trees meet, the solution is the
concatenation of both partial paths.

Reduces planning time by $O(\sqrt{})$ in high-dimensional spaces.

---

## 5. RRT for Kinodynamic Planning

In the kinematic RRT above, `steer` produces a straight-line segment. For a robot
with dynamics $\dot{x} = f(x, u)$, `steer` instead **propagates the dynamics**:
integrate $f(x, u)$ with a random or optimized control $u$ for a random duration.

This allows planning for diff-drive robots, car-like robots, and manipulators with
velocity/acceleration constraints — without manual constraint handling.

---

## 6. Practical Comparison: A\* vs. RRT\*

| | A\* | RRT\* |
|---|---|---|
| Space | Discrete grid | Continuous C-space |
| Dimensionality | 2D (scales poorly) | Up to ~10D practical |
| Solution quality | Optimal (grid resolution) | Asymptotically optimal |
| Kinodynamic | Requires lattice planner | Natural extension |
| Speed (2D grid) | Fast | Slower (many collision checks) |
| Use for | 2D occupancy grids | High-DOF or continuous spaces |

---

## Further Reading

- LaValle — *Rapidly-Exploring Random Trees: A New Tool for Path Planning*, TR 1998 (original RRT)
- Karaman & Frazzoli — *Sampling-Based Algorithms for Optimal Motion Planning*, IJRR 2011 (RRT\*)
- LaValle — *Planning Algorithms*, Cambridge 2006 (free online: <http://planning.cs.uiuc.edu/>)
