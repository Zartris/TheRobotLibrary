# Theory: Conflict-Based Search (CBS)

---

## 1. Motivation

The **joint state space** for $N$ robots on a grid of size $|S|$ is $|S|^N$.
Even for 10 robots on a 100-node graph, this is $100^{10} = 10^{20}$ states.
Planning in the joint space is infeasible.

CBS exploits the fact that **most robots do not interact** — their individual plans
are already collision-free. Only a small number of conflicts need resolution.

---

## 2. Two-Level Structure

**High level:** explores a **constraint tree** (CT). Each CT node $n$ stores:
- $n.\text{constraints}$: a set of constraints $(i, v, t)$ — "robot $i$ cannot be at
  position $v$ at time $t$"
- $n.\text{solution}$: a per-agent path consistent with $n.\text{constraints}$
- $n.\text{cost}$: sum of individual path lengths (SIC — sum of individual costs)

**Low level:** re-plans for a single agent under a given constraint set.
Uses space-time A\* on a 3D graph $(x, y, t)$, where the temporal dimension
encodes waiting and the constraints prune invalid states.

---

## 3. CBS Algorithm

```
Root: solve each agent independently (A* with no constraints); CT.insert(root)

while CT not empty:
    n ← CT.pop_min_cost()
    validate n.solution: find earliest conflict c = (i, j, v, t)
    if no conflict: return n.solution  ← OPTIMAL SOLUTION
    
    // Branch on who gets constrained
    for agent a in {i, j}:
        child = new CT node
        child.constraints = n.constraints ∪ {(a, v, t)}
        replan for agent a with child.constraints
        if solution found: CT.insert(child)
```

**Branching factor:** 2 at each CT node (constrain agent $i$ or agent $j$).
**Optimality proof:** the root is the globally cheapest un-conflict-checked solution;
branching only adds constraints (increases cost or keeps equal); the first conflict-free
solution found is thus globally optimal.

---

## 4. Conflict Types

**Vertex conflict:** agents $i$ and $j$ are both at grid cell $v$ at time $t$.
Resolved by adding constraint $(i, v, t)$ or $(j, v, t)$.

**Edge conflict:** agents $i$ and $j$ swap positions between $t$ and $t+1$
(they pass through each other). Resolved by adding constraints on the edge traversal.

**Following conflict:** agent $j$ follows agent $i$ but the trailing agent would be
at a cell the leading agent just vacated at the same timestep. Handled by ensuring
the trailing agent waits.

---

## 5. Space-Time A\*

The low-level planner runs A\* on a graph where each node is $(x, y, t)$ and edges
represent:
- Move to adjacent cell: $(x', y', t+1)$
- Wait in place: $(x, y, t+1)$

Constraint $(a, v, t)$ prunes node $(v, t)$ from agent $a$'s search.

**Heuristic:** $h = $ manhattan distance to goal (independent of $t$ — admissible).

---

## 6. Enhanced CBS (ECBS)

CBS explores the CT in order of **exact** optimal cost — can be slow when many similar-cost
nodes exist. ECBS uses a **focal search**:

- **OPEN:** all CT nodes, ordered by SIC cost $f$
- **FOCAL:** CT nodes with $f \le w \cdot f_{\min}$ (within sub-optimality factor $w$)
- Pop from FOCAL by a secondary heuristic: **number of conflicts** in the solution (prefer fewer conflicts → faster resolution)

ECBS guarantees solutions within factor $w$ of optimal. For $w = 1.2$, it is typically
10–100× faster than optimal CBS with only 20% cost increase.

---

## 7. Complexity

| | CBS | ECBS ($w$-suboptimal) |
|---|---|---|
| Time (worst case) | Exponential in $N$ | Polynomial for small $w$ |
| Time (typical, sparse) | Polynomial | Much faster |
| Solution quality | Optimal | $\le w \cdot$ optimal |

In practice, CBS + ECBS is the state of the art for $N \le 50$ robots in warehouse MAPF.

---

## Further Reading

- Sharon et al. — *Conflict-Based Search for Optimal Multi-Agent Pathfinding*, AIJ 2015
- Barer et al. — *Suboptimal Variants of the Conflict-Based Search Algorithm*, SoCS 2014 (ECBS)
- Li et al. — *EECBS: A Bounded-Suboptimal Search for Multi-Agent Path Finding*, AAAI 2021
- Stern et al. — *Multi-Agent Pathfinding: Definitions, Variants, and Benchmarks*, SoCS 2019
