# Theory: A* Search

See [`global_planning/docs/theory.md`](../docs/theory.md) for the shared graph search
framework (open/closed sets, priority queue, relaxation) and the general admissible
heuristic requirements.

This file covers A\*-specific material: heuristic design and implementation details.

---

## 1. Algorithm

A\* is Dijkstra's algorithm with a **heuristic** $h(n)$ added to the priority key:

$$f(n) = g(n) + h(n)$$

- $g(n)$: exact cost from start to $n$
- $h(n)$: estimated cost from $n$ to goal
- $f(n)$: estimated total cost of the cheapest path through $n$

```
OPEN ← priority queue, ordered by f(n)
OPEN.insert(start, f=h(start))
CLOSED ← {}

while OPEN not empty:
    n ← OPEN.pop_min()
    if n == goal: return reconstruct_path(n)
    CLOSED.add(n)
    for each neighbor m of n:
        tentative_g = g(n) + cost(n, m)
        if m in CLOSED and tentative_g >= g(m): continue
        if tentative_g < g(m) or m not in OPEN:
            g(m) = tentative_g
            f(m) = g(m) + h(m)
            parent(m) = n
            OPEN.insert_or_update(m, f(m))
```

---

## 2. Heuristic Admissibility and Consistency

**Admissible:** $h(n) \le h^*(n)$ (never overestimates). Guarantees optimality.

**Consistent (monotone):** $h(n) \le c(n, m) + h(m)$ for every edge $(n,m)$.
Consistency implies admissibility. With a consistent heuristic, every node is expanded
at most once → CLOSED set can be used safely.

---

## 3. Grid Heuristics

| Connectivity | Heuristic | Formula |
|---|---|---|
| 4-connected | Manhattan | $|dx| + |dy|$ |
| 8-connected | Octile | $\max(|dx|, |dy|) + (\sqrt{2}-1)\min(|dx|,|dy|)$ |
| Any (metric) | Euclidean | $\sqrt{dx^2 + dy^2}$ |

For a uniform-cost 8-connected grid: octile distance is the exact $h^*$ → A\* expands
only cells on the optimal path.

---

## 4. Weighted A\* (Suboptimal but Faster)

$$f(n) = g(n) + w \cdot h(n), \quad w > 1$$

Finds a solution within factor $w$ of optimal, often expanding exponentially fewer nodes.
Standard in practice: $w = 1.2$–$3.0$ depending on map complexity.

---

## 5. Implementation Notes

- **Priority queue:** `std::priority_queue` with a tie-breaking comparator (prefer
  higher $g$ on equal $f$ — expands closer to goal first, reducing corridor elongation).
- **Grid representation:** flat array + index math is faster than `std::unordered_map`
  for grid maps (cache coherence).
- **Path smoothing:** the raw A\* path follows grid edges. Apply `global_planning/`
  post-processing (string pulling, spline fitting) before velocity profiling.

---

## Further Reading

- Hart, Nilsson & Raphael — *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*, 1968 (original A\* paper)
- Harabor & Grastien — *Online Graph Pruning for Pathfinding on Grid Maps*, AAAI 2011 (JPS — A\* speedup)
