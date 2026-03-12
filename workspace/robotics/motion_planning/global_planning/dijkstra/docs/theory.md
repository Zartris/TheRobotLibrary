# Theory: Dijkstra's Algorithm

See [`global_planning/docs/theory.md`](../docs/theory.md) for the shared graph search
framework. This file covers Dijkstra-specific material and motivates when to use
Dijkstra over A\*.

---

## 1. Algorithm

Dijkstra's algorithm finds shortest paths from a source node to **all** reachable nodes
in a graph with **non-negative edge weights**:

```
OPEN ← priority queue, ordered by g(n)
g(start) = 0; g(all others) = ∞
OPEN.insert(start, 0)

while OPEN not empty:
    n ← OPEN.pop_min()       // extract minimum-distance node
    for each neighbor m of n:
        tentative_g = g(n) + cost(n, m)
        if tentative_g < g(m):
            g(m) = tentative_g
            parent(m) = n
            OPEN.insert_or_update(m, g(m))
```

This is A\* with $h(n) = 0$ — no heuristic. It is **complete and optimal** for any
graph with non-negative weights.

---

## 2. Dijkstra vs. A\*

| | Dijkstra | A\* |
|---|---|---|
| Heuristic | None | $h(n)$ |
| Expands nodes | In order of $g(n)$ (distance from start) | In order of $f(n) = g(n) + h(n)$ |
| Explored area | Full disk from start | Directed toward goal |
| Single source → all targets | Optimal (run to completion) | Re-run per goal |
| Single source → one target | Correct but wasteful | More efficient |
| Use case | Pre-compute distance maps, potential fields | Point-to-point queries |

**Key insight:** Dijkstra is preferred when you need the **distance to many goals at once**
(e.g., a distance-to-obstacle map, or a value function over a grid).

---

## 3. Multi-Source Dijkstra

Initialize the priority queue with all source nodes at cost 0. Dijkstra then computes
the distance from the nearest source to every other node:

```cpp
for (const auto& source : sources)
    pq.push({source, 0});
```

**Application — distance-to-obstacle map:** set all occupied cells as sources. The
resulting distance map gives, for each free cell, the clearance to the nearest obstacle.
Used to inflate obstacles and in DWA's obstacle cost term.

---

## 4. Complexity

- With a binary heap: $O((V + E) \log V)$
- With a Fibonacci heap: $O(V \log V + E)$ (theoretically optimal; rarely implemented)
- For a 4-connected grid with $M \times N$ cells: $O(MN \log(MN))$

---

## Further Reading

- Dijkstra — *A Note on Two Problems in Connexion with Graphs*, Numerische Mathematik 1959
- Johnson — *Efficient Algorithms for Shortest Paths in Sparse Networks*, JACM 1977
