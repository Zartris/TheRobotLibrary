# PRM Theory

## 1. Problem Statement

Given a configuration space $C$ (2D grid here), an obstacle-free region $C_{free}$, a
start $q_s$, and a goal $q_g$, find a collision-free path. PRM is a multi-query planner:
it amortises the roadmap construction cost across many queries.

## 2. Learning Phase (Roadmap Construction)

1. **Sample** $N$ uniformly random configurations in $C_{free}$.
2. For each sample $q$, find its $k$ nearest neighbours in the current roadmap.
3. For each neighbour $q'$, attempt to connect $q \to q'$ with a straight-line local planner.
4. If the straight line is collision-free (checked against `OccupancyGrid`), add the edge.

The result is a sparse graph approximating the connectivity of $C_{free}$.

## 3. Query Phase

Given $q_s$ and $q_g$:
1. Connect $q_s$ and $q_g$ to the roadmap via $k$-nearest collision-free edges.
2. Run Dijkstra (or A\*) on the roadmap graph.
3. The shortest roadmap path is the answer; disconnect from roadmap after query.

## 4. Probabilistic Completeness

PRM is probabilistically complete: as $N \to \infty$, the probability of finding a path
(if one exists) approaches 1. The rate of convergence depends on the clearance of the
passage and the sampling density.

## 5. Multi-Query Advantage

Unlike RRT (single-query), PRM builds a roadmap once and answers many queries cheaply.
This makes PRM ideal for environments that are known in advance (offline phase) and
require frequent re-planning to new goals.

## 6. Local Planner

The straight-line local planner checks collision by marching along the segment at
resolution $r \leq$ grid cell size. The collision check uses `OccupancyGrid::isOccupied()`
with inflation radius to provide clearance.

## 7. k-NN Graph Density

Larger $k$ → denser graph → shorter paths but higher construction cost. The standard
choice is $k = \ln(N)$ for asymptotic optimality (PRM*).
