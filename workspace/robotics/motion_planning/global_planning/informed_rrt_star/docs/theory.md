# Informed RRT* Theory

## Standard RRT*

RRT* is an asymptotically optimal extension of RRT. At each iteration:

1. Sample $\mathbf{x}_{rand}$ uniformly from $\mathcal{X}_{free}$
2. Find nearest tree node $\mathbf{x}_{nearest}$; steer toward $\mathbf{x}_{rand}$ to get $\mathbf{x}_{new}$
3. Choose the parent that minimizes cost-to-come within rewire radius $r$
4. Rewire the local neighbourhood to use $\mathbf{x}_{new}$ if it offers a cheaper path

**Asymptotic optimality:** As $n \to \infty$, the solution converges to the optimal path almost surely.

## Prolate Hyperspheroid Sampling

Once a feasible path with cost $c_{best}$ is found, all optimal improvements must lie inside the prolate hyperspheroid (PHS):

$$\mathcal{X}_{PHS} = \{ \mathbf{x} : \| \mathbf{x} - \mathbf{x}_{start} \| + \| \mathbf{x} - \mathbf{x}_{goal} \| \leq c_{best} \}$$

Sampling within the PHS is achieved by the transformation:

$$\mathbf{x}_{sample} = \mathbf{C} \cdot \text{diag}(r_1, r_2, \ldots, r_n) \cdot \mathbf{x}_{ball} + \mathbf{x}_{centre}$$

where $r_1 = c_{best}/2$, $r_2 = \cdots = r_n = \sqrt{c_{best}^2 - c_{min}^2}/2$, $\mathbf{C}$ is the rotation matrix aligning the start-goal axis with the first coordinate axis, and $\mathbf{x}_{ball}$ is a uniform sample in the unit ball.

## References

- Gammell et al., "Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic," IROS 2014
- Karaman & Frazzoli, "Sampling-based Algorithms for Optimal Motion Planning," IJRR 2011
