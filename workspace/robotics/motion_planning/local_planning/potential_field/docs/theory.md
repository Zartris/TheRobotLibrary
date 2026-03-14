# Potential Field Theory

## Artificial Potential Fields

The robot is modelled as a particle moving under the influence of two superimposed potential fields:

$$U(\mathbf{q}) = U_{att}(\mathbf{q}) + U_{rep}(\mathbf{q})$$

The robot follows the negative gradient: $\mathbf{F} = -\nabla U(\mathbf{q})$.

## Attractive Field

$$U_{att}(\mathbf{q}) = \begin{cases} \frac{1}{2} k_{att} \|\mathbf{q} - \mathbf{q}_{goal}\|^2 & \text{if } d \leq d_{switch} \\ k_{att} d_{switch} \cdot d - \frac{1}{2} k_{att} d_{switch}^2 & \text{if } d > d_{switch} \end{cases}$$

The conic part beyond $d_{switch}$ prevents the attractive force from growing without bound far from the goal.

## Repulsive Field

For each obstacle at distance $d$ (from `OccupancyGrid` nearest-obstacle lookup):

$$U_{rep}(\mathbf{q}) = \begin{cases} \frac{1}{2} k_{rep} \left(\frac{1}{d} - \frac{1}{d_0}\right)^2 & \text{if } d \leq d_0 \\ 0 & \text{if } d > d_0 \end{cases}$$

## Local Minima

A local minimum occurs when $\nabla U = 0$ but $\mathbf{q} \neq \mathbf{q}_{goal}$. Detection: robot velocity below threshold for $N$ consecutive steps. Escape: apply a random perturbation to break the symmetry.

## References

- Khatib, "Real-Time Obstacle Avoidance for Manipulators and Mobile Robots," IJRR, 1986
- Koditschek & Rimon, "Robot Navigation Functions on Manifolds with Boundary," Advances in Applied Mathematics, 1990
