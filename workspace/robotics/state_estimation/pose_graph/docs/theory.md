# Pose Graph Optimization Theory

## 1. Problem Formulation

A pose graph is a graph $G = (V, E)$ where:
- Nodes $V$ = robot poses $\{x_i\} \in SE(2)$
- Edges $E$ = relative-pose constraints $(i, j, \tilde{z}_{ij}, \Omega_{ij})$ where
  $\tilde{z}_{ij}$ is the measured relative transform and $\Omega_{ij} = \Sigma_{ij}^{-1}$
  is the information matrix.

The optimization finds the node positions $\{x_i^*\}$ minimising the total weighted
residual:
$$F(x) = \sum_{(i,j) \in E} e_{ij}^T \Omega_{ij} \, e_{ij}$$

where $e_{ij} = \tilde{z}_{ij} \ominus (x_j \ominus x_i)$ is the error between the
measured and predicted relative transform.

## 2. SE2 Error Residual

For SE2 poses $x = (t_x, t_y, \theta)$, the relative transform prediction is:
$$\hat{z}_{ij} = x_i^{-1} \oplus x_j$$

The residual $e_{ij}$ has three components: $\Delta x, \Delta y, \Delta \theta$.

## 3. Gauss-Newton Update

The Gauss-Newton method linearises the error around the current estimate:
$$H \Delta x = -b$$

where $H = \sum J_{ij}^T \Omega_{ij} J_{ij}$ (information matrix) and
$b = \sum J_{ij}^T \Omega_{ij} e_{ij}$ (gradient), with $J_{ij}$ the Jacobian of $e_{ij}$
w.r.t. the node poses. The linear system is solved via Eigen's SparseLU decomposition.

## 4. Gauge Freedom

The system is underdetermined without fixing a reference frame. Node 0 is anchored
(fixed during optimization) to resolve the gauge freedom.

## 5. Loop Closure Effect

Without loop closures, the graph forms a chain and error accumulates unboundedly.
Adding a loop closure edge creates a cycle: the optimizer redistributes accumulated
error across all nodes, correcting drift.

## 6. Levenberg-Marquardt Damping

When the Gauss-Newton step overshoots or diverges, LM adds a damping term $\lambda I$
to the Hessian: $(H + \lambda I) \Delta x = -b$. $\lambda$ is increased on divergence
and decreased (toward GN) on improvement.
