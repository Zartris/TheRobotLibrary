# Factor Graph Theory

## Factor Graph Representation

A **factor graph** $\mathcal{G} = (\mathcal{X}, \mathcal{F}, \mathcal{E})$ is a bipartite graph with:
- **Variable nodes** $\mathcal{X}$: unknown quantities to be estimated (poses, landmarks, biases)
- **Factor nodes** $\mathcal{F}$: probabilistic constraints (odometry, observations, priors)
- **Edges** $\mathcal{E}$: connecting each factor to the variables it involves

The joint probability factorizes as:

$$p(\mathcal{X}) \propto \prod_{f \in \mathcal{F}} f(\mathcal{X}_f)$$

Maximum a posteriori (MAP) inference corresponds to minimizing the negative log-likelihood:

$$\mathbf{x}^* = \arg\min_{\mathbf{x}} \sum_{f \in \mathcal{F}} \| \mathbf{r}_f(\mathbf{x}_f) \|^2_{\Omega_f}$$

## Gauss-Newton Optimization

At each iteration, linearize each factor residual $\mathbf{r}_f(\mathbf{x})$ around the current estimate $\hat{\mathbf{x}}$:

$$\mathbf{r}_f(\mathbf{x}) \approx \mathbf{r}_f(\hat{\mathbf{x}}) + J_f \Delta\mathbf{x}$$

The normal equations are:

$$\underbrace{J^T \Omega J}_{H} \Delta\mathbf{x} = -J^T \Omega \mathbf{r}$$

where $H$ is the sparse information matrix. Solved via Eigen `SimplicialLDLT` (sparse Cholesky).

## Variable Types

| Type | Description | DOF |
|------|-------------|-----|
| `PoseVariable` | SE2 pose $(x, y, \theta)$ | 3 |
| `LandmarkVariable` | 2D landmark $(x, y)$ | 2 |
| `BiasVariable` | IMU accelerometer + gyroscope bias | 6 |

## Factor Types

| Type | Variables | Residual |
|------|-----------|---------|
| `OdometryFactor` | pose $i$, pose $j$ | $\mathbf{r} = \ominus \hat{T}_{ij} \oplus T_j^{-1} T_i$ |
| `ObservationFactor` | pose $i$, landmark $l$ | $\mathbf{r} = z_{il} - h(\hat{T}_i, \hat{l})$ |
| `PriorFactor` | any variable | $\mathbf{r} = \hat{x} - x_{prior}$ |

## References

- Dellaert & Kaess, "Factor Graphs for Robot Perception," Foundations and Trends in Robotics, 2017
- Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using the Bayes Tree," IJRR 2012
