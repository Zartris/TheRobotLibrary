# 3D Object Detection Theory

## 3D DBSCAN Clustering

DBSCAN (Density-Based Spatial Clustering of Applications with Noise) groups points by density reachability. A point $p$ is a **core point** if at least `min_points` points lie within Euclidean distance ε. Clusters are formed by recursively collecting density-reachable points.

For 3D point clouds, ε-neighbourhood queries are accelerated by a KD-tree.

**Complexity:** $O(n \log n)$ with KD-tree; $O(n^2)$ brute force.

## PCA Oriented Bounding Box

For a cluster of points $\{p_i\}$, compute the covariance matrix:

$$\Sigma = \frac{1}{N} \sum_{i=1}^{N} (p_i - \bar{p})(p_i - \bar{p})^T$$

The eigenvectors of $\Sigma$ (via `SelfAdjointEigenSolver`) form the box axes. The eigenvalues correspond to variance along each axis, from which dimensions are derived as $2\sqrt{\lambda_j}$.

## Size-Based Classification

| Class | Condition |
|-------|-----------|
| `PERSON` | height > 1.0 m AND footprint < 0.5 × 0.5 m |
| `CAR` | footprint > 1.5 × 1.5 m |
| `UNKNOWN` | otherwise |

## References

- Ester et al., "A Density-Based Algorithm for Discovering Clusters in Large Spatial Databases," KDD 1996
- Rusu, "Semantic 3D Object Maps for Everyday Manipulation," PhD Thesis, TU Munich, 2009
