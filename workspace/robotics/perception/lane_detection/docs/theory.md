# Lane Detection Theory

## Hough Transform

The Hough transform maps each edge point $(x, y)$ to a sinusoid in parameter space $(\rho, \theta)$:

$$\rho = x\cos\theta + y\sin\theta$$

Intersection of multiple sinusoids in $(\rho, \theta)$ space corresponds to a line in image space. The accumulator is a 2D histogram over discretized $(\rho, \theta)$; peaks above a threshold correspond to detected lines.

No OpenCV dependency: the accumulator is implemented as an `Eigen::MatrixXi`.

## Left/Right Lane Assignment

Line slope determines lane identity:
- **Negative slope** + line in **left image half** → left lane
- **Positive slope** + line in **right image half** → right lane

## Polynomial Fitting via RANSAC

Line sample points from Hough detections are fitted to a degree-$d$ polynomial $y = \sum_{k=0}^d a_k x^k$ using RANSAC least-squares:

1. Sample 3 points randomly
2. Fit polynomial via Eigen's `JacobiSVD`
3. Count inliers within residual threshold
4. Repeat for N iterations; keep best fit

## References

- Duda & Hart, "Use of the Hough Transformation to Detect Lines and Curves in Pictures," CACM 1972
- Fischler & Bolles, "Random Sample Consensus: A Paradigm for Model Fitting," CACM 1981
