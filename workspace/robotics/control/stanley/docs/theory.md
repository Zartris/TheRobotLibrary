# Stanley Controller Theory

## 1. Problem Statement

Given a reference path and the robot's current pose (position + heading), compute a steering
angle $\delta$ that drives both the cross-track error (CTE) and heading error to zero.

## 2. Stanley Control Law

The steering command is the sum of two terms:
$$\delta = \psi_e + \arctan\!\left(\frac{k \cdot e_{cte}}{v + \varepsilon}\right)$$

- $\psi_e$ — heading error: angle between robot heading and path tangent at the closest point
- $e_{cte}$ — cross-track error: signed lateral distance from the front axle to the path
- $k$ — Stanley gain (tuned per vehicle)
- $v$ — forward speed of the front axle
- $\varepsilon$ — small softening constant to avoid division by zero at $v \approx 0$

## 3. Heading Error Term

$\psi_e = \theta_{path} - \theta_{robot}$ corrects for orientation misalignment. This is
the dominant term at high speed; the CTE term becomes small as $v \to \infty$.

## 4. Cross-Track Error Term

$\arctan(k \cdot e_{cte} / v)$ provides speed-normalized lateral correction. At low speed,
the CTE term dominates; at high speed, the heading term dominates. The $1/v$ normalization
naturally reduces the aggressiveness of lateral correction at highway speeds.

## 5. Stability

For small errors, the Stanley controller can be shown to be exponentially stable for constant
path curvature. The gain $k$ controls the rate of CTE decay: larger $k$ → faster convergence
but potentially oscillatory response.

## 6. Reversing

When $v < 0$ (reversing), the heading error term must account for the flipped direction
of travel. The implementation negates $\psi_e$ and $e_{cte}$ sign conventions accordingly.
