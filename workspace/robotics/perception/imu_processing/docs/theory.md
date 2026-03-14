# IMU Processing Theory

## 1. IMU Measurement Model

An IMU provides two raw measurements at each timestep $dt$:
- **Accelerometer:** $\tilde{a} = a + b_a + n_a$ (true linear acceleration + bias + noise)
- **Gyroscope:** $\tilde{\omega} = \omega + b_g + n_g$ (true angular rate + bias + noise)

Bias terms $b_a$, $b_g$ are modelled as random walks driven by slowly varying noise.

## 2. Complementary Filter

The complementary filter fuses gyro integration (accurate short-term) with accelerometer
gravity direction (accurate long-term) to estimate orientation:
$$\hat{R}_{k+1} = \hat{R}_k \cdot \text{Exp}((\tilde{\omega} - \hat{b}_g) \cdot dt)$$
$$\text{correction} = \alpha \cdot (\hat{R}_k^T g_{meas} \times g_{ref})$$

where $\alpha \in (0,1)$ is the complementary filter gain. Smaller $\alpha$ trusts the
gyro more; larger $\alpha$ applies stronger gravity correction.

## 3. Bias Estimation

Gyro bias is estimated via exponential moving average of the orientation correction signal.
Accelerometer bias is estimated from the residual gravity magnitude error. Both converge
under static or slow-motion conditions.

## 4. IMU Pre-Integration

Between two keyframe times $t_i$ and $t_j$, the pre-integration accumulates:
$$\Delta R_{ij} = \prod_{k=i}^{j-1} \text{Exp}((\tilde{\omega}_k - b_g) \cdot dt_k)$$
$$\Delta v_{ij} = \sum_{k=i}^{j-1} \Delta R_{ik} \cdot (\tilde{a}_k - b_a) \cdot dt_k$$
$$\Delta p_{ij} = \sum_{k=i}^{j-1} \left[\Delta v_{ik} \cdot dt_k + \frac{1}{2}\Delta R_{ik} \cdot (\tilde{a}_k - b_a) \cdot dt_k^2\right]$$

Pre-integration avoids re-integration when the bias estimate changes; instead, Jacobians
$\partial \Delta R / \partial b_g$, $\partial \Delta v / \partial b_a$, etc. enable
first-order bias correction without full re-integration.

## 5. Why IMU Pre-Integration for VIO

Visual odometry runs at keyframe rate (e.g., 10 Hz), while IMU runs at 100–1000 Hz.
Pre-integration collapses the high-rate IMU stream into a single compact constraint
$({\Delta R, \Delta v, \Delta p})$ that can be directly used as the IMU motion prior in
the VIO estimator at keyframe rate.
