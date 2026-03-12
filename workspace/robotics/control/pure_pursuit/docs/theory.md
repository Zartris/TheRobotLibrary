# Theory: Pure Pursuit

Pure pursuit is a **geometric path-tracking** controller. It requires no dynamic model of
the robot — it works purely from the robot's current pose and the path geometry.

---

## 1. The Lookahead Concept

The controller picks a **lookahead point** $G$ on the path: the point that lies exactly
distance $L_d$ ahead along the path from the closest point to the robot.

```
        path
    ----*-------G------*----
               / L_d
              /
           robot
```

It then computes the circular arc the robot must follow to reach $G$, and outputs the
angular velocity corresponding to that arc's curvature.

---

## 2. Geometric Derivation

Transform $G$ into the robot body frame: let $y_l$ be its lateral (left-right) offset and
$L_d$ be its distance. The radius $R$ of the arc connecting the robot to $G$ satisfies:

$$R = \frac{L_d^2}{2\, y_l}$$

Curvature $\kappa = 1/R$:

$$\kappa = \frac{2\, y_l}{L_d^2}$$

For a differential-drive robot moving at forward speed $v$, the angular velocity command is:

$$\omega = v \cdot \kappa = \frac{2\, v\, y_l}{L_d^2}$$

The forward speed $v$ is an external input (set by a speed scheduler or held constant).

---

## 3. Lookahead Distance $L_d$

$L_d$ is the single most important tuning parameter:

| $L_d$ small | $L_d$ large |
|---|---|
| Tracks path tightly | Cuts corners |
| Oscillates at speed | Smooth at speed |
| Slow convergence on curves | Fast progress along straights |

**Adaptive lookahead:** Scale $L_d$ with speed to get the best of both:

$$L_d = k \cdot v + L_{d,\min}$$

where $k$ is a tunable constant and $L_{d,\min}$ is a lower bound.

---

## 4. Lookahead Point Selection

When the robot is close to the end of the path, no point at distance $L_d$ exists.
Common strategies:
- **Clamp to the goal** — use the final path point if the path is shorter than $L_d$
- **Stop condition** — declare goal reached when the robot comes within a tolerance

---

## 5. Limitations

- Pure pursuit assumes the robot can instantaneously reach any curvature — it does not
  respect maximum angular acceleration or robot dynamics.
- For high-speed or dynamic environments, prefer MPC (which respects constraints explicitly)
  or combine pure pursuit with a speed controller that limits acceleration.

---

## Further Reading

- Coulter — *Implementation of the Pure Pursuit Path Tracking Algorithm*, CMU-RI-TR-92-01 (1992)
- Snider — *Automatic Steering Methods for Autonomous Automobile Path Tracking*, CMU-RI-TR-09-08 (2009)
