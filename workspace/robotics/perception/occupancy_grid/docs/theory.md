# Theory: Occupancy Grid Mapping

---

## 1. The Mapping Problem

Given a sequence of sensor measurements $\mathbf{z}_{1:t}$ and robot poses $\mathbf{x}_{1:t}$,
estimate the map $\mathbf{m}$ — specifically, whether each cell is occupied or free.

Occupancy grid mapping decomposes the map into independent cells and estimates each one
separately: $p(m_i \mid \mathbf{z}_{1:t}, \mathbf{x}_{1:t})$.

---

## 2. Binary Bayes Filter per Cell

Each cell $m_i$ is a binary random variable (occupied = 1, free = 0).
Apply Bayes' rule for each new measurement:

$$p(m_i \mid \mathbf{z}_{1:t}) = \frac{p(\mathbf{z}_t \mid m_i)\, p(m_i \mid \mathbf{z}_{1:t-1})}{p(\mathbf{z}_t \mid \mathbf{z}_{1:t-1})}$$

---

## 3. Log-Odds Representation

Direct probability multiplication suffers numerical underflow. Use **log-odds** instead:

$$l(m_i) = \log \frac{p(m_i = 1)}{1 - p(m_i = 1)}$$

The update rule becomes an addition:

$$l(m_i \mid \mathbf{z}_{1:t}) = l(m_i \mid \mathbf{z}_{1:t-1}) + \underbrace{l(\mathbf{z}_t \mid m_i)}_{\text{inverse sensor model}} - l_0$$

where $l_0 = \log\frac{p_0}{1-p_0}$ is the log-odds prior (typically $l_0 = 0$, i.e. $p_0 = 0.5$).

Recover probability: $p = 1 - \frac{1}{1 + e^l}$

---

## 4. Inverse Sensor Model

The inverse sensor model $l(\mathbf{z}_t \mid m_i)$ defines how a measurement updates each cell:

- **Cells the beam passes through (free):** subtract $l_{\text{free}}$ (typically $-0.4$)
- **Cell the beam terminates in (hit = occupied):** add $l_{\text{occ}}$ (typically $+0.85$)
- **Cells beyond the hit:** no update

The beam path through the grid is traced with **Bresenham's line algorithm** for efficiency.

---

## 5. Clamping

Without clamping, the log-odds can grow without bound in highly observed areas,
making the map rigid and unable to unlearn old information:

$$l(m_i) \leftarrow \text{clamp}(l(m_i),\; l_{\min},\; l_{\max})$$

Typical values: $l_{\min} = -2.0$, $l_{\max} = 3.5$.

---

## 6. Map Inflation

For path planning, expand occupied cells by the robot's circumradius $r$:
any cell within distance $r$ of an occupied cell is also marked as occupied (or inflated).
This allows the path planner to treat the robot as a point.

$$\text{inflated}(i) = 1 \quad \text{if } \exists j : m_j = 1 \text{ and } d(i,j) \le r$$

---

## Further Reading

- Elfes — *Using Occupancy Grids for Mobile Robot Perception and Navigation*, 1989
- Thrun, Burgard, Fox — *Probabilistic Robotics*, Chapter 9
