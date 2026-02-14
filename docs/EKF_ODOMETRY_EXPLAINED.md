# EKF Odometry - Complete Mathematical Explanation

## Table of Contents
1. [Overview](#overview)
2. [State Vector and Covariance](#state-vector-and-covariance)
3. [Differential Drive Kinematics](#differential-drive-kinematics)
4. [Extended Kalman Filter Theory](#extended-kalman-filter-theory)
5. [Prediction Step](#prediction-step)
6. [Update Step 1: Wheel Measurement](#update-step-1-wheel-measurement)
7. [Update Step 2: IMU Orientation](#update-step-2-imu-orientation)
8. [Process Noise Model (Thrun's Alpha)](#process-noise-model-thruns-alpha)
9. [Mahalanobis Distance Gating](#mahalanobis-distance-gating)
10. [Joseph Form Covariance Update](#joseph-form-covariance-update)
11. [Complete Algorithm Flow](#complete-algorithm-flow)
12. [Implementation Details](#implementation-details)

---

## Overview

### What is EKF Odometry?

**Extended Kalman Filter (EKF)** is a probabilistic state estimation algorithm that:
- Fuses multiple noisy sensor measurements
- Estimates the robot's pose (position and orientation)
- Tracks uncertainty in the estimate
- Handles nonlinear motion models

### Sensor Fusion Strategy

1. **Wheel Encoders** (`/joint_states`): Good for position, but heading drifts
2. **IMU** (`/imu`): Good for heading rate, but has bias

The EKF optimally combines both to get better estimates than either alone!

### Why Estimate Gyro Bias?

IMU gyroscopes have a **bias** (constant offset) that changes slowly over time:
```
ω_measured = ω_true + bias + noise
```

If this bias is not estimated, it integrates into the heading estimate and causes drift:
```
After 60 seconds with 0.01 rad/s bias:
θ_error = 0.01 × 60 = 0.6 rad = 34.4 degrees!
```

By estimating the bias online, subtract it out and get much better heading estimates.

---

## State Vector and Covariance

### State Vector

The state has **4 dimensions**:

```python
x = [x, y, θ, b_g]ᵀ
```

Where:
- **x, y**: Robot position in meters (in the `odom` frame)
- **θ**: Robot heading in radians (0 = facing East, positive = counter-clockwise)
- **b_g**: Gyroscope bias in rad/s (how much the IMU over-reports rotation)

### Covariance Matrix

The **covariance matrix P** (4×4) represents uncertainty:

```
P = [σ_x²      σ_xy     σ_xθ     σ_x,bg  ]
    [σ_xy      σ_y²     σ_yθ     σ_y,bg  ]
    [σ_xθ      σ_yθ     σ_θ²     σ_θ,bg  ]
    [σ_x,bg    σ_y,bg   σ_θ,bg   σ_bg²   ]
```

**Diagonal elements**: Variance (uncertainty) in each state
- `P[0,0] = σ_x²`: How uncertain about x position
- `P[2,2] = σ_θ²`: How uncertain about heading

**Off-diagonal elements**: Correlation between states
- `P[2,3] = σ_θ,bg`: Correlation between heading and bias
  - This is KEY! When measuring heading, it tells about bias too!

### Initial Values

```python
# Start at origin with some uncertainty
state = [0.0, 0.0, 0.0, 0.0]

# Initial covariance
P = [[0.01,  0,     0,     0    ],
     [0,     0.01,  0,     0    ],
     [0,     0,     0.01,  0    ],
     [0,     0,     0,     0.0001]]
```

Fairly confident about starting position, but less sure about initial bias.

---

## Differential Drive Kinematics

### From Wheel Encoders to Velocities

**Step 1: Read wheel positions** (in radians)
```python
left_pos, right_pos = read_encoders()  # Current positions
delta_left = left_pos - prev_left_pos   # Change since last time
delta_right = right_pos - prev_right_pos
```

**Step 2: Convert to linear velocities**

When a wheel rotates by angle `Δθ`, it travels distance `s = r × Δθ`:
```python
v_left = (delta_left * wheel_radius) / dt
v_right = (delta_right * wheel_radius) / dt
```

Example:
- Wheel rotates 0.1 radians in 0.05 seconds
- Wheel radius = 0.033 m
- Linear velocity = (0.1 × 0.033) / 0.05 = 0.066 m/s

### Robot Center Velocities

**Linear velocity** (forward/backward):
```python
v = (v_right + v_left) / 2.0
```

**Why average?** The robot's center is midway between the wheels, so it moves at their average speed.

**Angular velocity** (rotation):
```python
ω = (v_right - v_left) / wheel_base
```

**Derivation:**

Consider a robot turning in an arc:
```
        ● ← Right wheel (moving faster)
        |
    ----+---- Robot center (turning around this point)
        |
        ● ← Left wheel (moving slower)

    |<- L ->|  (L = wheel_base)
```

Both wheels travel around circles with the same angular velocity `ω` but different radii:
- Left wheel: `v_L = ω × (R - L/2)`
- Right wheel: `v_R = ω × (R + L/2)`

Subtract:
```
v_R - v_L = ω(R + L/2) - ω(R - L/2) = ωL
```

Therefore:
```
ω = (v_R - v_L) / L
```

**Examples:**

| Left Speed | Right Speed | Linear v | Angular ω | Motion |
|------------|-------------|----------|-----------|--------|
| 1.0 m/s | 1.0 m/s | 1.0 m/s | 0 rad/s | Straight forward |
| 0.5 m/s | 1.0 m/s | 0.75 m/s | 3.125 rad/s | Forward + left turn |
| -0.5 m/s | 0.5 m/s | 0 m/s | 6.25 rad/s | Spin in place |
| 0 m/s | 0 m/s | 0 m/s | 0 rad/s | Stopped |

---

## Extended Kalman Filter Theory

The Kalman Filter has two steps repeated in a loop:

### 1. Prediction (Time Update)

**What it does:** Predict where the robot will be based on motion model

```
x̂⁻ = f(x̂⁺, u)        # Predict state
P⁻ = F P⁺ Fᵀ + Q      # Predict covariance
```

Where:
- `x̂⁻`: Predicted state (before measurement)
- `x̂⁺`: Previous corrected state (after measurement)
- `u`: Control input (velocities)
- `f`: Nonlinear motion model
- `F`: Jacobian of f (linearization)
- `Q`: Process noise (model uncertainty)

### 2. Update (Measurement Update)

**What it does:** Correct the prediction using sensor measurements

```
y = z - h(x̂⁻)          # Innovation (measurement residual)
S = H P⁻ Hᵀ + R        # Innovation covariance
K = P⁻ Hᵀ S⁻¹          # Kalman gain
x̂⁺ = x̂⁻ + Ky          # Corrected state
P⁺ = (I - KH) P⁻       # Corrected covariance
```

Where:
- `z`: Actual measurement
- `h`: Measurement model (what we expect to measure)
- `H`: Jacobian of h
- `R`: Measurement noise
- `K`: Kalman gain (how much to trust measurement vs prediction)

---

## Prediction Step

### Motion Model

Our robot moves according to this **nonlinear** model:

```python
def f(state, v, ω_imu, dt):
    x, y, θ, b_g = state

    # De-bias the IMU measurement
    ω_corrected = ω_imu - b_g

    # Update position using wheel velocity and current heading
    x_new = x + v * cos(θ) * dt
    y_new = y + v * sin(θ) * dt

    # Update heading using de-biased IMU
    θ_new = θ + ω_corrected * dt

    # Bias persists (random walk model)
    b_g_new = b_g

    return [x_new, y_new, θ_new, b_g_new]
```

**Why this model?**

1. **Position**: If we're facing angle θ and moving at speed v:
   - x changes by `v × cos(θ) × dt` (horizontal component)
   - y changes by `v × sin(θ) × dt` (vertical component)

2. **Heading**: We trust the IMU for rotation rate (after removing bias)

3. **Bias**: We assume bias stays constant (with some random drift)

### Jacobian Matrix F

The EKF needs to **linearize** the nonlinear function f around the current state.

The Jacobian `F = ∂f/∂x` is:

```
F = [∂x_new/∂x    ∂x_new/∂y    ∂x_new/∂θ           ∂x_new/∂b_g  ]
    [∂y_new/∂x    ∂y_new/∂y    ∂y_new/∂θ           ∂y_new/∂b_g  ]
    [∂θ_new/∂x    ∂θ_new/∂y    ∂θ_new/∂θ           ∂θ_new/∂b_g  ]
    [∂b_g_new/∂x  ∂b_g_new/∂y  ∂b_g_new/∂θ         ∂b_g_new/∂b_g]
```

Let's compute each element:

**Row 1: x_new = x + v cos(θ) dt**
- `∂x_new/∂x = 1` (x appears with coefficient 1)
- `∂x_new/∂y = 0` (y doesn't appear)
- `∂x_new/∂θ = ∂/∂θ [v cos(θ) dt] = -v sin(θ) dt`
- `∂x_new/∂b_g = 0` (b_g doesn't appear)

**Row 2: y_new = y + v sin(θ) dt**
- `∂y_new/∂x = 0`
- `∂y_new/∂y = 1`
- `∂y_new/∂θ = ∂/∂θ [v sin(θ) dt] = v cos(θ) dt`
- `∂y_new/∂b_g = 0`

**Row 3: θ_new = θ + (ω_imu - b_g) dt**
- `∂θ_new/∂x = 0`
- `∂θ_new/∂y = 0`
- `∂θ_new/∂θ = 1`
- `∂θ_new/∂b_g = ∂/∂b_g [(ω_imu - b_g) dt] = -dt`

**Row 4: b_g_new = b_g**
- `∂b_g_new/∂x = 0`
- `∂b_g_new/∂y = 0`
- `∂b_g_new/∂θ = 0`
- `∂b_g_new/∂b_g = 1`

**Final Jacobian:**
```python
F = [[1,  0,  -v*sin(θ)*dt,   0 ],
     [0,  1,   v*cos(θ)*dt,   0 ],
     [0,  0,   1,            -dt],
     [0,  0,   0,             1 ]]
```

**KEY INSIGHT:** `F[2,3] = -dt` creates correlation between θ and b_g!

When we later measure heading, the Kalman gain will use this correlation to update both θ AND b_g simultaneously.

### Covariance Prediction

```python
P = F @ P @ F.T + Q
```

This says:
1. `F @ P @ F.T`: Propagate old uncertainty through the motion model
2. `+ Q`: Add new uncertainty from process noise

**Example:**
- If we're very uncertain about θ (large `P[2,2]`)
- And we move forward (large v)
- Then `F[0,2] = -v sin(θ) dt` spreads that θ uncertainty into x
- Result: Position uncertainty grows!

### Code Implementation

```python
def ekf_predict(self, v, omega_imu, dt):
    x, y, theta, b_g = self.state

    # De-biased IMU angular velocity
    omega_corrected = omega_imu - b_g

    # State prediction
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + omega_corrected * dt
    theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))  # Wrap to [-π, π]
    b_g_new = b_g

    self.state = np.array([x_new, y_new, theta_new, b_g_new])

    # Jacobian
    F = np.array([
        [1, 0, -v * np.sin(theta) * dt, 0],
        [0, 1,  v * np.cos(theta) * dt, 0],
        [0, 0, 1, -dt],
        [0, 0, 0, 1]
    ])

    # Process noise (see next section)
    Q = self._compute_process_noise(v, omega_corrected, theta, dt)

    # Covariance prediction
    self.P = F @ self.P @ F.T + Q
```

**Why wrap angle?** `np.arctan2(sin(θ), cos(θ))` keeps θ in `[-π, π]` to avoid numerical issues.

---

## Update Step 1: Wheel Measurement

### What Are We Measuring?

The wheel encoders give us angular velocity `ω_wheel = (v_R - v_L) / L`.

We compare this to the IMU:
```
Measurement:      z = ω_wheel
Expected value:   h(x) = ω_imu - b_g
```

**Why `ω_imu - b_g`?**

The IMU measures `ω_imu = ω_true + b_g + noise`. If we've correctly estimated the bias, then the de-biased IMU `ω_imu - b_g` should match the wheel-derived angular velocity `ω_wheel`.

### Measurement Model

```python
def h(state, omega_imu):
    b_g = state[3]
    return omega_imu - b_g  # De-biased IMU prediction
```

### Observation Jacobian H

```
h(x) = ω_imu - x[3]
```

Therefore:
```
H = [∂h/∂x, ∂h/∂y, ∂h/∂θ, ∂h/∂b_g]
  = [0,     0,     0,     -1     ]
```

**This says:** The measurement only directly depends on bias.

**BUT:** Remember `P[2,3]` (correlation between θ and b_g) from prediction!

The Kalman gain will exploit this correlation to update BOTH θ and b_g.

### Innovation (Measurement Residual)

```python
z = omega_wheel                    # What we measured
h_x = omega_imu - state[3]        # What we expected
innovation = z - h_x               # Difference
```

Expanding:
```
innovation = ω_wheel - (ω_imu - b_g)
          = ω_wheel - ω_imu + b_g
```

**If innovation > 0:** Wheels say we're turning faster than IMU → maybe b_g is too small
**If innovation < 0:** Wheels say we're turning slower than IMU → maybe b_g is too large

### Innovation Covariance

```python
S = H @ P @ H.T + R_wheel
```

Where `R_wheel = σ²_wheel` is the measurement noise variance.

`S` tells us: "How uncertain is our innovation?"
- Large `S`: High uncertainty → trust prediction more
- Small `S`: Low uncertainty → trust measurement more

### Kalman Gain

```python
K = P @ H.T @ inv(S)
```

The Kalman gain is a **4×1** vector that says how much each state should change:

```
K = [K_x  ]   ← How much to adjust x per unit innovation
    [K_y  ]   ← How much to adjust y per unit innovation
    [K_θ  ]   ← How much to adjust θ per unit innovation
    [K_bg ]   ← How much to adjust b_g per unit innovation
```

**Magic of EKF:**
- Even though H = [0, 0, 0, -1] (measurement only sees bias)
- K will have non-zero entries for θ!
- This is because `P[2,3] ≠ 0` (correlation from prediction step)

**Mathematical reason:**
```
K = P @ H.T @ inv(S)
  = [P[0,0]  P[0,1]  P[0,2]  P[0,3]] @ [0  ]
    [P[1,0]  P[1,1]  P[1,2]  P[1,3]]   [0  ]  @ inv(S)
    [P[2,0]  P[2,1]  P[2,2]  P[2,3]]   [0  ]
    [P[3,0]  P[3,1]  P[3,2]  P[3,3]]   [-1 ]

  = [-P[0,3]]
    [-P[1,3]]  @ inv(S)
    [-P[2,3]]  ← This is NON-ZERO because prediction created correlation!
    [-P[3,3]]
```

### State Update

```python
state_new = state + K @ innovation
```

**Example:**
- Innovation = 0.1 rad/s (wheels say turning faster than IMU predicts)
- `K_θ = 0.05`, `K_bg = 0.8`
- θ increases by `0.05 × 0.1 = 0.005` rad
- b_g increases by `0.8 × 0.1 = 0.08` rad/s

The filter says: "We're turning faster than expected. This is probably because our bias estimate was too low, but also adjust heading a bit."

### Covariance Update

Standard form:
```python
P = (I - K @ H) @ P
```

But we use **Joseph form** (see later section) for numerical stability:
```python
I_KH = I - K @ H
P = I_KH @ P @ I_KH.T + K @ R @ K.T
```

### Code Implementation

```python
def ekf_update_wheel(self, omega_wheel, omega_imu, dt):
    # Measurement
    z = np.array([omega_wheel])
    h_x = np.array([omega_imu - self.state[3]])

    # Innovation
    innovation = z - h_x

    # Observation matrix
    H = np.array([[0.0, 0.0, 0.0, -1.0]])

    # Innovation covariance
    S = H @ self.P @ H.T + self.R_wheel

    # Mahalanobis gating (reject outliers)
    mahal_dist = float(innovation @ np.linalg.inv(S) @ innovation.T)
    if mahal_dist > self.mahal_threshold:
        self._rejected_count += 1
        return

    # Kalman gain
    K = self.P @ H.T @ np.linalg.inv(S)

    # State correction
    self.state += (K @ innovation).flatten()
    self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

    # Covariance update (Joseph form)
    I_KH = np.eye(4) - K @ H
    self.P = I_KH @ self.P @ I_KH.T + K @ self.R_wheel @ K.T
    self.P = (self.P + self.P.T) / 2.0  # Ensure symmetry
```

---

## Update Step 2: IMU Orientation

### What Are We Measuring?

Some IMUs provide orientation (quaternion) in addition to angular velocity. We can extract yaw (heading) and use it as a direct measurement of θ.

```
Measurement:      z = imu_yaw
Expected value:   h(x) = x[2]  (current heading estimate)
```

### Measurement Model

```python
def h(state):
    return state[2]  # Just return theta
```

### Observation Jacobian

```
h(x) = x[2]
```

Therefore:
```
H = [∂h/∂x, ∂h/∂y, ∂h/∂θ, ∂h/∂b_g]
  = [0,     0,     1,     0        ]
```

This directly measures heading!

### Innovation

```python
innovation = imu_yaw - state[2]
# Wrap to [-π, π]
innovation = atan2(sin(innovation), cos(innovation))
```

**Why wrap?** Angles are circular:
- If true heading = 179° and estimate = -179°
- Simple difference = 358° (WRONG!)
- Wrapped difference = -2° (CORRECT!)

### Update Process

Same as wheel update:
1. Compute innovation covariance `S = H P Hᵀ + R_heading`
2. Mahalanobis gating
3. Compute Kalman gain `K = P Hᵀ S⁻¹`
4. Update state: `state += K × innovation`
5. Update covariance (Joseph form)

### Why Higher Noise?

```python
R_wheel = 0.01    # Trust wheels more
R_heading = 0.05  # Trust IMU orientation less
```

**Reason:** IMU orientation (from magnetometer or filter) can drift over time and be affected by magnetic interference. We use it to help, but don't trust it as much as the wheel/gyro combination.

### Code Implementation

```python
def ekf_update_orientation(self):
    # Innovation (angle wrapping is critical!)
    innovation = self.imu_yaw - self.state[2]
    innovation = np.arctan2(np.sin(innovation), np.cos(innovation))
    innovation = np.array([innovation])

    # Observation matrix
    H = np.array([[0.0, 0.0, 1.0, 0.0]])

    # Innovation covariance
    S = H @ self.P @ H.T + self.R_heading

    # Mahalanobis gating
    mahal_dist = float(innovation @ np.linalg.inv(S) @ innovation.T)
    if mahal_dist > self.mahal_threshold:
        self._rejected_count += 1
        return

    # Kalman gain
    K = self.P @ H.T @ np.linalg.inv(S)

    # State correction
    self.state += (K @ innovation).flatten()
    self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

    # Covariance update (Joseph form)
    I_KH = np.eye(4) - K @ H
    self.P = I_KH @ self.P @ I_KH.T + K @ self.R_heading @ K.T
    self.P = (self.P + self.P.T) / 2.0
```

---

## Process Noise Model (Thrun's Alpha)

### Why Velocity-Dependent Noise?

**Problem with constant Q:**
- When robot is stationary: prediction should be very accurate (small noise)
- When robot is moving fast: more uncertainty due to wheel slip, vibration, etc.

**Solution:** Scale process noise with velocity!

### Thrun's Motion Noise Model

From probabilistic robotics (Thrun et al.), motion noise is modeled as:

```
σ²_v = α₁ v² + α₂ ω²
σ²_ω = α₃ v² + α₄ ω²
```

Where:
- `σ²_v`: Variance in linear velocity
- `σ²_ω`: Variance in angular velocity
- `α₁, α₂, α₃, α₄`: Robot-specific parameters

**Physical interpretation:**
- `α₁ v²`: Fast linear motion causes linear noise (wheel slip)
- `α₂ ω²`: Fast rotation causes linear noise (centrifugal effects)
- `α₃ v²`: Fast linear motion causes angular noise (differential wheel slip)
- `α₄ ω²`: Fast rotation causes angular noise (gyro drift)

### Our Parameters

```python
alpha1 = 0.05   # velocity  → velocity noise
alpha2 = 0.01   # rotation  → velocity noise
alpha3 = 0.01   # velocity  → rotation noise
alpha4 = 0.05   # rotation  → rotation noise
```

**Tuning guide:**
- Larger α → more process noise → trust measurements more
- Smaller α → less process noise → trust motion model more

### From Control Noise to State Noise

We have noise in **controls** (v, ω), but we need noise in **state** (x, y, θ, b_g).

**Control noise covariance:**
```python
M = [[σ²_v,  0   ],
     [0,     σ²_ω]]
```

**Control Jacobian** (how controls affect state):

From the motion model:
```
Δx = v cos(θ) dt
Δy = v sin(θ) dt
Δθ = ω dt
```

The Jacobian `V = ∂state/∂[v,ω]` is:

```python
V = [[cos(θ) dt,  0  ],   # ∂x/∂v,  ∂x/∂ω
     [sin(θ) dt,  0  ],   # ∂y/∂v,  ∂y/∂ω
     [0,          dt ],   # ∂θ/∂v,  ∂θ/∂ω
     [0,          0  ]]   # ∂b_g/∂v, ∂b_g/∂ω
```

**Project control noise into state space:**
```python
Q = V @ M @ V.T
```

This gives us a 4×4 matrix where:
- `Q[0,0]`: Position x noise
- `Q[1,1]`: Position y noise
- `Q[2,2]`: Heading θ noise
- `Q[0,1]`: x-y correlation

**Add bias noise:**
```python
Q[3,3] = Q_bias  # Random walk (constant small drift)
```

### Code Implementation

```python
def _compute_process_noise(self, v, omega, theta, dt):
    # Motion-dependent noise variances (with minimum floor)
    sigma_v_sq = self.alpha1 * v**2 + self.alpha2 * omega**2 + 1e-6
    sigma_w_sq = self.alpha3 * v**2 + self.alpha4 * omega**2 + 1e-6

    # Control noise covariance [2x2]
    M = np.diag([sigma_v_sq, sigma_w_sq])

    # Jacobian of motion model w.r.t. control noise [4x2]
    V = np.array([
        [np.cos(theta) * dt, 0],
        [np.sin(theta) * dt, 0],
        [0, dt],
        [0, 0]
    ])

    # Project control noise into state space [4x4]
    Q = V @ M @ V.T

    # Add bias random walk noise
    Q[3, 3] = self.Q_bias

    return Q
```

**Example:**
- Robot moving at v=1.0 m/s, ω=0.5 rad/s, dt=0.05s, θ=0
- `σ²_v = 0.05×1² + 0.01×0.5² = 0.0525`
- `σ²_ω = 0.01×1² + 0.05×0.5² = 0.0225`
- `V = [[0.05, 0], [0, 0], [0, 0.05], [0, 0]]`
- Result: Larger Q when moving fast!

---

## Mahalanobis Distance Gating

### What Is Outlier Rejection?

Sometimes sensors give bad measurements:
- Wheel slips on wet floor
- IMU electromagnetic interference
- Encoder glitch

If we blindly trust these outliers, they corrupt our estimate!

### Mahalanobis Distance

Instead of rejecting measurements based on simple threshold:
```
if |innovation| > threshold:  # BAD: doesn't account for uncertainty
    reject
```

We use **Mahalanobis distance**, which accounts for uncertainty:

```
d² = innovationᵀ S⁻¹ innovation
```

Where `S = H P Hᵀ + R` is the innovation covariance.

**Intuition:**
- If uncertainty is large (large S), we're more tolerant of big innovations
- If uncertainty is small (small S), we reject even moderate innovations

### Chi-Squared Distribution

Under the assumption that noise is Gaussian, `d²` follows a **chi-squared distribution** with `k` degrees of freedom (k = dimension of measurement).

For our measurements:
- Wheel measurement: k=1 (one scalar)
- IMU orientation: k=1 (one scalar)

**Chi-squared thresholds (k=1):**
| Confidence | Threshold |
|------------|-----------|
| 95% | 3.84 |
| 99% | 6.63 |
| 99.5% | 7.88 |

We use **7.88** (99.5%) to be conservative but still reject extreme outliers.

### Code Implementation

```python
# Innovation covariance
S = H @ self.P @ H.T + R

# Mahalanobis distance
mahal_dist = float(innovation @ np.linalg.inv(S) @ innovation.T)

# Reject if too large
if mahal_dist > self.mahal_threshold:  # 7.88
    self._rejected_count += 1
    return  # Skip this update
```

**Example:**
- Innovation = 0.5 rad/s
- S = 0.02 (low uncertainty)
- `d² = 0.5² / 0.02 = 12.5 > 7.88` → REJECT (too unlikely)

VS:
- Innovation = 0.5 rad/s
- S = 0.10 (high uncertainty)
- `d² = 0.5² / 0.10 = 2.5 < 7.88` → ACCEPT (plausible given uncertainty)

---

## Joseph Form Covariance Update

### Standard Kalman Covariance Update

The textbook formula is:
```python
P = (I - K @ H) @ P
```

**Problem:** Numerical errors can make P lose nice properties:
1. **Non-symmetric**: `P ≠ Pᵀ` (should be symmetric)
2. **Non-positive-definite**: Can get negative variances!

This happens due to:
- Floating-point rounding errors
- Matrix inversion numerical instability
- Accumulation over many iterations

### Joseph Form

The **Joseph form** is algebraically equivalent but numerically superior:

```python
I_KH = I - K @ H
P = I_KH @ P @ I_KH.T + K @ R @ K.T
```

**Why better?**

**Proof of equivalence:**

Starting from the definition of Kalman gain:
```
K = P Hᵀ S⁻¹
where S = H P Hᵀ + R
```

Standard form:
```
P⁺ = (I - KH) P
   = P - KH P
   = P - K(HP)
```

Joseph form (expand):
```
P⁺ = (I - KH) P (I - KH)ᵀ + K R Kᵀ
   = (I - KH) P (Iᵀ - (KH)ᵀ) + K R Kᵀ
   = (I - KH) P (I - HᵀKᵀ) + K R Kᵀ
   = P - KHP - PHᵀKᵀ + KHPHᵀKᵀ + KRKᵀ
```

Substitute `K = PHᵀS⁻¹`:
```
   = P - PHᵀS⁻¹HP - PHᵀS⁻¹HP + PHᵀS⁻¹HPHᵀS⁻¹HP + PHᵀS⁻¹R S⁻¹HP

Simplify using S = HPHᵀ + R:
   = P - KHP
```

So they're mathematically the same! But numerically:

**Advantages:**
1. **Guaranteed symmetric**: `(I-KH)P(I-KH)ᵀ` is automatically symmetric
2. **Guaranteed positive semi-definite**: Sum of PSD matrices is PSD
   - `(I-KH)P(I-KH)ᵀ` is PSD if P is PSD
   - `KRKᵀ` is PSD if R is PSD
3. **Adds back measurement noise**: The `KRKᵀ` term ensures we don't over-confident

### Code Implementation

```python
# Joseph form
I_KH = np.eye(4) - K @ H
self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T

# Force exact symmetry (fix tiny floating-point errors)
self.P = (self.P + self.P.T) / 2.0
```

**The symmetrization step** `(P + Pᵀ)/2` is a final safety check that removes any tiny asymmetry from floating-point errors.

---

## Complete Algorithm Flow

### Per-Timestep Processing

Here's what happens every 0.05 seconds (20 Hz):

```
┌─────────────────────────────────────────────┐
│ 1. Read Sensors                             │
│    - Get wheel positions from /joint_states │
│    - Get IMU angular velocity from /imu     │
│    - Get IMU orientation (if available)     │
└────────────┬────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────┐
│ 2. Compute Wheel Odometry                   │
│    Δleft = left_pos - prev_left_pos         │
│    Δright = right_pos - prev_right_pos      │
│    v_left = Δleft × r / dt                  │
│    v_right = Δright × r / dt                │
│    v = (v_left + v_right) / 2               │
│    ω_wheel = (v_right - v_left) / L         │
└────────────┬────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────┐
│ 3. EKF PREDICTION                           │
│    State:                                   │
│      x += v cos(θ) dt                       │
│      y += v sin(θ) dt                       │
│      θ += (ω_imu - b_g) dt                  │
│      b_g unchanged                          │
│                                             │
│    Covariance:                              │
│      Compute F (Jacobian)                   │
│      Compute Q (velocity-dependent)         │
│      P = F P Fᵀ + Q                         │
└────────────┬────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────┐
│ 4. EKF UPDATE (Wheel)                       │
│    Innovation:                              │
│      y = ω_wheel - (ω_imu - b_g)            │
│                                             │
│    Mahalanobis gating:                      │
│      S = H P Hᵀ + R_wheel                   │
│      d² = yᵀ S⁻¹ y                          │
│      if d² > 7.88: REJECT, return           │
│                                             │
│    Kalman update:                           │
│      K = P Hᵀ S⁻¹                           │
│      state += K y                           │
│      P = (I-KH) P (I-KH)ᵀ + K R Kᵀ          │
└────────────┬────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────┐
│ 5. EKF UPDATE (IMU Orientation) [Optional]  │
│    if IMU orientation available:            │
│      Innovation:                            │
│        y = imu_yaw - θ                      │
│                                             │
│      Mahalanobis gating:                    │
│        S = H P Hᵀ + R_heading               │
│        d² = yᵀ S⁻¹ y                        │
│        if d² > 7.88: REJECT, return         │
│                                             │
│      Kalman update:                         │
│        K = P Hᵀ S⁻¹                         │
│        state += K y                         │
│        P = (I-KH) P (I-KH)ᵀ + K R Kᵀ        │
└────────────┬────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────┐
│ 6. Publish Results                          │
│    - Odometry message to /odom_ekf          │
│    - TF transform: odom → base_footprint_ekf│
│    - Path for visualization                 │
│    - Save to trajectory file                │
└─────────────────────────────────────────────┘
```

### What Makes Bias Observable?

This is the **key insight** of the algorithm:

**Step 1:** Prediction creates correlation
```
F[2,3] = -dt  →  P[2,3] ≠ 0
```
After prediction, heading θ and bias b_g are correlated.

**Step 2:** Wheel update measures angular velocity
```
z = ω_wheel
h = ω_imu - b_g
H = [0, 0, 0, -1]
```
This measurement directly depends on bias.

**Step 3:** Kalman gain exploits correlation
```
K = P Hᵀ S⁻¹
  = [P[0,3], P[1,3], P[2,3], P[3,3]]ᵀ / S
```
Since `P[2,3] ≠ 0`, the gain `K[2]` is non-zero.

**Step 4:** Both θ and b_g get updated
```
θ += K[2] × (ω_wheel - ω_imu + b_g)
b_g += K[3] × (ω_wheel - ω_imu + b_g)
```

**Physical interpretation:**
- If wheels and de-biased IMU disagree on rotation rate
- It could be because:
  1. Our heading is wrong (update θ)
  2. Our bias estimate is wrong (update b_g)
- The Kalman filter optimally splits the correction between both!

---

## Implementation Details

### Angle Wrapping

**Problem:** Angles are circular. Adding angular changes can make them go outside `[-π, π]`.

**Solution:** After every angle update:
```python
theta = np.arctan2(np.sin(theta), np.cos(theta))
```

This wraps θ to `[-π, π]` by:
1. Converting to unit vector: `(cos θ, sin θ)`
2. Converting back to angle with `atan2`

**Why it works:**
- `atan2(sin(370°), cos(370°)) = atan2(sin(10°), cos(10°)) = 10°`
- Handles all quadrants correctly (unlike `atan`)

### Innovation Wrapping

For orientation updates, wrap the innovation too:
```python
innovation = imu_yaw - theta
innovation = np.arctan2(np.sin(innovation), np.cos(innovation))
```

**Example:**
- IMU: 179° = 3.124 rad
- Estimate: -179° = -3.124 rad
- Raw difference: 6.248 rad (WRONG! 358°)
- Wrapped: -0.035 rad (CORRECT! -2°)

### Matrix Symmetry Enforcement

After covariance updates:
```python
self.P = (self.P + self.P.T) / 2.0
```

This fixes tiny numerical asymmetries (like `P[0,1] = 1e-15` but `P[1,0] = -1e-15`).

### Timing

```python
dt = (current_time - last_update_time).nanoseconds / 1e9
if dt <= 0:
    return  # Skip if no time passed
```

Handles:
- Zero time steps (duplicate callbacks)
- Negative time (shouldn't happen but be safe)

### Initialization

On the first timestep:
```python
if self.last_update_time is None:
    # Store initial wheel positions
    self.prev_left_pos = left_pos
    self.prev_right_pos = right_pos
    return  # Can't compute velocity without previous position
```

We need two measurements to compute velocity, so skip the first one.

### Covariance in Odometry Message

ROS Odometry messages have a 6×6 covariance (x, y, z, roll, pitch, yaw).

We only estimate x, y, yaw, so:
```python
odom.pose.covariance[0] = self.P[0, 0]   # x variance (index 0)
odom.pose.covariance[7] = self.P[1, 1]   # y variance (index 7 = 1×6+1)
odom.pose.covariance[35] = self.P[2, 2]  # yaw variance (index 35 = 5×6+5)
```

Other entries stay zero (we don't estimate z, roll, pitch).

---

## Summary: Why This Works

### The Problem
- **Wheel odometry** alone: position drifts due to heading errors
- **IMU** alone: heading drifts due to gyro bias

### The Solution
1. **Use IMU for heading prediction** (high-rate angular velocity)
2. **Use wheels for position prediction** (measure distance traveled)
3. **Compare wheels vs IMU** for heading rate (estimates bias)
4. **Optionally use IMU orientation** for absolute heading reference

### Key Innovations
1. **Bias estimation**: State augmentation makes gyro bias observable
2. **Velocity-dependent noise**: More realistic uncertainty model
3. **Mahalanobis gating**: Robust to sensor outliers
4. **Joseph form**: Numerically stable covariance updates
5. **Dual updates**: Wheel + orientation for maximum information

### Expected Performance
- **Better heading** than wheel odometry alone (thanks to IMU)
- **Less bias drift** than IMU alone (thanks to wheels)
- **Uncertainty quantification** (covariance tells us confidence)
- **Outlier robustness** (Mahalanobis gating)

---

## References

1. **Probabilistic Robotics** - Thrun, Burgard, Fox
   - Chapter 5: Mobile Robot Localization
   - Chapter 7: Kalman Filters
   - Section 5.4.2: Velocity-dependent motion noise

2. **Optimal State Estimation** - Dan Simon
   - Chapter 13: Extended Kalman Filter
   - Section 6.4: Joseph form covariance update

3. **ROS Navigation Tuning Guide**
   - http://wiki.ros.org/navigation/Tuning

---

## Appendix: Parameter Tuning Guide

### Process Noise (α₁, α₂, α₃, α₄)

**Symptoms of too small:**
- Filter is over-confident
- Doesn't adapt to new information quickly
- Innovations are consistently large

**Symptoms of too large:**
- Filter is under-confident
- Jumpy, erratic estimates
- Overly trusts measurements

**Tuning:**
1. Start with α₁ = α₄ = 0.05, α₂ = α₃ = 0.01
2. Drive robot in circles at constant velocity
3. Check innovation consistency (should be zero-mean)
4. Increase α if innovations have bias, decrease if too noisy

### Measurement Noise (R_wheel, R_heading)

**Symptoms of too small:**
- Jumpy estimates when measurement updates
- Filter overly trusts bad measurements

**Symptoms of too large:**
- Measurements ignored
- Acts like prediction-only filter

**Tuning:**
1. Record data with robot stationary
2. Compute variance of ω_wheel - should match R_wheel
3. For R_heading, set 2-5× larger than R_wheel (less trust)

### Mahalanobis Threshold

**Symptoms of too small:**
- Too many rejections
- Filter ignores valid measurements

**Symptoms of too large:**
- Outliers accepted
- Occasional jumps in estimate

**Tuning:**
- 3.84 (95%): Aggressive, may reject valid data
- 7.88 (99.5%): Conservative, only rejects obvious outliers (recommended)
- No gating: Vulnerable to outliers

### Bias Random Walk (Q_bias)

**Symptoms of too small:**
- Can't track time-varying bias
- Bias estimate freezes

**Symptoms of too large:**
- Bias estimate drifts randomly
- Unstable

**Tuning:**
- Typical: 1e-8 to 1e-6
- Depends on IMU quality (better IMU → smaller Q_bias)

---

**END OF DOCUMENT**
