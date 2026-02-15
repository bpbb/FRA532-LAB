# EKF Odometry

## Overview

Extended Kalman Filter implementation for TurtleBot3 Burger fusing wheel encoders and IMU measurements. State vector includes gyroscope bias estimation for improved heading accuracy.

## State Vector

```
x = [x, y, θ, b_g]ᵀ
```

- `x, y`: Robot position (meters) in odom frame
- `θ`: Heading (radians)
- `b_g`: Gyroscope bias (rad/s)

Covariance matrix `P` (4×4) tracks uncertainty and correlations between states.

## Motion Model

### Differential Drive Kinematics

From wheel encoder positions to robot velocities:

```python
v_left = (Δleft × r) / dt
v_right = (Δright × r) / dt
v = (v_right + v_left) / 2.0
ω_wheel = (v_right - v_left) / L
```

Where `r = 0.033m` (wheel radius), `L = 0.160m` (wheel base).

### Prediction Step

Nonlinear motion model:

```python
x' = x + v × cos(θ) × dt
y' = y + v × sin(θ) × dt
θ' = θ + (ω_imu - b_g) × dt
b_g' = b_g  # Persistence model
```

Jacobian matrix:

```
F = [[1,  0,  -v×sin(θ)×dt,   0 ],
     [0,  1,   v×cos(θ)×dt,   0 ],
     [0,  0,   1,            -dt],
     [0,  0,   0,             1 ]]
```

Covariance propagation: `P = F P Fᵀ + Q`

## Process Noise (Thrun's Model)

Velocity-dependent motion noise:

```
σ²_v = α₁v² + α₂ω²
σ²_ω = α₃v² + α₄ω²
```

Parameters: `α₁=0.05, α₂=0.01, α₃=0.01, α₄=0.05`

Control Jacobian:

```
V = [[cos(θ)×dt,  0  ],
     [sin(θ)×dt,  0  ],
     [0,          dt ],
     [0,          0  ]]
```

Process noise: `Q = V M Vᵀ + diag([0, 0, 0, Q_bias])`

## Update Step 1: Wheel Angular Velocity

Measurement model compares wheel-derived angular velocity with de-biased IMU:

```
z = ω_wheel
h(x) = ω_imu - b_g
```

Observation Jacobian: `H = [0, 0, 0, -1]`

Innovation: `y = ω_wheel - (ω_imu - b_g)`

Innovation covariance: `S = H P Hᵀ + R_wheel`

Kalman gain: `K = P Hᵀ S⁻¹`

State update: `x = x + K y`

Covariance update (Joseph form): `P = (I-KH) P (I-KH)ᵀ + K R Kᵀ`

## Update Step 2: IMU Orientation

Direct heading measurement from IMU quaternion:

```
z = yaw_imu
h(x) = θ
```

Observation Jacobian: `H = [0, 0, 1, 0]`

Innovation requires angle wrapping: `y = arctan2(sin(yaw_imu - θ), cos(yaw_imu - θ))`

Same Kalman update process with `R_heading = 0.05` (less trust than wheel measurement).

## Robustness Features

### Mahalanobis Distance Gating

Outlier rejection using chi-squared test:

```
d² = yᵀ S⁻¹ y
```

Threshold: 7.88 (99.5% confidence, 1 DOF)

Reject measurement if `d² > 7.88`.

### Joseph Form Covariance

Numerically stable covariance update:

```
P = (I-KH) P (I-KH)ᵀ + K R Kᵀ
```

Guarantees symmetry and positive semi-definiteness. Force exact symmetry: `P = (P + Pᵀ)/2`

### Angle Wrapping

All angle operations wrapped to [-π, π]:

```python
θ = arctan2(sin(θ), cos(θ))
```

## Observability

Bias becomes observable through correlation created in prediction step (`F[2,3] = -dt`). When measuring angular velocity, Kalman gain exploits `P[2,3] ≠ 0` to update both heading and bias simultaneously.

## Performance Characteristics

**Strengths:**
- Reduced heading drift (IMU gyroscope corrections)
- Online bias adaptation
- Uncertainty quantification
- Outlier robustness

**Limitations:**
- Cannot correct translational drift (no absolute position reference)
- Bias estimation requires sufficient motion excitation
