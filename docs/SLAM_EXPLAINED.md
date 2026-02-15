# SLAM with slam_toolbox

## Overview

Graph-based SLAM using slam_toolbox (ROS2 package) for simultaneous localization and mapping with loop closure. Builds global map and corrects accumulated odometry drift through graph optimization.

## SLAM vs Odometry

| Aspect | Odometry | SLAM |
|--------|----------|------|
| **Map** | None | Global occupancy grid |
| **Drift** | Unbounded | Bounded by loop closure |
| **Complexity** | Low | High |
| **Output** | Pose in odom frame | Pose + map in map frame |
| **Loop closure** | No | Yes |

## Graph-Based SLAM

### Pose Graph Structure

**Nodes**: Robot poses at different timesteps `{x₀, x₁, x₂, ..., xₙ}`

**Edges**: Constraints between poses

1. **Odometry edges** (sequential): Connect consecutive poses with measured motion `(Δx, Δy, Δθ)` and uncertainty (covariance)
2. **Loop closure edges** (non-sequential): Connect distant poses when robot revisits location

### Optimization Problem

Minimize sum of squared errors weighted by uncertainty:

```
x* = argmin Σᵢⱼ ||h(xᵢ, xⱼ) - zᵢⱼ||²_Σᵢⱼ
```

Where:
- `xᵢ, xⱼ`: Pose nodes
- `zᵢⱼ`: Measured relative transformation
- `h(xᵢ, xⱼ)`: Expected relative transformation
- `Σᵢⱼ`: Measurement covariance

Solver distributes error across all poses to satisfy all constraints optimally.

## Loop Closure

### Detection Process

1. **Candidate search**: Find poses distant in time (>10s) but close in space (<3m)
2. **Scan matching**: Run ICP between current and candidate scans
3. **Verification**: Check alignment quality (error, correspondences)
4. **Add constraint**: Create loop closure edge if verified
5. **Optimize graph**: Adjust all poses to satisfy new constraint

### Impact

Loop closure distributes accumulated drift backwards through trajectory:
- Before: Final pose drifts from true position
- After: Entire trajectory adjusted to close loop
- Result: Bounded error throughout trajectory

## slam_toolbox Architecture

### Inputs

1. **`/scan`** (sensor_msgs/LaserScan): LiDAR measurements at 5 Hz
2. **TF: `odom → base_footprint_ekf`**: Odometry from EKF node (via TF lookup)

### Outputs

1. **`/map`** (nav_msgs/OccupancyGrid): Global occupancy grid map
2. **TF: `map → odom`**: SLAM correction transform
3. **Full pose**: `map → odom → base_footprint_ekf` gives SLAM-corrected pose


## Trajectory Recorder Implementation

Custom `slam.py` node records SLAM trajectory:

### TF Lookup

10 Hz timer queries transform:

```python
transform = tf_buffer.lookup_transform(
    'map', 'base_footprint_ekf', rclpy.time.Time()
)
```

Extract pose:

```python
x = transform.translation.x
y = transform.translation.y

# Quaternion to yaw
q = transform.rotation
siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
θ = atan2(siny_cosp, cosy_cosp)
```

### Output

- **Trajectory file**: `trajectory_slam_seqXX.txt` (timestamp, x, y, θ)
- **Path topic**: `/path_slam` for RViz visualization
- **Map image**: `map_slam_seqXX.png` (saved on shutdown)

### Map Subscription

Subscribes to `/map` with **Transient Local** QoS (durability) to receive latest map even if subscription starts late.

Map conversion: OccupancyGrid (-1=unknown, 0=free, 100=occupied) → PNG (gray, white, black).

## Launch and Data Flow

Single launch file (`part3_complete.launch.py`) starts:
- Wheel odometry node
- EKF odometry node
- ICP odometry node
- slam_toolbox (async mode)
- `slam.py` trajectory recorder
- RViz
- Bag player (with `--clock 200`)

Data flow:
```
Bag (/scan, /imu, /joint_states)
  → EKF node (publishes TF: odom → base_footprint_ekf)
  → slam.py (publishes TF: base_footprint_ekf → base_scan)
  → slam_toolbox (reads /scan + TF, publishes /map + TF: map → odom)
  → slam.py (looks up map → base_footprint_ekf, records trajectory)
```
## Performance Characteristics

**Strengths:**

- Eliminates unbounded drift through loop closure
- Provides globally consistent map for navigation
- Graph optimization distributes error optimally
- Handles long-term operation in complex environments

**Limitations:**

- Higher computational cost than odometry
- Requires loop closure opportunities (revisiting areas)
- Map quality depends on feature availability
- Graph size grows with trajectory length
