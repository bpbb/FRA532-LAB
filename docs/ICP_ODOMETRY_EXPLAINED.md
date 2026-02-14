# ICP Odometry - Complete Mathematical Explanation

## Table of Contents
1. [Overview](#overview)
2. [ICP Fundamentals](#icp-fundamentals)
3. [Scan-to-Map vs Scan-to-Scan](#scan-to-map-vs-scan-to-scan)
4. [Local Map Management](#local-map-management)
5. [Point-to-Point ICP Algorithm](#point-to-point-icp-algorithm)
6. [SVD-Based Pose Estimation](#svd-based-pose-estimation)
7. [Robustness Features](#robustness-features)
8. [Integration with EKF](#integration-with-ekf)
9. [Occupancy Grid Map Building](#occupancy-grid-map-building)
10. [Complete Algorithm Flow](#complete-algorithm-flow)
11. [Implementation Details](#implementation-details)

---

## Overview

### What is ICP Odometry?

**Iterative Closest Point (ICP)** is a geometric alignment algorithm that:
- Matches point clouds by minimizing distance between corresponding points
- Iteratively refines the transformation estimate
- Provides highly accurate local pose estimation

### Why ICP After EKF?

**Problem with wheel+IMU odometry:**
- Heading drift accumulates from gyro bias
- Wheel slip causes position errors
- Both errors grow unbounded over time

**Solution with LiDAR:**
- LiDAR directly measures environment geometry
- Geometric constraints reduce drift significantly
- ICP provides "visual" correction to odometry

### Our Approach: Scan-to-Map ICP

Instead of matching consecutive scans, we:
1. **Use EKF as initial guess** (coarse but reliable)
2. **Build local map** from recent keyframes
3. **Match current scan to local map** using ICP
4. **Refine pose estimate** with geometric constraints

This combines the best of both:
- **EKF**: Handles fast motion, never gets lost
- **ICP**: Provides accurate geometric refinement

---

## ICP Fundamentals

### The Registration Problem

**Given:**
- **Source point cloud** `S = {s₁, s₂, ..., sₙ}` (current LiDAR scan)
- **Target point cloud** `T = {t₁, t₂, ..., tₘ}` (local map)

**Find:** Transformation `(x, y, θ)` that best aligns S with T

### Objective Function

Minimize the sum of squared distances between corresponding points:

```
E(x, y, θ) = Σᵢ ||T(sᵢ; x, y, θ) - tᵢ||²
```

Where:
- `T(s; x, y, θ)` transforms point s by pose (x, y, θ)
- `tᵢ` is the closest point in T to transformed `sᵢ`

**Problem:** This is non-convex (multiple local minima) due to unknown correspondences.

### ICP Strategy

ICP alternates between two steps:

**1. Correspondence step** (fix pose, find matches):
```
For each source point sᵢ:
    tᵢ = argmin_{t ∈ T} ||T(sᵢ; x, y, θ) - t||
```
Find the closest target point to each transformed source point.

**2. Optimization step** (fix matches, update pose):
```
(x, y, θ) = argmin_{x,y,θ} Σᵢ ||T(sᵢ; x, y, θ) - tᵢ||²
```
Find the transformation that minimizes distance to matched points.

**Convergence:** Iterate until the pose change is negligible.

### Why It Works

Each step decreases (or maintains) the objective:
- Correspondence step: Finds the best match for current transformation
- Optimization step: Finds the best transformation for current matches

The error monotonically decreases, guaranteeing convergence to a local minimum.

**Caveat:** May converge to wrong local minimum if initial guess is poor. This is why we use EKF as a good initial guess!

---

## Scan-to-Map vs Scan-to-Scan

### Scan-to-Scan ICP

**Approach:** Match consecutive LiDAR scans directly
```
Scan[t-1] ←→ Scan[t]
```

**Advantages:**
- Simple: only need two scans
- Fast: small point clouds

**Disadvantages:**
- **Drift accumulation**: Each pairwise match has error that accumulates
- **Limited overlap**: If robot moves fast, consecutive scans may not overlap well
- **Dynamic objects**: Moving objects cause spurious matches

### Scan-to-Map ICP (Our Approach)

**Approach:** Match current scan to accumulated local map
```
LocalMap ←→ Scan[t]
```

Where `LocalMap` = merged point cloud from recent keyframes

**Advantages:**
- **Reduced drift**: Map provides more stable reference
- **Larger overlap**: Map covers wider area than single scan
- **Better constraints**: More points → more robust optimization
- **Outlier filtering**: Dynamic objects in map get averaged out

**Disadvantages:**
- More complex: need map management
- Slower: larger target point cloud
- Memory: need to store map

**Our Implementation:** We use scan-to-map for better accuracy!

---

## Local Map Management

### Why a Local Map?

**Problem with global map:**
- Unbounded growth (memory issues)
- Old, incorrect data persists
- Computational cost increases over time

**Solution: Sliding window local map**
- Keep only recent keyframes (last 15 scans)
- Map moves with robot (like a "local window" into the world)
- Old keyframes automatically removed

### Keyframe Selection

**Not every scan becomes a keyframe!** Only when robot moves significantly:

```python
dist_from_last_kf = sqrt((x - last_kf_x)² + (y - last_kf_y)²)
angle_from_last_kf = |theta - last_kf_theta|

if dist > 0.15m OR angle > 5°:
    add_keyframe()
```

**Why?**
- **Redundancy reduction**: Consecutive scans at same pose are redundant
- **Computational efficiency**: Fewer keyframes = faster ICP
- **Information maximization**: Keyframes from different viewpoints provide more constraints

### Map Building Pipeline

```
┌─────────────────────────────────────────────────┐
│ 1. Current Scan (Body Frame)                    │
│    - Raw LiDAR points: (range, angle)           │
│    - Convert to 2D Cartesian: (x_b, y_b)        │
└────────────┬────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────┐
│ 2. Transform to Odom Frame                      │
│    Using current pose estimate (x, y, θ):       │
│    x_o = x_b cos(θ) - y_b sin(θ) + x            │
│    y_o = x_b sin(θ) + y_b cos(θ) + y            │
└────────────┬────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────┐
│ 3. Add to Keyframe Deque                        │
│    deque: [KF₁, KF₂, ..., KF₁₅]                │
│    When full, oldest KF₁ is automatically       │
│    removed (sliding window)                     │
└────────────┬────────────────────────────────────┘
             │
             ▼
┌─────────────────────────────────────────────────┐
│ 4. Rebuild Local Map (when dirty)               │
│    - Merge all keyframe point clouds            │
│    - Voxel downsample (0.05m grid)              │
│    - Build KDTree for fast nearest neighbor     │
└─────────────────────────────────────────────────┘
```

### Voxel Downsampling

**Problem:** Merged map has too many points (15 scans × 360 points = 5400 points!)

**Solution:** Voxel grid filtering

**Algorithm:**
1. Divide space into 3D grid (voxel size = 0.05m)
2. Assign each point to its voxel: `voxel = floor(point / voxel_size)`
3. For each voxel containing points, keep only the centroid

**Example:**
```
Voxel (5, 10):
  Points: [(0.25, 0.51), (0.26, 0.49), (0.24, 0.50)]
  Centroid: (0.25, 0.50) ← Keep this

Voxel (5, 11):
  Points: [(0.27, 0.56)]
  Centroid: (0.27, 0.56) ← Keep this
```

**Result:** ~5400 points → ~800 points (10× reduction) with minimal information loss!

### Code Implementation

```python
def transform_to_odom(self, points, x, y, theta):
    """Transform body-frame points to odom frame."""
    c = np.cos(theta)
    s = np.sin(theta)
    # Rotation matrix + translation
    return np.column_stack([
        points[:, 0] * c - points[:, 1] * s + x,  # x_odom
        points[:, 0] * s + points[:, 1] * c + y   # y_odom
    ])

def voxel_downsample(self, points, voxel_size):
    """Simple 2D voxel grid downsampling."""
    # Quantize to voxel indices
    voxel_indices = np.floor(points / voxel_size).astype(int)

    # Group points by voxel
    voxel_dict = {}
    for i in range(len(points)):
        key = (voxel_indices[i, 0], voxel_indices[i, 1])
        if key not in voxel_dict:
            voxel_dict[key] = []
        voxel_dict[key].append(points[i])

    # Compute centroid of each voxel
    downsampled = np.array([
        np.mean(pts, axis=0) for pts in voxel_dict.values()
    ])
    return downsampled
```

---

## Point-to-Point ICP Algorithm

### Iteration Loop

```python
for iteration in range(max_iterations):
    # Step 1: Find correspondences
    # Step 2: Optimize transformation
    # Step 3: Check convergence
```

### Step 1: Find Correspondences

**Transform source points** with current pose estimate:
```python
c = cos(theta)
s = sin(theta)
transformed = [[s_x * c - s_y * s + x,
                s_x * s + s_y * c + y]
               for (s_x, s_y) in scan_points]
```

**Find nearest neighbor** in local map for each transformed point:
```python
dists, indices = kdtree.query(transformed)
```

KDTree provides `O(log n)` nearest neighbor lookup (much faster than brute force `O(n)`).

**Filter by distance threshold:**
```python
valid = dists < max_correspondence_distance  # e.g., 0.5m
```

Points with no close match are outliers (dynamic objects, measurement noise).

**Check minimum correspondences:**
```python
if sum(valid) < min_correspondences:  # e.g., 30 points
    return FAILURE  # Not enough overlap
```

### Step 2: Trimming (Robust Estimation)

**Problem:** Some correspondences are still outliers (wrong matches).

**Solution:** Use only the best 80% of matches.

```python
if n_valid > 30:
    # Compute 80th percentile of distances
    trim_thresh = percentile(dists[valid], 80)
    # Keep only distances below this threshold
    valid = valid AND (dists <= trim_thresh)
```

**Why 80%?**
- Conservative: Keeps most good matches
- Robust: Removes worst 20% outliers
- Common in robust statistics (similar to RANSAC)

### Step 3: Compute Transformation

Now we have:
- **Source points** (body frame): `src = scan_points[valid]`
- **Target points** (odom frame): `tgt = map_points[indices[valid]]`

We need to find `(x, y, θ)` that aligns src to tgt.

This is a **2D rigid transformation estimation problem**, solved using SVD (see next section).

### Step 4: Check Convergence

```python
mean_error = mean(dists[valid])

if |prev_error - mean_error| < tolerance:  # e.g., 1e-6
    break  # Converged!

prev_error = mean_error
```

**Convergence criteria:**
- Error change is tiny (algorithm has stopped improving)
- Or reached max iterations (e.g., 50)

---

## SVD-Based Pose Estimation

### The Problem

**Given:**
- Source points `S = {s₁, s₂, ..., sₙ}` in body frame
- Target points `T = {t₁, t₂, ..., tₙ}` in odom frame (correspondences known)

**Find:** Rotation `R` and translation `t` such that:
```
tᵢ ≈ R sᵢ + t    for all i
```

### The Solution: Kabsch Algorithm

**Step 1: Center the point clouds**

Compute centroids:
```
s_c = (1/n) Σᵢ sᵢ
t_c = (1/n) Σᵢ tᵢ
```

Center the points:
```
s'ᵢ = sᵢ - s_c
t'ᵢ = tᵢ - t_c
```

**Why?** Removing translation simplifies the rotation estimation.

**Step 2: Compute cross-covariance matrix**

```
W = Σᵢ t'ᵢ ⊗ s'ᵢ
  = [Σ t'ᵢ,ₓ s'ᵢ,ₓ    Σ t'ᵢ,ₓ s'ᵢ,ᵧ]
    [Σ t'ᵢ,ᵧ s'ᵢ,ₓ    Σ t'ᵢ,ᵧ s'ᵢ,ᵧ]
```

In code:
```python
W = (target_centered.T @ source_centered)  # 2×2 matrix
```

**Intuition:** W captures the orientation relationship between source and target.

**Step 3: SVD decomposition**

```
W = U Σ Vᵀ
```

Where:
- `U`: 2×2 orthogonal matrix (left singular vectors)
- `Σ`: 2×2 diagonal matrix (singular values)
- `Vᵀ`: 2×2 orthogonal matrix (right singular vectors transpose)

**Step 4: Extract rotation**

```
R = U [[1,  0    ],  Vᵀ
       [0,  sign(det(UVᵀ))]]
```

The `sign(det(UVᵀ))` ensures:
- If det > 0: Proper rotation (preserves orientation)
- If det < 0: Reflection, so we flip to get proper rotation

**Why this works?** SVD gives the optimal rotation matrix that minimizes Frobenius norm between W and rotation matrix.

**Step 5: Extract translation**

Now that we have R, translation is:
```
t = t_c - R s_c
```

**Derivation:**
```
We want: tᵢ = R sᵢ + t
Centering: (tᵢ - t_c) = R(sᵢ - s_c) + t - R s_c + R s_c - t_c
Rearranging: t_c = R s_c + t
Therefore: t = t_c - R s_c
```

**Step 6: Extract (x, y, θ)**

From the 2×2 rotation matrix:
```
R = [[cos θ,  -sin θ],
     [sin θ,   cos θ]]
```

Extract angle:
```
θ = atan2(R[1,0], R[0,0])
```

Extract translation:
```
x = t[0]
y = t[1]
```

### Code Implementation

```python
# 1. Center point clouds
src_c = np.mean(src_matched, axis=0)
tgt_c = np.mean(tgt_matched, axis=0)

src_centered = src_matched - src_c
tgt_centered = tgt_matched - tgt_c

# 2. Cross-covariance
W = (tgt_centered.T @ src_centered)

# 3. SVD
U, _, Vt = np.linalg.svd(W)

# 4. Rotation (with reflection check)
d = np.linalg.det(U @ Vt)
R_opt = U @ np.diag([1.0, np.sign(d)]) @ Vt

# 5. Translation
t_opt = tgt_c - R_opt @ src_c

# 6. Extract pose
theta = np.arctan2(R_opt[1, 0], R_opt[0, 0])
x = t_opt[0]
y = t_opt[1]
```

### Why SVD?

**Advantages:**
- **Optimal**: Minimizes least squares error
- **Closed-form**: No iterative optimization needed
- **Fast**: O(n) for n points (most time in SVD of 2×2 matrix)
- **Numerically stable**: SVD is well-conditioned

**Alternative methods:**
- **Gradient descent**: Slower, may get stuck in local minimum
- **RANSAC**: More robust but much slower
- **Analytical**: Only for special cases

---

## Robustness Features

### 1. Maximum Correspondence Distance

```python
max_correspondence_distance = 0.5  # meters
```

**What it does:** Reject matches where points are far apart.

**Why?**
- Far matches are likely wrong (different objects)
- Prevents pulling points toward distant features
- Acts as an outlier filter

**Example:**
- Scan point at (1.0, 0.0)
- Nearest map point at (1.8, 0.0)
- Distance = 0.8m > 0.5m → REJECT

### 2. Minimum Correspondences

```python
min_correspondences = 30
```

**What it does:** Fail ICP if too few matches.

**Why?**
- Few matches = unreliable transformation estimate
- Better to trust EKF than bad ICP
- Prevents degenerate cases (e.g., all points rejected)

### 3. Trimming (80th Percentile)

```python
trim_thresh = percentile(valid_dists, 80)
valid = valid AND (dists <= trim_thresh)
```

**What it does:** Use only the best 80% of correspondences.

**Why?**
- Even with distance threshold, some outliers remain
- Trimming is a form of robust estimation
- Similar principle to RANSAC but simpler

**Example:**
- 100 valid correspondences
- Distances: [0.02, 0.03, 0.05, ..., 0.40, 0.48, 0.49]  (sorted)
- 80th percentile: 0.35m
- Keep 80 best matches, reject 20 worst

### 4. Scan Downsampling

```python
max_scan_points = 300
if len(current_scan) > 300:
    subsample randomly
```

**What it does:** Limit scan size for ICP.

**Why?**
- Faster correspondence search
- Less memory
- 300 points sufficient for good pose estimate
- Random sampling preserves statistical properties

### 5. Voxel Downsampling of Map

```python
local_map_voxel_size = 0.05  # meters
```

**What it does:** Reduce map point density.

**Why?**
- Faster KDTree queries
- Removes redundant points
- 0.05m resolution sufficient for indoor navigation

### 6. ICP Correction Limits

```python
max_translation_correction = 0.15  # meters
max_rotation_correction = radians(2.0)  # degrees
```

**What it does:** Reject ICP result if too different from EKF.

**Why?**
- **Safety check**: ICP shouldn't drastically disagree with EKF
- Large corrections usually indicate ICP failure
- EKF is reliable short-term; trust it more than suspicious ICP

**Example:**
- EKF says: (1.0, 0.5, 45°)
- ICP says: (1.3, 0.4, 43°)
- Translation diff: sqrt((1.3-1.0)² + (0.4-0.5)²) = 0.32m
- 0.32m > 0.15m → REJECT ICP, keep EKF

**Fallback behavior:**
```python
if correction_too_large:
    fallback_count += 1
    # Keep EKF pose (already applied the delta)
else:
    x, y, theta = icp_result
    icp_success_count += 1
```

---

## Integration with EKF

### Two-Stage Odometry Pipeline

```
┌──────────────┐
│ Wheel + IMU  │
└──────┬───────┘
       │
       ▼
┌──────────────┐      ┌────────────┐
│ EKF Filter   │      │ LiDAR Scan │
└──────┬───────┘      └──────┬─────┘
       │                     │
       │  ┌──────────────────┘
       │  │
       ▼  ▼
┌─────────────────┐
│ ICP Refinement  │
└────────┬────────┘
         │
         ▼
  ┌────────────┐
  │ Final Pose │
  └────────────┘
```

### Why EKF First?

**EKF provides:**
1. **Initial guess** for ICP (within convergence basin)
2. **High-rate updates** (20 Hz) between scans (5 Hz)
3. **Fallback** when ICP fails
4. **Smoothness** in fast motion

**ICP provides:**
5. **Geometric accuracy** to correct EKF drift
6. **Absolute reference** from environment structure

### Relative Odometry Strategy

**Key idea:** Apply EKF delta, then refine with ICP.

```python
# 1. Compute EKF motion since last update
ekf_dx = ekf_x - prev_ekf_x
ekf_dy = ekf_y - prev_ekf_y
ekf_dtheta = ekf_theta - prev_ekf_theta

# 2. Apply to ICP pose
icp_x += ekf_dx
icp_y += ekf_dy
icp_theta += ekf_dtheta

# 3. Refine with ICP (only at keyframes)
if is_keyframe:
    icp_x, icp_y, icp_theta = run_icp(scan, map, icp_x, icp_y, icp_theta)
```

**Why relative?**
- EKF and ICP might have different scale/bias
- Relative motion is more consistent
- Avoids "jumps" when switching between methods

### Keyframe-Based ICP

**Not every scan triggers ICP!**

```python
dist_from_kf = sqrt((x - last_kf_x)² + (y - last_kf_y)²)
angle_from_kf = abs(theta - last_kf_theta)

if dist > 0.15m OR angle > 5°:
    # Run ICP
    # Add to local map
    # Update keyframe reference
```

**Benefits:**
1. **Efficiency**: ICP is expensive, run only when needed
2. **Redundancy**: Consecutive scans at same pose are redundant
3. **Map quality**: Keyframes from diverse poses improve map
4. **Temporal consistency**: Matches same reference for nearby poses

---

## Occupancy Grid Map Building

### Overview

In addition to refining odometry, the ICP node also builds a **2D occupancy grid map** using the corrected pose and LiDAR scans. This provides a visual representation of the environment similar to the SLAM map.

### Map Representation

The map is a 2D grid where each cell stores an occupancy probability:

```
Map parameters:
  resolution: 0.05 m/cell
  size: 200m x 200m (4000 x 4000 cells)
  origin: (-100, -100) meters

Cell values (int8):
  -1: Unknown (not yet observed)
   0: Free space (no obstacle)
  100: Occupied (obstacle present)
  1-99: Probability of occupancy
```

The map is maintained using **hit/miss counting**:
- `hit_count[cell]`: Number of times a LiDAR beam endpoint landed in this cell
- `miss_count[cell]`: Number of times a LiDAR beam passed through this cell
- `occupancy = hit_count / (hit_count + miss_count) * 100`

### Ray Tracing with Bresenham's Algorithm

For each LiDAR scan point, a ray is traced from the robot position to the scan endpoint:

```python
def update_map(self, scan_points):
    # Transform scan points to world frame using current pose
    world_points = transform_to_odom(scan_points, x, y, theta)

    # Convert robot position to grid coordinates
    robot_grid = world_to_grid(robot_x, robot_y)

    for point in world_points:
        point_grid = world_to_grid(point)

        # Trace ray from robot to point (Bresenham's line)
        cells = bresenham_line(robot_grid, point_grid)

        # All cells along ray (except endpoint) are FREE
        for cell in cells[:-1]:
            miss_count[cell] += 1

        # Endpoint cell is OCCUPIED
        hit_count[point_grid] += 1
```

**Bresenham's line algorithm** efficiently computes all grid cells along a ray between two points using only integer arithmetic.

### Map Saving

The map is saved as a **cropped PNG** image on shutdown:

1. **Threshold**: Cells with occupancy >= 50% are marked as occupied (black), < 50% as free (white), unknown as gray
2. **Crop**: The image is cropped to the bounding box of explored (non-unknown) cells with 20px padding, removing the large unused gray border
3. **Flip**: The image is flipped vertically to match the standard map orientation (Y-axis up)

```python
# Threshold for clean binary look
map_img[free] = 254       # white
map_img[occupied] = 0     # black
map_img[unknown] = 205    # gray

# Crop to explored area + padding
# ... bounding box detection + 20px pad ...
map_img = np.flipud(map_img)
```

### Map Publishing

The occupancy grid is published as a ROS `OccupancyGrid` message at 0.5 Hz on `/map_icp` for real-time visualization in RViz.

---

## Complete Algorithm Flow

### Per-Scan Processing

```
┌────────────────────────────────────────────────┐
│ 1. Receive LiDAR Scan + Latest EKF Odom        │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 2. Convert Scan to Point Cloud                 │
│    - Parse ranges and angles                   │
│    - Filter invalid (min/max range)            │
│    - Convert to Cartesian (body frame)         │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 3. Compute EKF Delta Motion                    │
│    Δx = ekf_x - prev_ekf_x                     │
│    Δy = ekf_y - prev_ekf_y                     │
│    Δθ = ekf_θ - prev_ekf_θ                     │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 4. Apply EKF Delta to ICP Pose                 │
│    icp_x += Δx                                 │
│    icp_y += Δy                                 │
│    icp_θ += Δθ                                 │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 5. Check if Keyframe Threshold Reached         │
│    dist = sqrt((x-kf_x)² + (y-kf_y)²)          │
│    angle = |θ - kf_θ|                          │
│                                                │
│    if dist > 0.15m OR angle > 5°:              │
│        ↓ YES: Run ICP                          │
│    else:                                       │
│        ↓ NO: Skip to step 10                   │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 6. Rebuild Local Map (if dirty)                │
│    - Merge all keyframe scans                  │
│    - Voxel downsample (0.05m)                  │
│    - Build KDTree                              │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 7. Run ICP: Scan-to-Map Alignment              │
│    Input: scan (body), map (odom), guess       │
│    Output: refined_x, refined_y, refined_θ     │
│                                                │
│    ICP Loop (max 50 iterations):               │
│      a) Transform scan with current pose       │
│      b) Find nearest neighbors in map          │
│      c) Filter by distance + trim 80%          │
│      d) SVD to compute optimal transform       │
│      e) Update pose estimate                   │
│      f) Check convergence                      │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 8. Validate ICP Result                         │
│    Δt = |refined_pose - ekf_pose|_translation  │
│    Δr = |refined_θ - ekf_θ|                    │
│                                                │
│    if Δt < 0.15m AND Δr < 2°:                  │
│        Accept: pose = refined_pose             │
│    else:                                       │
│        Reject: keep EKF pose                   │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 9. Update Local Map                            │
│    - Transform scan to odom frame              │
│    - Add to keyframe deque (sliding window)    │
│    - Mark map as dirty                         │
│    - Update keyframe reference                 │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 10. Update Occupancy Grid Map                  │
│     - Ray trace from robot to each scan point  │
│     - Mark ray cells as free, endpoint occupied│
│     - Update hit/miss counts                   │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 11. Save Trajectory Point                      │
│     trajectory.append([time, x, y, θ])         │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 12. Publish Odometry, Path & Map               │
│     - /odom_icp topic                          │
│     - TF: odom → base_footprint_icp            │
│     - /path_icp for visualization              │
│     - /map_icp occupancy grid (0.5 Hz)         │
└────────────┬───────────────────────────────────┘
             │
             ▼
┌────────────────────────────────────────────────┐
│ 13. Update Previous EKF Pose                   │
│     prev_ekf_x = ekf_x                         │
│     prev_ekf_y = ekf_y                         │
│     prev_ekf_θ = ekf_θ                         │
└────────────────────────────────────────────────┘
```

---

## Implementation Details

### Coordinate Frames

**Three frames in play:**

1. **Body frame** (`base_footprint`):
   - Origin: robot center
   - X-axis: forward, Y-axis: left
   - LiDAR scans are in this frame

2. **Odom frame** (`odom`):
   - Origin: robot starting position
   - Fixed orientation (usually aligned with world)
   - EKF and ICP both estimate pose in this frame

3. **Map frame** (`map`):
   - Used by SLAM in Part 3
   - Not relevant for ICP odometry

### Transformation: Body → Odom

Given robot pose `(x, y, θ)` in odom frame, transform body-frame point `(x_b, y_b)`:

```
Rotation:
  x_r = x_b cos(θ) - y_b sin(θ)
  y_r = x_b sin(θ) + y_b cos(θ)

Translation:
  x_o = x_r + x = x_b cos(θ) - y_b sin(θ) + x
  y_o = y_r + y = x_b sin(θ) + y_b cos(θ) + y
```

**Matrix form:**
```
[x_o]   [cos θ  -sin θ  x] [x_b]
[y_o] = [sin θ   cos θ  y] [y_b]
[1  ]   [0       0      1] [1  ]
```

### Sensor QoS for /scan

The `/scan` subscription uses `qos_profile_sensor_data` (BEST_EFFORT reliability) instead of the default (RELIABLE). This matches the QoS profile used by the bag publisher for sensor data, ensuring messages are received properly during bag playback.

```python
from rclpy.qos import qos_profile_sensor_data

self.scan_sub = self.create_subscription(
    LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
```

### Scan Conversion

**From polar (LiDAR) to Cartesian (body frame):**

```python
def scan_to_pointcloud(scan_msg):
    ranges = np.array(scan_msg.ranges)
    angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

    # Filter invalid measurements
    valid = ((ranges > scan_msg.range_min) &
             (ranges < scan_msg.range_max) &
             np.isfinite(ranges) &
             (ranges > 0.12))  # Remove very close points (robot body)

    ranges = ranges[valid]
    angles = angles[valid]

    # Polar to Cartesian
    x = ranges * np.cos(angles)
    y = ranges * np.sin(angles)

    return np.column_stack([x, y])
```

**Why filter?**
- `range_min/max`: Sensor valid range
- `isfinite`: Remove NaN/Inf from no-return
- `> 0.12`: Remove robot body reflections

### Angle Wrapping

Always wrap angles after arithmetic:

```python
theta = np.arctan2(np.sin(theta), np.cos(theta))
```

For angle differences:
```python
dtheta = ekf_theta - prev_ekf_theta
dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
```

### KDTree Usage

**Build tree:**
```python
from scipy.spatial import KDTree
tree = KDTree(map_points)  # O(n log n)
```

**Query nearest neighbors:**
```python
distances, indices = tree.query(query_points)  # O(m log n)
```

Returns:
- `distances[i]`: Distance to nearest neighbor of `query_points[i]`
- `indices[i]`: Index in `map_points` of that neighbor

**Example:**
```python
map_points = [[1.0, 2.0],
              [1.5, 2.5],
              [2.0, 3.0]]
tree = KDTree(map_points)

query = [[1.1, 2.1],
         [3.0, 4.0]]

dists, idx = tree.query(query)
# dists = [0.141, 1.414]
# idx = [0, 2]
# Meaning: query[0] is closest to map_points[0]
#          query[1] is closest to map_points[2]
```

### Deque for Sliding Window

```python
from collections import deque

local_map_scans = deque(maxlen=15)
```

**Behavior:**
- `append(item)`: Add to right
- When full (15 items), leftmost item is automatically removed
- Perfect for sliding window!

**Example:**
```python
q = deque(maxlen=3)
q.append('A')  # ['A']
q.append('B')  # ['A', 'B']
q.append('C')  # ['A', 'B', 'C']
q.append('D')  # ['B', 'C', 'D']  ← 'A' was removed
```

### Statistics Tracking

```python
_scan_count = 0          # Total scans received
_update_count = 0        # Total updates processed
_keyframe_count = 0      # Keyframes added to map
_icp_success_count = 0   # ICP accepted
_fallback_count = 0      # ICP rejected, used EKF
```

**Metrics:**
- **Success rate**: `icp_success / keyframe_count`
- **Fallback rate**: `fallback / keyframe_count`
- **Keyframe rate**: `keyframe_count / update_count`

Good performance: >90% ICP success rate

---

## Summary

### Algorithm Strengths

1. **Accuracy**: Geometric constraints from LiDAR reduce drift
2. **Robustness**: EKF fallback when ICP fails
3. **Efficiency**: Keyframe-based + downsampling = real-time performance
4. **Stability**: Relative odometry prevents jumps

### Compared to Methods

| Method | Drift | Computational Cost | Robustness |
|--------|-------|-------------------|------------|
| Wheel+IMU (EKF) | High (unbounded) | Low | Good |
| Scan-to-Scan ICP | Medium | Medium | Medium |
| **Scan-to-Map ICP** | **Low** | **Medium** | **Good** |
| Full SLAM | Very Low | High | Excellent |

### When ICP Works Well

✓ **Structured environments** (hallways, rooms)
✓ **Static scenes** (no moving objects)
✓ **Slow to moderate motion** (ICP assumes small incremental motion)
✓ **Good initial guess** (EKF provides this)

### When ICP Struggles

✗ **Featureless areas** (long empty corridors, open spaces)
✗ **Highly dynamic scenes** (crowds, moving furniture)
✗ **Very fast motion** (large frame-to-frame displacement)
✗ **Sensor occlusion** (LiDAR blocked)

In these cases, the algorithm falls back to EKF, maintaining robustness.

---

## References

1. **Besl & McKay (1992)** - "A Method for Registration of 3-D Shapes"
   - Original ICP paper

2. **Censi (2008)** - "An ICP variant using a point-to-line metric"
   - More advanced ICP for 2D LiDAR

3. **Segal et al. (2009)** - "Generalized-ICP"
   - Probabilistic framework for ICP

4. **Rusinkiewicz & Levoy (2001)** - "Efficient Variants of ICP"
   - Survey of ICP improvements

5. **Horn (1987)** - "Closed-form solution of absolute orientation using unit quaternions"
   - Mathematical basis for SVD alignment

---

## Appendix: Debugging Tips

### Poor ICP Performance

**Symptom:** High fallback rate, trajectory diverges from EKF

**Check:**
1. Is local map empty? Need at least 3 keyframes
2. Are correspondences found? Check `n_valid < min_correspondences`
3. Is voxel size too large? Try smaller (0.03m)
4. Is max_correspondence_distance too small? Try larger (0.7m)
5. Are scans noisy? Check sensor data quality

### ICP Jumps

**Symptom:** Sudden position jumps in trajectory

**Likely cause:** ICP converged to wrong local minimum

**Solutions:**
1. Decrease `max_translation_correction` (more conservative)
2. Increase keyframe threshold (run ICP less often)
3. Check if initial guess (EKF) has large error

### Slow Performance

**Symptom:** Cannot run in real-time (>200ms per scan)

**Solutions:**
1. Reduce `max_scan_points` (fewer than 300)
2. Increase `local_map_voxel_size` (more downsampling)
3. Reduce `local_map_size` (fewer keyframes)
4. Reduce `icp_max_iterations` (e.g., 30 instead of 50)

### Visualization in RViz

```
Add → Path → Topic: /path_ekf (red)
Add → Path → Topic: /path_icp (green)
Add → LaserScan → Topic: /scan
Add → TF
```

Compare EKF (red) vs ICP (green) trajectories visually!

---

**END OF DOCUMENT**
