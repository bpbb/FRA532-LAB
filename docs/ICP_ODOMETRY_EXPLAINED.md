# ICP Odometry

## Overview

Scan-to-local-map Iterative Closest Point (ICP) implementation for geometric odometry refinement using LiDAR. EKF odometry provides initial guess; ICP corrects pose using environment geometry.

## Algorithm Architecture

### Two-Stage Pipeline

1. **EKF baseline**: Compute EKF motion delta `(Δx, Δy, Δθ)` since last update
2. **Apply delta**: Update ICP pose with EKF motion
3. **Keyframe detection**: Trigger ICP refinement if moved >0.15m or rotated >5°
4. **ICP alignment**: Match current scan to local map
5. **Validation**: Accept ICP result if correction <0.15m translation and <2° rotation
6. **Map update**: Add keyframe scan to sliding window local map

### Local Map Management

Sliding window of last 15 keyframe scans in odom frame:
- Merged into single point cloud
- Voxel downsampled (0.05m grid) for efficiency
- KDTree built for O(log n) nearest neighbor queries
- Map rebuilt when dirty (new keyframe added)

Voxel downsampling reduces ~5400 points → ~800 points while preserving geometry.

## Point-to-Point ICP

### Iterative Loop

Maximum 50 iterations or until convergence (error change < 1e-6):

1. **Transform scan**: Apply current pose estimate to scan points
2. **Find correspondences**: KDTree nearest neighbor for each scan point
3. **Filter outliers**:
   - Distance threshold: reject if `d > 0.5m`
   - Minimum correspondences: require at least 30 valid matches
   - Trimming: keep best 80% of matches (robust estimation)
4. **Compute transformation**: SVD-based optimal pose
5. **Update estimate**: Apply computed transformation
6. **Check convergence**: Stop if mean error change negligible

### SVD-Based Pose Estimation (Kabsch Algorithm)

Given matched point sets `S` (source) and `T` (target):

**Center point clouds:**
```
s_c = (1/n) Σᵢ sᵢ
t_c = (1/n) Σᵢ tᵢ
s'ᵢ = sᵢ - s_c
t'ᵢ = tᵢ - t_c
```

**Cross-covariance matrix:**
```
W = Σᵢ t'ᵢ ⊗ s'ᵢ = (T_centered)ᵀ S_centered
```

**SVD decomposition:**
```
W = U Σ Vᵀ
```

**Extract rotation:**
```
R = U [[1, 0], [0, sign(det(UVᵀ))]] Vᵀ
```

**Extract translation:**
```
t = t_c - R s_c
```

**Convert to 2D pose:**
```
θ = arctan2(R[1,0], R[0,0])
x = t[0]
y = t[1]
```

## Robustness Features

### Correspondence Filtering

- **Max distance**: 0.5m (reject far matches)
- **Min correspondences**: 30 points minimum
- **Trimming**: use 80th percentile to reject worst 20%
- **Scan subsampling**: max 300 points for efficiency

### Safety Constraints

ICP correction limits (relative to EKF):
- Translation: `Δtrans < 0.15m`
- Rotation: `Δrot < 2°`

If exceeded, reject ICP and use EKF pose (fallback behavior).

### Relative Odometry Integration

```python
# Compute EKF delta
ekf_dx = ekf_x - prev_ekf_x
ekf_dy = ekf_y - prev_ekf_y
ekf_dθ = ekf_θ - prev_ekf_θ

# Apply to ICP pose
icp_x += ekf_dx
icp_y += ekf_dy
icp_θ += ekf_dθ

# Refine with ICP (keyframes only)
if is_keyframe:
    icp_x, icp_y, icp_θ = run_icp(...)
```

Prevents jumps and maintains consistency between EKF and ICP estimates.

## Performance Characteristics

**Strengths:**
- Consistent ~1m accuracy across environments
- Geometric constraints reduce drift
- EKF fallback ensures robustness
- Real-time capable with optimizations

**Limitations:**
- Drift accumulates without loop closure
- Struggles in featureless environments
- Requires good initial guess (EKF provides this)
- Performance degrades with smooth motion (weak geometric constraints)