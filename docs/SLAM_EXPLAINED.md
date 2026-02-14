# SLAM with slam_toolbox - Complete Explanation

## Table of Contents
1. [Overview](#overview)
2. [What is SLAM?](#what-is-slam)
3. [Graph-Based SLAM](#graph-based-slam)
4. [Loop Closure](#loop-closure)
5. [slam_toolbox Architecture](#slam_toolbox-architecture)
6. [Trajectory Recorder Implementation](#trajectory-recorder-implementation)
7. [Comparison with ICP Odometry](#comparison-with-icp-odometry)
8. [Complete Workflow](#complete-workflow)

---

## Overview

### Part 3 Objective

Use **slam_toolbox** (a ROS2 SLAM system) to:
1. Build a map of the environment
2. Localize the robot within that map
3. Detect and close loops to reduce drift
4. Compare trajectory with wheel/EKF/ICP odometry

### What You Implement

**NOT implementing SLAM from scratch!** Instead:
- Run `slam_toolbox` (pre-built ROS2 package)
- Record the resulting trajectory via TF lookup
- Compare with odometry methods from Parts 1-2

This is realistic: in practice, you use existing SLAM systems and integrate them into your pipeline.

---

## What is SLAM?

### SLAM Definition

**Simultaneous Localization And Mapping**

**The problem:**
- Robot starts in unknown environment
- No GPS, no prior map
- Must build map AND localize within it simultaneously

**Chicken-and-egg paradox:**
- To build a map, you need to know where you are (localization)
- To localize, you need a map
- SLAM solves both at the same time!

### SLAM vs Odometry

| Aspect | Odometry | SLAM |
|--------|----------|------|
| **Map** | No map built | Builds map |
| **Drift** | Unbounded accumulation | Bounded by loop closure |
| **Complexity** | Low | High |
| **Computation** | Real-time | Can be heavy |
| **Output** | Pose in odom frame | Pose + map in map frame |

### Why SLAM is Better

**Odometry:**
```
Start → → → → → → → → End
        ↓ drift
        (unbounded error accumulation)
```

**SLAM with Loop Closure:**
```
Start → → → → → → → → End
  ↑                     ↓
  ← ← ← ← ← ← ← ← ← ← ←
        (loop detected!)

        ↓
Optimize entire trajectory to close loop
        ↓
Bounded error!
```

---

## Graph-Based SLAM

### Pose Graph Representation

SLAM represents the trajectory as a **graph**:

**Nodes:** Robot poses at different times
```
x₀ → x₁ → x₂ → x₃ → ... → xₙ
```

**Edges:** Constraints between poses

1. **Odometry edges** (sequential):
   - Connect consecutive poses
   - From EKF/ICP: "from x₁ to x₂, I moved Δx, Δy, Δθ"
   - Uncertainty: covariance matrix

2. **Loop closure edges** (non-sequential):
   - Connect distant poses when robot returns to same place
   - From scan matching: "x₁₀₀ is the same place as x₅"
   - Provides constraint to reduce drift

### Example Graph

```
Trajectory with loop:

    x₀ → x₁ → x₂
           ↓
    x₅ ← x₄ ← x₃
    ↓
    x₆ → x₇ → x₈

Edges:
  Odometry: x₀→x₁, x₁→x₂, x₂→x₃, x₃→x₄, x₄→x₅, x₅→x₆, x₆→x₇, x₇→x₈
  Loop closure: x₈→x₁ (robot returned to near x₁)

Before loop closure:
  - Drift accumulates
  - x₈ might be far from x₁ even though same place

After loop closure optimization:
  - Graph is adjusted to satisfy all constraints
  - x₈ and x₁ align correctly
  - Entire trajectory is corrected!
```

### Optimization Problem

**Objective:** Find poses that best satisfy all constraints.

Minimize:
```
Σ (edge error)²

Where edge error = measured_transform - predicted_transform
```

**Mathematically:**
```
x* = argmin Σᵢⱼ ||h(xᵢ, xⱼ) - zᵢⱼ||²_Σᵢⱼ

Where:
- xᵢ, xⱼ: poses i and j
- zᵢⱼ: measured relative pose (from odometry or scan matching)
- h(xᵢ, xⱼ): expected relative pose given current estimates
- Σᵢⱼ: uncertainty (covariance) of measurement
```

**Intuition:**
- If odometry says "moved 1m" but loop closure says "same place", there's conflict
- Optimization distributes error across all poses to best satisfy constraints
- More certain measurements (small Σ) have more influence

### Graph Optimization Algorithms

**Common solvers:**
1. **Gauss-Newton** - Iterative linearization
2. **Levenberg-Marquardt** - Gauss-Newton with damping
3. **g2o** - General graph optimization (used by slam_toolbox)
4. **iSAM2** - Incremental smoothing and mapping
5. **Ceres** - Google's optimization library

slam_toolbox uses **Karto** backend with **Ceres** solver (configured in our `slam_toolbox_params.yaml`).

---

## Loop Closure

### What is a Loop?

**Loop:** Robot returns to a previously visited location.

**Example:**
```
Robot path:
  Walk down hallway (poses 0-50)
  Turn around
  Walk back (poses 51-100)

At pose 100, robot is near pose 0 → LOOP!
```

### Loop Closure Detection

**How to detect loops?**

1. **Scan matching:**
   - Compare current LiDAR scan with previous scans
   - If very similar → likely same place
   - ICP to estimate relative pose

2. **Heuristics:**
   - Only check scans that are far apart in time (>10 seconds)
   - But close in space (estimated distance < threshold)
   - Prevents matching similar-looking but different places

3. **Verification:**
   - Check alignment quality (number of correspondences, error)
   - Reject false positives

### Loop Closure Workflow

```
┌──────────────────────────────────────────────┐
│ 1. New scan arrives                          │
└────────────┬─────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────┐
│ 2. Search for loop candidates                │
│    - Poses far in time (>10s)                │
│    - Close in space (<3m)                    │
└────────────┬─────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────┐
│ 3. For each candidate, run scan matching     │
│    - ICP current scan vs candidate scan      │
│    - Compute relative pose + uncertainty     │
└────────────┬─────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────┐
│ 4. Verify match quality                      │
│    - Check alignment error                   │
│    - Check number of correspondences         │
│    - If good: add loop closure edge          │
└────────────┬─────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────┐
│ 5. Optimize pose graph                       │
│    - Include new loop closure constraint     │
│    - Adjust all poses to satisfy constraints │
│    - Update map                              │
└──────────────────────────────────────────────┘
```

### Impact of Loop Closure

**Before loop closure:**
```
Ground truth:      Estimated (with drift):

    ┌─────┐           ┌─────┐
    │     │           │     │
    │     │           │     └──── Drift
    │     │           │        ∖
    └─────┘           └─────────┘

Loop error: 0.5m
```

**After loop closure:**
```
Optimized:

    ┌─────┐
    │     │
    │     │  ← Entire trajectory adjusted
    │     │
    └─────┘

Loop error: 0.02m
```

The drift is distributed backwards through the trajectory!

---

## slam_toolbox Architecture

### System Components

```
┌─────────────┐
│ LiDAR       │──┐
│ /scan       │  │
└─────────────┘  │
                 │
┌─────────────┐  │    ┌──────────────────┐
│ EKF TF:     │  │    │                  │
│ odom →      │──┼───▶│  slam_toolbox    │
│ base_foot-  │  │    │                  │
│ print_ekf   │  │    └────────┬─────────┘
└─────────────┘  │             │
                               │
    ┌──────────────────────────┼───────────────────┐
    │                          │                   │
    ▼                          ▼                   ▼
┌─────────┐              ┌──────────┐       ┌──────────┐
│ /map    │              │ TF:      │       │ /pose    │
│ (grid)  │              │ map→odom │       │ (SLAM    │
└─────────┘              └──────────┘       │ estimate)│
                                            └──────────┘
```

**Note:** slam_toolbox uses the TF tree for odometry input (configured via `base_frame: base_footprint_ekf` and `odom_frame: odom`), not the `/odom_ekf` topic. The EKF node publishes the `odom -> base_footprint_ekf` transform which slam_toolbox reads via TF.

### Inputs

1. **`/scan`** (sensor_msgs/LaserScan):
   - LiDAR measurements
   - Used for scan matching and mapping
   - 5 Hz (LDS-01 on TurtleBot3 Burger)

2. **TF: `odom -> base_footprint_ekf`**:
   - Published by the EKF node from Part 1
   - slam_toolbox reads this via TF (configured as `base_frame: base_footprint_ekf`)
   - Provides the odometry motion model for initial scan matching guess

### Outputs

1. **`/map`** (nav_msgs/OccupancyGrid):
   - 2D occupancy grid map
   - Cell values: 0=free, 100=occupied, -1=unknown
   - Updated incrementally as robot explores

2. **TF: `map -> odom`**:
   - The SLAM correction transform
   - Compensates for odometry drift
   - Combined with EKF TF: `map -> odom -> base_footprint_ekf` gives the full SLAM pose

### Internal Pipeline

```
New Scan + TF (odom → base_footprint_ekf)
   │
   ▼
┌────────────────────┐
│ Scan Matching      │ ← Match against local map
│ (correlative)      │   using TF-based odometry as initial guess
└─────────┬──────────┘
          │
          ▼
┌────────────────────┐
│ Add Node to Graph  │ ← Create new pose node
│                    │   with odometry edge
└─────────┬──────────┘
          │
          ▼
┌────────────────────┐
│ Loop Search        │ ← Check if near previous poses
│                    │
└─────────┬──────────┘
          │
          ├─ No loop ──────────┐
          │                    │
          ▼                    │
┌────────────────────┐         │
│ Loop Closure       │         │
│ Scan Matching      │         │
└─────────┬──────────┘         │
          │                    │
          ▼                    │
┌────────────────────┐         │
│ Graph Optimization │         │
│ (g2o/Ceres)        │         │
└─────────┬──────────┘         │
          │                    │
          └────────┬───────────┘
                   │
                   ▼
          ┌────────────────┐
          │ Update Map     │
          │ Publish TF     │
          └────────────────┘
```

### Configuration Parameters

Key parameters in slam_toolbox:

```yaml
# Frames
odom_frame: odom
map_frame: map
base_frame: base_footprint_ekf    # Uses EKF TF for odometry

# Scan matching
minimum_travel_distance: 0.3      # Min distance before new scan
minimum_travel_heading: 0.3       # Min rotation before new scan

# Loop closure
loop_search_maximum_distance: 3.0 # Search radius for loops
do_loop_closing: true

# Optimization
solver_plugin: solver_plugins::CeresSolver
ceres_trust_strategy: LEVENBERG_MARQUARDT

# Map
resolution: 0.02  # Map grid cell size (meters)
max_laser_range: 2.5  # LDS-01 max useful range
```

---

## Trajectory Recorder Implementation

### Purpose

slam_toolbox doesn't save trajectory to file automatically. Our `slam.py` node (class `Slam`):
1. Looks up TF: `map → base_footprint_ekf`
2. Records (timestamp, x, y, θ)
3. Publishes `/path_slam` for visualization
4. Saves to file on shutdown

### Dynamic TF: base_footprint_ekf -> base_scan

The `slam.py` node periodically publishes the `base_footprint_ekf -> base_scan` transform that slam_toolbox needs to relate the LiDAR to the robot base:

```python
# TurtleBot3 Burger laser offset:
# base_footprint->base_link (z=0.010) + base_link->base_scan (x=-0.032, z=0.172)
# Combined: x=-0.032, y=0, z=0.182
t.header.frame_id = 'base_footprint_ekf'
t.child_frame_id = 'base_scan'
t.transform.translation.x = -0.032
t.transform.translation.z = 0.182
```

**Why dynamic (not static)?** During bag playback with `--clock`, sim time can jump, causing the TF buffer to clear. A static TF published once would be lost. By publishing periodically (every 0.1s), the transform survives these clock jumps.

### TF Lookup

**What is TF?**
- ROS2 Transform library
- Maintains tree of coordinate frame relationships
- Can lookup transformation between any two frames

**Our lookup:**
```python
transform = tf_buffer.lookup_transform(
    'map',               # Target frame
    'base_footprint_ekf', # Source frame
    rclpy.time.Time()    # Latest available
)
```

**Returns:**
- Translation: (x, y, z)
- Rotation: quaternion (x, y, z, w)

**Timing:** 10 Hz timer callback (every 0.1 seconds)

### Extract Pose from TF

```python
# Position
x = transform.translation.x
y = transform.translation.y

# Orientation (quaternion → yaw)
q = transform.rotation
siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
theta = math.atan2(siny_cosp, cosy_cosp)
```

**Why quaternion math?**
- TF uses quaternions for 3D rotations (no gimbal lock)
- We only care about yaw (2D heading)
- Formula extracts yaw from quaternion

**Simplified for 2D:**
Could also use `tf_transformations.euler_from_quaternion()` but we avoid the dependency.

### Path Publishing

```python
pose = PoseStamped()
pose.header.frame_id = 'map'
pose.pose.position.x = x
pose.pose.position.y = y
pose.pose.orientation = transform.rotation

path_msg.poses.append(pose)
path_pub.publish(path_msg)
```

**Visualization in RViz:**
- Add → Path → Topic: `/path_slam`
- Shows SLAM trajectory in real-time

### Data Saving

```python
trajectory.append([time_sec, x, y, theta])

# On shutdown:
np.savetxt(output_file, trajectory,
           fmt='%.6f',
           header='timestamp x y theta')
```

**Output format:** Same as EKF and ICP for easy comparison!

### Map Subscription

The node subscribes to `/map` with **Transient Local** durability QoS, which ensures it receives the latest map even if the subscription starts after slam_toolbox has already published:

```python
map_qos = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE
)
self.map_sub = self.create_subscription(OccupancyGrid, '/map', ..., map_qos)
```

The latest map is saved as a PNG image on shutdown.

### Error Handling

```python
try:
    transform = tf_buffer.lookup_transform(...)
except (LookupException, ConnectivityException, ExtrapolationException):
    pass  # TF not ready yet, try again next time
```

**Common exceptions:**
- `LookupException`: Transform doesn't exist
- `ConnectivityException`: Frames not connected
- `ExtrapolationException`: Requested time not available

Early in execution, slam_toolbox hasn't published TF yet, so exceptions are normal.

---

## Comparison with ICP Odometry

### Similarities

Both ICP odometry and SLAM:
- Use LiDAR scan matching
- Build local maps
- Refine odometry with geometric constraints

### Key Differences

| Aspect | ICP Odometry (Part 2) | SLAM (Part 3) |
|--------|----------------------|---------------|
| **Map scope** | Local (15 keyframes) | Global (entire environment) |
| **Loop closure** | No | Yes |
| **Drift** | Reduced but unbounded | Bounded by loops |
| **Optimization** | Local (current pose) | Global (entire graph) |
| **Complexity** | Medium | High |
| **Output** | Odometry (`/odom_icp`) | Map + Pose (`/map`, TF) |

### When to Use Each

**ICP Odometry:**
- Need real-time odometry
- No loops in trajectory (straight paths)
- Computational constraints
- Odometry is sufficient for task

**SLAM:**
- Building a map for navigation
- Long missions with loops
- Drift must be minimized
- Have computational resources

**In practice:** Often use both!
- ICP odometry for local navigation
- SLAM for global localization and mapping

---

## Complete Workflow

### System Setup

All nodes are launched together via a single launch file that handles bag playback, all odometry nodes, slam_toolbox, the trajectory recorder, and RViz:

```bash
cd ~/FRA532-LAB1
source install/setup.bash

ros2 launch lab1 part3_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq00
```

The launch file starts:
- Wheel odometry node
- EKF odometry node
- ICP odometry node
- slam_toolbox (async mode)
- `slam.py` trajectory recorder (publishes TF, records trajectory, saves map)
- RViz (delayed 3s to allow TF frames to be published)
- Bag player (with `--clock 200` for sim time)

When the bag finishes, all nodes shut down automatically and save their outputs.

### Data Flow

```
ROS Bag
  │
  ├─ /scan ──────────────────────┐
  │                              │
  ├─ /imu ─────┐                 │
  │            │                 │
  └─ /joint_states               │
                │                │
                ▼                │
          ┌──────────────┐       │
          │ EKF Node     │       │
          └──────┬───────┘       │
                 │               │
                 │ TF: odom →    │
                 │ base_footprint│
                 │ _ekf          │
                 ▼               │
          ┌─────────────┐        │
          │ slam.py     │────────┤ TF: base_footprint
          │ (TF pub)    │        │      _ekf → base_scan
          └──────┬──────┘        │
                 │               │
                 ▼               ▼
          ┌──────────────────────────┐
          │ slam_toolbox             │
          │ (reads TF + /scan)       │
          └──────┬───────────────────┘
                 │
                 ├─ /map (OccupancyGrid)
                 └─ TF: map → odom
                          │
                          ▼
                   ┌──────────────────┐
                   │ slam.py          │
                   │ (TF lookup +     │
                   │  trajectory rec) │
                   └──────┬───────────┘
                          │
                          ├─ /path_slam
                          ├─ trajectory_slam_seqXX.txt
                          └─ map_slam_seqXX.png
```

### Analysis Pipeline

After running all sequences through Part 3, trajectory files are saved to `results/`. Use the analysis launch file to generate comparison plots:

```bash
ros2 launch lab1 plot_analysis.launch.py sequences:="00 01 02"
```

This runs `analyze_trajectories.py` which loads all four trajectory files per sequence (`trajectory_wheel_seqXX.txt`, `trajectory_ekf_seqXX.txt`, `trajectory_icp_seqXX.txt`, `trajectory_slam_seqXX.txt`) and generates:
- 2D trajectory comparison plots
- X and Y vs time plots
- Position error vs SLAM reference
- Metrics bar charts (mean error vs SLAM, heading drift, trajectory length)

Output is saved to `results/analysis/`.

**Expected results:**
- **Wheel (orange)**: Smooth but drifts significantly over time
- **EKF (blue)**: Reduced heading drift, but translational drift remains
- **ICP (green)**: Less drift than EKF, corrected by scan matching
- **SLAM (red)**: Minimal drift, loops close properly

---

## Map Visualization

### Occupancy Grid Format

slam_toolbox publishes **nav_msgs/OccupancyGrid**:

```
Map:
  - resolution: 0.05 m/cell
  - width: 4000 cells
  - height: 4000 cells
  - origin: (-100, -100) meters

Data: array of int8 [-1, 100]
  - -1: Unknown (not observed)
  -  0: Free space
  - 100: Occupied (obstacle)
  - 1-99: Probability (for probabilistic mapping)
```

### In RViz

```
Add → Map → Topic: /map

Settings:
  - Color Scheme: map (black=occupied, white=free)
  - Alpha: 0.7
```

You'll see the environment map being built in real-time!

### Saving Map

The `slam.py` node saves the SLAM map as a **PNG image** on shutdown by subscribing to the `/map` topic and converting the OccupancyGrid:

```
OccupancyGrid values → Image pixels:
  -1 (unknown)  → 205 (gray)
   0 (free)     → 254 (white)
 100 (occupied) → 0   (black)
 1-99           → scaled grayscale
```

Output: `results/map_slam_seqXX.png`

---

## Performance Metrics

### 1. Loop Closure Error

**Definition:** Distance between loop endpoints

For a trajectory that returns to start:
```python
final_pos = slam[-1, 1:3]
initial_pos = slam[0, 1:3]
loop_error = np.linalg.norm(final_pos - initial_pos)
```

**Good SLAM:** Loop error < 0.1m

### 2. Trajectory Length

```python
total_distance = 0
for i in range(1, len(trajectory)):
    dx = trajectory[i, 1] - trajectory[i-1, 1]
    dy = trajectory[i, 2] - trajectory[i-1, 2]
    total_distance += np.sqrt(dx**2 + dy**2)
```

Compare across methods - should be similar if all correct.

### 3. Map Quality (Visual Inspection)

**Good map characteristics:**
- Walls are straight and thin
- Corners are sharp
- Repeated structures align (e.g., doors in hallway)
- No "ghosting" (double walls)

**Bad map indicates:**
- Poor loop closure
- Incorrect scan matching
- Odometry drift

### 4. Number of Loop Closures

Check slam_toolbox output:
```
[INFO] Loop closure detected: node 150 to node 30
[INFO] Loop closure detected: node 300 to node 45
...
```

More loop closures (in loopy trajectory) = better!

---

## Troubleshooting

### No TF Published

**Symptom:** Trajectory recorder finds no transform

**Cause:** slam_toolbox not running or hasn't initialized

**Fix:**
1. Check slam_toolbox is running: `ros2 node list`
2. Check TF tree: `ros2 run tf2_tools view_frames`
3. Wait a few seconds for initialization

### Poor Map Quality

**Symptom:** Map has double walls, misaligned features

**Causes:**
1. **Bad odometry input:** EKF has large errors
2. **No loop closures:** Check slam_toolbox params
3. **Fast motion:** LiDAR can't keep up

**Fixes:**
1. Verify EKF is working (check Part 1)
2. Enable loop closure: `do_loop_closing: true`
3. Reduce playback speed: `ros2 bag play --rate 0.5`

### High Computational Load

**Symptom:** Real-time factor < 1.0, messages dropped

**Causes:**
1. Too many particles (particle filter mode)
2. Frequent optimization
3. Large map size

**Fixes:**
1. Use asynchronous mode: `online_async_launch.py`
2. Reduce scan rate: subsample `/scan` topic
3. Increase `minimum_travel_distance`

---

## Summary

### What You Learned

1. **Graph-based SLAM**: Pose graph representation and optimization
2. **Loop closure**: Detection and its impact on drift
3. **slam_toolbox**: Industry-standard ROS2 SLAM system
4. **TF framework**: Looking up transforms between frames
5. **Comparison**: EKF vs ICP vs SLAM performance

### Key Takeaways

✓ **SLAM eliminates unbounded drift** through loop closure
✓ **Graph optimization** distributes error across entire trajectory
✓ **Maps enable navigation** beyond just localization
✓ **Trade-off:** Complexity and computation vs accuracy

### Real-World Applications

- **Warehouse robots:** Map building for autonomous navigation
- **Vacuum cleaners:** Efficient coverage planning
- **Delivery robots:** Long-term operation in buildings
- **Autonomous cars:** Local mapping in parking lots

---

## References

1. **Thrun, Burgard, Fox** - "Probabilistic Robotics"
   - Chapter 11: SLAM
   - Chapter 13: Graph-based SLAM

2. **Konolige et al. (2010)** - "Efficient Sparse Pose Adjustment for 2D mapping"
   - Karto SLAM (backend used by slam_toolbox)

3. **Hess et al. (2016)** - "Real-time loop closure in 2D LIDAR SLAM"
   - Google Cartographer

4. **Grisetti et al. (2010)** - "A Tutorial on Graph-Based SLAM"
   - Mathematical foundations

5. **slam_toolbox documentation:**
   - https://github.com/SteveMacenski/slam_toolbox

---

## Appendix: Advanced Topics

### Backend Options

slam_toolbox supports multiple solvers:

1. **Ceres Solver** (Google):
   - Fast, robust
   - Automatic differentiation
   - Recommended for most use cases

2. **G2O** (General Graph Optimization):
   - Classic solver
   - Many backends (Cholmod, CSparse)
   - Very mature

3. **SPA** (Sparse Pose Adjustment):
   - Original Karto backend
   - Simple, efficient for 2D

### Localization Mode

After mapping, can switch to **localization-only** mode:

```yaml
mode: localization  # instead of mapping
```

**Benefits:**
- Localize in pre-built map
- No map updates (faster)
- Suitable for deployment after mapping phase

### Serialization

Save/load pose graph:

```bash
# Save
ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/path/to/map'}"

# Load
ros2 service call /slam_toolbox/deserialize_map \
    slam_toolbox/srv/DeserializePoseGraph \
    "{filename: '/path/to/map'}"
```

Useful for:
- Continuing mapping later
- Localization in known map
- Map sharing between robots

---

**END OF DOCUMENT**
