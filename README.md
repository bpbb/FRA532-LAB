# FRA532 LAB1: Kalman Filter / SLAM

Localization pipeline for TurtleBot3 Burger using ROS2 Humble. Progressively integrates sensor fusion (EKF), scan matching (ICP), and SLAM to estimate the robot's pose from recorded rosbag data.

## Project Structure

```
FRA532-LAB1/
├── src/lab1/
│   ├── scripts/
│   │   ├── wheel_odometry.py       # Baseline: differential-drive wheel odometry
│   │   ├── ekf_odometry.py         # Part 1: EKF sensor fusion (wheel + IMU)
│   │   ├── icp_odometry.py         # Part 2: ICP scan-to-local-map refinement
│   │   ├── slam.py                 # Part 3: SLAM trajectory recorder + TF publisher
│   │   └── analyze_trajectories.py # Trajectory comparison plots and metrics
│   ├── launch/
│   │   ├── part1_complete.launch.py  # Part 1: wheel + EKF
│   │   ├── part2_complete.launch.py  # Part 2: wheel + EKF + ICP
│   │   ├── part3_complete.launch.py  # Part 3: wheel + EKF + ICP + SLAM
│   │   └── plot_analysis.launch.py   # Generate comparison plots
│   ├── config/
│   │   └── slam_toolbox_params.yaml  # slam_toolbox configuration
│   └── rviz/
│       ├── part1.rviz
│       ├── part2.rviz
│       └── part3.rviz
├── FRA532_LAB1_DATASET/
│   ├── fibo_floor3_seq00/            # Empty hallway
│   ├── fibo_floor3_seq01/            # Sharp turns with obstacles
│   └── fibo_floor3_seq02/            # Smooth motion with obstacles
└── results/
    ├── trajectory_wheel_seqXX.txt
    ├── trajectory_ekf_seqXX.txt
    ├── trajectory_icp_seqXX.txt
    ├── trajectory_slam_seqXX.txt
    ├── map_icp_seqXX.png
    ├── map_slam_seqXX.png
    ├── discussion.md
    └── analysis/
```

## Robot Parameters

| Parameter | Value |
|-----------|-------|
| Platform | TurtleBot3 Burger |
| Wheel radius | 0.033 m |
| Wheel base | 0.160 m |
| LiDAR | LDS-01, 360 deg, 5 Hz |
| IMU | 20 Hz (gyro + accel + orientation) |
| Wheel encoders | 20 Hz (via `/joint_states`) |

---

## Part 1: EKF Odometry Fusion

### Overview

Fuses wheel odometry (`/joint_states`) with IMU measurements (`/imu`) using an Extended Kalman Filter to produce a filtered odometry estimate with reduced heading drift.

### EKF Design

**State vector:** `[x, y, theta, b_g]`
- `x, y, theta` - robot pose in odom frame
- `b_g` - gyroscope bias (estimated online)

**Prediction step:**
- IMU angular velocity (bias-corrected) drives heading: `theta += (omega_imu - b_g) * dt`
- Wheel encoders provide linear velocity via differential-drive model

**Update step (dual measurement):**
1. **Wheel angular velocity** - cross-checks IMU heading rate
2. **IMU orientation** - absolute heading correction (when available)

**Key features:**
- Velocity-dependent process noise (Thrun's alpha model from *Probabilistic Robotics*)
- Online gyro bias estimation via state augmentation
- Mahalanobis distance gating for outlier rejection
- Joseph form covariance update for numerical stability

### Topics

| Direction | Topic | Type | Rate |
|-----------|-------|------|------|
| Subscribe | `/joint_states` | `sensor_msgs/JointState` | 20 Hz |
| Subscribe | `/imu` | `sensor_msgs/Imu` | 20 Hz |
| Publish | `/odom_ekf` | `nav_msgs/Odometry` | 20 Hz |
| Publish | `/path_ekf` | `nav_msgs/Path` | 20 Hz |
| Publish | `/tf` (`odom` -> `base_footprint_ekf`) | TF | 20 Hz |

### How to Run

```bash
cd ~/FRA532-LAB1
source install/setup.bash

ros2 launch lab1 part1_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq00
```

Replace `seq00` with `seq01` or `seq02` for other sequences.

---

## Part 2: ICP Odometry Refinement

### Overview

Refines EKF odometry using LiDAR scan matching. Matches each scan against a **local map** built from recent keyframes, using SVD-based ICP to correct the robot pose. Also builds and saves a 2D occupancy grid map.

### Algorithm Design

**Scan-to-Local-Map ICP** (not scan-to-scan):

1. **EKF baseline** - Apply the EKF delta (global pose difference) to advance the pose. This is the verified baseline from Part 1.

2. **Keyframe detection** - When the robot moves > 0.15m or rotates > 5 deg from the last keyframe, trigger an ICP refinement.

3. **Local map** - Maintains a sliding window of the last 15 keyframe scans, merged and voxel-downsampled (5cm grid) for efficiency.

4. **ICP matching** - Transform the current scan to the odom frame using the current pose estimate, find nearest-neighbor correspondences in the local map via KDTree, then solve for the optimal pose via SVD.

5. **Safety constraints** - Reject ICP corrections larger than 15cm translation or 2 deg rotation to prevent bad matches from accumulating.

**Key features:**
- Local map provides richer geometry than single previous scan
- Voxel grid downsampling prevents map from growing unbounded
- Trimmed ICP (best 80% of correspondences) for robustness
- SVD-based alignment with reflection check (`det(R) > 0`)
- Sensor QoS (`BEST_EFFORT`) for `/scan` subscription to match bag publisher
- Occupancy grid map built via ray tracing with Bresenham's line algorithm

### Topics

| Direction | Topic | Type | Rate |
|-----------|-------|------|------|
| Subscribe | `/scan` | `sensor_msgs/LaserScan` | 5 Hz |
| Subscribe | `/odom_ekf` | `nav_msgs/Odometry` | 20 Hz |
| Publish | `/odom_icp` | `nav_msgs/Odometry` | 5 Hz |
| Publish | `/path_icp` | `nav_msgs/Path` | 5 Hz |
| Publish | `/map_icp` | `nav_msgs/OccupancyGrid` | 0.5 Hz |
| Publish | `/tf` (`odom` -> `base_footprint_icp`) | TF | 5 Hz |

### How to Run

```bash
cd ~/FRA532-LAB1
source install/setup.bash

ros2 launch lab1 part2_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq00
```

---

## Part 3: Full SLAM with slam_toolbox

### Overview

Uses `slam_toolbox` (online async mode) to perform graph-based SLAM with loop closure. Builds a 2D occupancy grid map while estimating the robot's pose in the `map` frame. Compares the SLAM trajectory against wheel, EKF, and ICP odometry.

### Architecture

1. **EKF as odometry source** - slam_toolbox uses the EKF odometry (`odom -> base_footprint_ekf` TF) as its motion model initial guess.

2. **Dynamic TF** - The `slam.py` node periodically publishes `base_footprint_ekf -> base_scan` (TurtleBot3 Burger laser offset) since the dataset bags contain no TF data. Published as dynamic TF (not static) to survive TF buffer clears caused by sim time clock jumps during bag playback.

3. **slam_toolbox** - Runs in online async mapping mode. Performs scan matching and loop closure to produce:
   - `map -> odom` TF (SLAM correction)
   - `/map` occupancy grid

4. **Trajectory recorder** - Looks up `map -> base_footprint_ekf` TF at 10 Hz and records the SLAM-corrected pose to file. Saves the SLAM map as PNG on shutdown.

### How to Run

```bash
cd ~/FRA532-LAB1
source install/setup.bash

ros2 launch lab1 part3_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq00
```

---

## Plot & Analysis

After running sequences through Part 1/2/3, trajectory files and maps are saved to `results/`. Generate comparison plots and metrics:

```bash
cd ~/FRA532-LAB1
source install/setup.bash

# Single sequence
ros2 launch lab1 plot_analysis.launch.py sequences:="00"

# All sequences
ros2 launch lab1 plot_analysis.launch.py sequences:="00 01 02"
```

Output is saved to `results/analysis/`.

---

## Run All Sequences

```bash
cd ~/FRA532-LAB1
source install/setup.bash

# Part 3 - Sequence 00
ros2 launch lab1 part3_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq00

# Part 3 - Sequence 01
ros2 launch lab1 part3_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq01

# Part 3 - Sequence 02
ros2 launch lab1 part3_complete.launch.py \
    bag_path:=$HOME/FRA532-LAB1/FRA532_LAB1_DATASET/fibo_floor3_seq02

# Plot & analyze all sequences
ros2 launch lab1 plot_analysis.launch.py sequences:="00 01 02"
```

## Dependencies

- ROS2 Humble
- Python 3.10+
- `numpy`, `scipy`, `matplotlib`, `pandas`, `Pillow`
- `tf_transformations`
- `slam_toolbox` (Part 3)
