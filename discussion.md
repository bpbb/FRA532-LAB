# Discussion: Comparing Odometry and SLAM Methods

## Overview

Four localization methods were evaluated across three sequences recorded on a TurtleBot3 Burger in indoor hallway environments. **SLAM (slam_toolbox with loop closure) is treated as the best available reference since it uses graph-based optimization**.

| Sequence | Environment | Key Challenge |
|----------|-------------|---------------|
| 00 | Empty hallway, no obstacles | Featureless walls  |
| 01 | Obstacles with sharp turns | Rapid heading changes stress IMU fusion |
| 02 | Obstacles with smooth motion | Longer trajectory, tests drift accumulation |

---

## Accuracy (Position Error vs SLAM)

| Method | Seq 00 Mean (m) | Seq 01 Mean (m) | Seq 02 Mean (m) |
|--------|----------------|----------------|----------------|
| Wheel  | 3.560          | 2.720          | 2.780          |
| EKF    | 2.930          | 0.640          | 2.620          |
| ICP    | 0.990          | 0.990          | 1.170          |


<img src="results/analysis/metrics_comparison_seq00.png" alt="Metrics Comparison - Sequence 00" width="80%"/>

**Other Sequences:**
- [Metrics Comparison - Sequence 01](results/analysis/metrics_comparison_seq01.png)
- [Metrics Comparison - Sequence 02](results/analysis/metrics_comparison_seq02.png)

**Observations:**

- **ICP is the most consistently accurate** method across all three sequences, with mean error vs SLAM staying around 1.0m. Looking at the trajectory comparison plots below and the error time series, ICP (green line) maintains relatively stable error throughout each sequence. The error histograms show ICP has the tightest distribution, with most errors concentrated below 1.5m. The LiDAR scan matching provides continuous position corrections that prevent drift from accumulating unbounded.

- **EKF shows highly variable performance** depending on motion profile:
  - **Sequence 01 (sharp turns):** EKF achieves its best performance (0.64m mean error) - visible in the metrics plot as the blue line staying near zero error especially during turn maneuvers. The IMU gyroscope excels at correcting heading during rapid rotations, which is the dominant error source in this sequence.
  - **Sequences 00 & 02 (straighter motion):** EKF provides only modest improvement over wheel odometry (2.93m and 2.62m vs 3.56m and 2.78m). The error time series show EKF (blue) tracking closely with wheel odometry (orange), since IMU cannot correct translational drift in straight-line motion.

- **Wheel odometry has the highest error** in all sequences (2.7-3.6m mean). The error time series show a characteristic upward drift trend as integration errors accumulate over time. The error distribution histograms are broad and right-skewed, with some errors exceeding 5-6m by the end of longer trajectories. Without any external reference, wheel slip and encoder noise cause unbounded drift.

- **SLAM as ground truth:** While treated as reference, SLAM is not perfect. SLAM can drift in featureless areas (Seq 00) before loop closure occurs, but graph optimization ensures global consistency. The trajectory comparison plots show SLAM (red) produces the smoothest and most geometrically consistent path. It's the best available reference, not absolute ground truth.

---

## Drift Analysis

Drift is the accumulated position error over time. The following shows how error grows throughout each trajectory:

### Sequence 00 (Empty Hallway)

<center>
<img src="results/analysis/trajectory_comparison_seq00.png" alt="Trajectory Comparison - Sequence 00" width="70%"/>
</center>

**Visual Analysis:**
The trajectory plot shows all four methods' paths in the x-y plane. SLAM (red) provides the reference trajectory. Key observations:

- **Wheel odometry (orange)** shows significant lateral drift, veering noticeably from the SLAM reference. The trajectory appears to curve slightly due to accumulated heading error (37.9° end-to-end, see heading drift table below). Even in a relatively straight hallway, encoder noise and wheel slip cause the estimate to diverge.

- **EKF (blue)** reduces some heading drift (34.2° vs 37.9°) via IMU fusion, resulting in a slightly straighter path than wheel odometry. However, the trajectories largely overlap, as the IMU cannot correct translational drift. The modest improvement (2.93m vs 3.56m mean error) reflects heading correction benefits in a straight corridor.

- **ICP (green)** stays much closer to the SLAM reference throughout. Despite the featureless walls, the hallway geometry provides sufficient structure for scan matching. ICP's trajectory shows small deviations but no systematic drift, maintaining mean error around 0.99m and heading drift of only 5.7°.

- **SLAM (red)** produces the smoothest trajectory. In this featureless environment, loop closure detection is more challenging due to lack of distinctive landmarks - all parts of the hallway look similar, making it harder for SLAM to confidently recognize when it revisits an area. However, SLAM's graph optimization and scan matching still outperform ICP by maintaining global consistency.

### Sequence 01 (Sharp Turns)

<center>
<img src="results/analysis/trajectory_comparison_seq01.png" alt="Trajectory Comparison - Sequence 01" width="70%"/>
</center>

**Visual Analysis:**
This sequence contains the sharpest turns and most obstacles, providing the most challenging test for odometry methods:

- **Wheel odometry (orange)** shows severe distortion at turn points. The trajectory shape is noticeably warped compared to SLAM, with corners appearing rounded or cut short due to accumulated heading error during rapid rotations (36.8° total drift). Fast rotations amplify encoder integration errors, causing the path to diverge significantly from the true trajectory.

- **EKF (blue)** demonstrates its greatest advantage here, with the trajectory closely matching SLAM's shape and turn geometry. The IMU gyroscope provides accurate angular velocity during rapid rotations, correcting heading errors that plague wheel odometry. Remarkably, EKF achieves only 0.4° heading drift (vs 36.8° for wheel odometry) and 0.64m mean error. The trajectory plot shows EKF following SLAM through all turns with minimal deviation.

- **ICP (green)** also performs excellently (0.99m mean error, 2.2° heading drift). The obstacles provide rich geometric features for scan matching, allowing ICP to correct both position and heading at each keyframe. The trajectory shows ICP tracking SLAM through complex maneuvers with high fidelity.

- **SLAM (red)** benefits from loop closure opportunities as the robot revisits areas. Graph optimization produces a geometrically consistent map with sharp, well-defined corners. The abundant features make this an ideal environment for SLAM, achieving only 2.4° heading drift.

### Sequence 02 (Smooth Motion)

<center>
<img src="results/analysis/trajectory_comparison_seq02.png" alt="Trajectory Comparison - Sequence 02" width="70%"/>
</center>

**Visual Analysis:**
This sequence features the longest trajectory (~60-62m) with smooth, gradual motion. It provides the ultimate test of long-term drift accumulation:

- **Wheel odometry (orange)** exhibits severe accumulated drift over the extended trajectory. The path diverges substantially from SLAM, with the endpoint showing significant positional offset. Heading drift reaches 47.8° (the highest across all sequences), causing the trajectory to curve away from the true path. The gradual accumulation of small errors over many timesteps results in large final position error (~3-4m visible in the plot).

- **EKF (blue)** also accumulates significant heading error (37.6°) over the long trajectory, only modestly better than wheel odometry. Without external position reference, even IMU-corrected odometry cannot prevent translational drift over extended distances. The trajectory plot shows EKF deviating from SLAM in parallel with wheel odometry, confirming that IMU provides limited benefit for straight-line drift correction (2.62m vs 2.78m mean error).

- **ICP (green)** maintains better accuracy (1.17m mean error) but shows more drift than in shorter sequences (Seq 00: 0.99m, Seq 01: 0.99m). Heading drift increases to 25.9° - much higher than Seq 00 (5.7°) or Seq 01 (2.2°). The trajectory shows ICP following SLAM more closely than wheel/EKF but with gradual divergence. This demonstrates that scan matching without global optimization (loop closure) still accumulates drift over long distances, especially in areas with less distinctive geometry.

- **SLAM (red)** maintains remarkably low heading drift (6.4°) despite the long trajectory, thanks to loop closure detecting and correcting accumulated errors through graph optimization. The trajectory appears smooth and globally consistent. SLAM reports the longest path distance (62.1m vs ~59-60m for other methods), likely reflecting more accurate distance estimation - methods with heading drift may over- or under-estimate distance depending on how errors compound.

### Heading Drift (Start vs End Orientation)

| Method | Seq 00 | Seq 01 | Seq 02 |
|--------|--------|--------|--------|
| Wheel  | 37.9 deg  | 36.8 deg  | 47.8 deg  |
| EKF    | 34.2 deg  | 0.4 deg   | 37.6 deg  |
| ICP    | 5.7 deg   | 2.2 deg   | 25.9 deg  |
| SLAM   | 7.4 deg   | 2.4 deg   | 6.4 deg   |

**Analysis:**

Heading drift (difference between start and end orientation) is a critical indicator of rotational error accumulation. This metric complements the trajectory plots above, which show the visual impact of heading errors on path shape:

- **Wheel odometry** consistently accumulates the largest heading drift (36-48°) across all sequences. This rotational error causes the curved trajectories visible in all three trajectory plots, where the wheel odometry path (orange) systematically diverges from the SLAM reference. Sequence 02 shows the worst drift (47.8°) due to the longer trajectory duration.

- **EKF** shows dramatic performance variation:
  - **Seq 01:** Exceptional performance with only 0.4° drift. The trajectory plot confirms this - EKF (blue) follows SLAM through all sharp turns with minimal distortion. The IMU gyroscope excels at correcting heading during rapid rotations.
  - **Seq 00 & 02:** Modest improvement (34.2° and 37.6° vs 37.9° and 47.8° for wheel). The trajectory plots show EKF and wheel odometry paths remaining close together, confirming IMU provides limited benefit for straight-line motion where translational drift dominates.

- **ICP** maintains low heading drift in shorter sequences (5.7° and 2.2°) but degrades significantly in Seq 02 (25.9°). The Seq 02 trajectory plot visually confirms this - ICP (green) shows more deviation from SLAM than in the other sequences. Without global loop closure, scan matching accuracy degrades over long trajectories, especially in areas with less distinctive geometry.

- **SLAM** achieves consistently low heading drift (2-7°) across all sequences. Loop closure detects when the robot revisits areas and adjusts the entire trajectory graph to maintain global consistency. The trajectory plots show SLAM (red) producing the smoothest, most geometrically accurate paths in all environments.

---

## Robustness

| Method | Strengths | Weaknesses |
|--------|-----------|------------|
| Wheel Odometry | Smooth output, no external sensor dependency, always available | Accumulates drift continuously, sensitive to wheel slip, no heading correction |
| EKF (Wheel+IMU) | Reduced heading drift via IMU fusion, online gyro bias estimation, works in any environment | Still open-loop (no external position correction), translational drift unchanged |
| ICP (EKF+LiDAR) | LiDAR-based position and heading correction, reduces both translational and rotational drift | Can struggle in featureless environments, sensitive to scan quality, no global correction |
| SLAM (slam_toolbox) | Loop closure corrects accumulated drift, globally consistent map and trajectory | Requires sufficient features for loop closure detection, higher computational cost |

### Map Quality Comparison

The occupancy grid maps generated by ICP and SLAM provide visual insight into mapping accuracy and consistency. Below is Sequence 01 (sharp turns with obstacles), which provides the richest geometric features:

<table>
<tr>
<td width="50%">
<center>

**ICP Map - Sequence 01:**

<img src="results/map_icp_seq00.png" alt="ICP Map - Sequence 00" height=300/>

</center>

</td>
<td width="50%">
<center>

**SLAM Map - Sequence 01:**

<img src="results/map_slam_seq00.png" alt="SLAM Map - Sequence 00" height=300/>

</center>
</td>
</tr>
</table>

**Visual Comparison:**

The left side of both maps shows an interesting phenomenon where the LiDAR point cloud becomes highly distributed and scattered in that region, and the two methods handle it very differently:

- **SLAM map:** The left wall appears faded or missing because SLAM's global optimization detects inconsistencies. When the robot observes this area from different positions, the highly distributed point cloud produces inconsistent measurements that don't agree geometrically across multiple viewpoints. During loop closure and graph optimization, SLAM recognizes these measurements are unreliable and filters them out or reduces their weight to maintain global consistency. Result: the wall appears faded/missing, but the rest of the map remains geometrically correct with sharp corners and parallel walls.

- **ICP map:** The left wall appears as a strong, visible line BUT it's misaligned/misplaced from where it should be. The scattered and distributed point cloud creates false distance measurements that make ICP think there's a wall at an incorrect position. ICP incrementally adds all measurements without global consistency checks - it blindly accepts the unreliable measurements from the distributed point cloud. These returns are strong enough to create a clear line in the occupancy grid, but this line is geometrically incorrect (wrong position/alignment). This is why you see a "strong line but not aligned" - the wall exists in the map, but it's in the wrong place due to the unreliable range measurements from the scattered point cloud.

**Key Insight - Trade-off between methods:**
- **SLAM approach:** Better global consistency by rejecting/suppressing noisy data → wall appears faded but map geometry is more accurate overall
- **ICP approach:** Blindly preserves all measurements (good or bad) → wall appears visible but misplaced/misaligned, creating geometric errors

The same incremental approach causes ICP to show slight blurring or double-imaging in other areas where the robot revisited from different angles - accumulated drift means features don't perfectly align when revisited.

**Fundamental limitation:** Both methods struggle with areas where LiDAR produces highly distributed/scattered point clouds (possibly from reflective surfaces like mirrors or glass, or other environmental factors). SLAM handles it by filtering (making it invisible), while ICP handles it by mapping it incorrectly (making it visible but wrong). This is an inherent challenge of laser-based mapping systems when encountering measurement inconsistencies.

**Other Sequences:**
- [ICP Map - Sequence 01](results/map_icp_seq01.png) | [SLAM Map - Sequence 01](results/map_slam_seq01.png)
- [ICP Map - Sequence 02](results/map_icp_seq02.png) | [SLAM Map - Sequence 02](results/map_slam_seq02.png)

### Robustness Across Environments

- **Empty hallway (Seq 00):** Despite being featureless, ICP achieves good performance because the hallway walls provide consistent parallel geometry. Straight-line motion along parallel walls gives ICP stable geometric constraints for scan matching. SLAM has fewer opportunities for loop closure due to lack of distinctive landmarks, but still performs well through continuous scan matching.

- **Sharp turns with obstacles (Seq 01):** This is the only sequence where EKF outperforms ICP (0.64m vs 0.99m position error, 0.4° vs 2.2° heading drift). EKF excels here because the IMU gyroscope provides extremely accurate angular velocity measurements during rapid rotations, making heading correction the dominant factor. The sharp turns create situations where IMU-based heading correction is more effective than LiDAR scan matching at reducing overall position error. ICP still performs well due to rich geometric features from obstacles and corners, but cannot match EKF's exceptional heading accuracy during fast rotations. SLAM benefits from abundant features for loop closure detection.

- **Smooth motion with obstacles (Seq 02):** ICP shows the worst performance despite having obstacles. With fewer sharp turns and more straight-line segments, there are fewer distinctive geometric features (corners, angles) for ICP to match against. Straight motion provides weaker geometric constraints than sharp turns - ICP has less certainty about position corrections without distinctive landmarks. This demonstrates that geometric distinctiveness and motion profile affect ICP accuracy. Only SLAM with loop closure maintains low heading drift by globally optimizing the entire trajectory.

---

## Summary

**Progressive Improvement Pipeline:**

| Method | Mean Error | Heading Drift Range | Key Strength | Key Limitation |
|--------|-----------|---------------------|--------------|----------------|
| **Wheel Odometry** | 2.7-3.6m | 36-48° | Smooth, always available | Unbounded drift accumulation |
| **EKF (Wheel+IMU)** | 0.64-2.93m | 0.4-37.6° | Excellent heading in sharp turns (Seq 01) | Cannot correct translational drift |
| **ICP (EKF+LiDAR)** | 0.99-1.17m | 2.2-25.9° | Consistent ~1m accuracy across all sequences | Drift accumulates without loop closure |
| **SLAM** | Reference | 2.4-7.4° | Globally consistent via loop closure | Higher computational cost |

**Critical Findings:**

1. **Motion profile matters most for EKF:** Seq 01 (sharp turns) is the only case where EKF beats ICP (0.64m vs 0.99m) due to IMU's exceptional heading correction during rapid rotations. In straight-line motion (Seq 00, 02), EKF provides minimal benefit over wheel odometry.

2. **ICP provides most consistent performance:** Around 1.0m error across all sequences regardless of environment. However, without loop closure, even ICP accumulates significant heading drift (25.9°) in smooth-motion sequences with weaker geometric constraints.

3. **Geometric distinctiveness affects ICP accuracy:** Sharp corners (Seq 01) provide strong constraints → best performance (2.2° heading drift). Smooth curves (Seq 02) provide weak constraints → worst performance (25.9° heading drift). Feature-richness matters less than geometric distinctiveness.

4. **SLAM maintains global consistency:** Loop closure keeps heading drift low (2-7°) across all sequences. Maps show sharper geometry than ICP, though SLAM filters out noisy data (e.g., distributed point clouds from reflective surfaces).
