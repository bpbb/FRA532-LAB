# LAB1: Kalman Filter / SLAM

## Learning Outcomes
By completing this lab, students will be able to:

- Implement EKF-based sensor fusion for mobile robots
- Apply ICP for LiDAR-based odometry refinement
- Understand the role of loop closure in SLAM
- Critically evaluate different localization and mapping approaches

## Lab Objective
The objective of this lab is to understand and evaluate a complete 2D mobile robot localization pipeline by progressively integrating sensor fusion, scan matching, and full SLAM. Students will:

- Fuse wheel odometry and IMU measurements using an Extended Kalman Filter (EKF)
- Refine odometry using LiDAR-based ICP scan matching
- Perform full SLAM using `slam_toolbox` (Nav2)
- Quantitatively and qualitatively compare odometry estimation and mapping performance across different methods

Through this lab, students will gain hands-on experience with probabilistic state estimation, scan matching, and graph-based SLAM using real ROS2 data.

---

## Collaboration Policy
**Individual Work:** Part 1 (EKF Odometry Fusion) and Part 2 (ICP Odometry Refinement) must be completed individually.

**Group Work:** Part 3 (Full SLAM with `slam_toolbox`) must be completed ingruops of **up to 3 students**.

Each student is expected to understand and be able to explain all submitted work.

---

## Dataset Description
The dataset is provided as a ROS bag and contains sensor measurements recorded during robot motion.

**Topics included:**

- **`/scan`**: 2D LiDAR laser scans at 5 Hz
- **`/imu`**: Gyroscope and accelerometer data at 20 Hz
- **`/joint_states`**: Wheel motor position and velocity at 20 Hz

The dataset is divided into three sequences, each representing a different environmental condition:

1. **Sequence 00 – Empty Hallway:**
A static indoor hallway environment with minimal obstacles and no dynamic objects. This sequence is intended to evaluate baseline odometry and sensor fusion performance.

2. **Sequence 01 – Non-Empty Hallway with Sharp Turns:**
An indoor hallway environment containing obstacles and clutter, with sections of sharp turning motion. This sequence is designed to challenge odometry and scan matching performance under rapid heading changes.

3. **Sequence 02 – Non-Empty Hallway with Non-Aggressive Motion:**
An indoor hallway environment with obstacles, similar to Sequence 2, but recorded with smoother and non-aggressive robot motion. This sequence is intended to evaluate performance under more stable motion conditions.

---

## Robot Dimension

The figure below illustrates the robot geometry and key dimensions used throughout this lab. These parameters are required for accurate wheel odometry computation and sensor frame alignment.

![Turtlebot3 Burger Dimension](pic/turtlebot3_dimension1.png)

---

## Part 1: EKF Odometry Fusion

### Objective
The objective of this part is to implement an Extended Kalman Filter (EKF) to fuse wheel odometry and IMU measurements in order to obtain a filtered and more reliable odometry estimate compared to raw wheel odometry.

### Description
In this part, you will compute wheel odometry from `/joint_states` and fuse it with IMU measurements from `/imu` using an EKF. The EKF estimates the robot pose by combining a differential-drive motion model with probabilistic sensor updates.

---

## Part 2: ICP Odometry Refinement

### Objective
The objective of this part is to refine the EKF-based odometry using LiDAR scan matching and evaluate the improvement in accuracy and drift.

### Description
In this part, you will use the EKF odometry from Part 1 as the initial guess for ICP scan matching on consecutive `/scan` messages. The estimated relative pose from ICP is integrated to produce a LiDAR-based odometry estimate.

---

## Part 3: Full SLAM with `slam_toolbox`

### Objective
The objective of this part is to perform full SLAM using `slam_toolbox` and compare its pose estimation and mapping performance with the ICP-based odometry from Part 2.

### Description
In this part, you will run `slam_toolbox` using LiDAR data and an odometry source to perform full SLAM with loop closure. The resulting trajectory and map are compared with the ICP-based odometry from Part 2.

---

## Deliverables

Students should submit:

1. Source code
2. README
3. Plots of trajectories from:
   - Wheel odometry
   - EKF odometry
   - ICP odometry
   - SLAM pose output
4. Generated 2D maps from ICP odometry and `slam_toolbox`
5. A discussion comparing accuracy, drift, and robustness of each method

---

## MATLAB Support

**Note for MATLAB users:** Students who choose to use MATLAB for this lab can load and process the ROS bag files using MATLAB’s ROS Toolbox. Please refer to the official MathWorks documentation:
https://www.mathworks.com/help/ros/ug/work-with-rosbag-logfiles.html

---
