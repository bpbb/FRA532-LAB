#!/usr/bin/env python3
"""
EKF Odometry Node - Part 1: EKF Sensor Fusion

Fuses wheel odometry (from /joint_states) with IMU measurements (from /imu)
using an Extended Kalman Filter (EKF) to produce a filtered odometry estimate.

EKF State: [x, y, theta, b_g]
  - x, y, theta: robot pose in the odom frame
  - b_g: gyroscope bias (rad/s) - estimated online

Algorithm:
  - Prediction: IMU angular velocity (with bias correction) for heading,
                wheel encoders for linear velocity
  - Update:     Wheel angular velocity as heading rate measurement,
                IMU orientation as heading measurement (if available)

Key features:
  - Velocity-dependent process noise (Thrun's alpha model)
  - Online gyro bias estimation via state augmentation
  - Mahalanobis distance gating for outlier rejection
  - Joseph form covariance update for numerical stability
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations
import os


class EKFOdometry(Node):
    def __init__(self):
        super().__init__('ekf_odometry')

        # Declare parameters
        self.declare_parameter('sequence', '00')
        self.declare_parameter('output_file', 'trajectory_ekf.txt')

        self.sequence = self.get_parameter('sequence').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # Robot parameters
        self.wheel_radius = 0.033  # meters
        self.wheel_base = 0.160    # meters

        # State vector: [x, y, theta, b_g]
        #   b_g = gyroscope bias in rad/s
        self.state = np.array([0.0, 0.0, 0.0, 0.0])

        # State covariance matrix (4x4)
        self.P = np.diag([0.01, 0.01, 0.01, 0.0001])

        # Velocity-dependent motion noise parameters (Thrun's model)
        # sigma_v^2 = alpha1*v^2 + alpha2*omega^2
        # sigma_w^2 = alpha3*v^2 + alpha4*omega^2
        self.alpha1 = 0.05   # velocity  -> velocity noise
        self.alpha2 = 0.01   # rotation  -> velocity noise
        self.alpha3 = 0.01   # velocity  -> rotation noise
        self.alpha4 = 0.05   # rotation  -> rotation noise

        # Bias random walk noise (how fast the gyro bias drifts)
        self.Q_bias = 1e-8

        # Measurement noise: wheel angular velocity (rad/s)^2
        self.R_wheel = np.diag([0.01])

        # Measurement noise: IMU orientation heading (rad)^2
        self.R_heading = np.diag([0.05])

        # Mahalanobis distance threshold for gating
        # chi-squared with 1 DOF: 3.84 (95%), 6.63 (99%), 7.88 (99.5%)
        self.mahal_threshold = 7.88
        # ==========================================================

        # Sensor data
        self.latest_joint_msg = None
        self.latest_imu_angular_z = 0.0

        # IMU orientation tracking
        self.imu_yaw = 0.0
        self.imu_initial_yaw = None
        self.has_imu_orientation = False

        # Previous wheel positions for delta computation
        self.prev_left_pos = None
        self.prev_right_pos = None

        # Timing
        self.last_update_time = None

        # Trajectory storage
        self.trajectory = []

        # Path for visualization
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, 10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom_ekf', 10)
        self.path_pub = self.create_publisher(Path, '/path_ekf', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # EKF update timer at 20 Hz (matches sensor rate)
        self.update_rate = 20.0
        self.ekf_timer = self.create_timer(1.0 / self.update_rate, self.ekf_update_loop)

        # Statistics
        self._update_count = 0
        self._rejected_count = 0
        self.start_time = None
        self.end_time = None
        self.first_msg_time = None
        self.last_msg_time = None

        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'EKF ODOMETRY - Sequence {self.sequence}')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'Output: {self.output_file}')
        self.get_logger().info(f'State: [x, y, theta, b_g]')
        self.get_logger().info(f'Alpha: [{self.alpha1}, {self.alpha2}, {self.alpha3}, {self.alpha4}]')
        self.get_logger().info(f'R_wheel: {np.diag(self.R_wheel)}, R_heading: {np.diag(self.R_heading)}')
        self.get_logger().info(f'Mahalanobis threshold: {self.mahal_threshold}')
        self.get_logger().info(f'{"="*60}')

    # ==================== Callbacks ====================

    def joint_callback(self, msg):
        """Store latest joint states message."""
        self.latest_joint_msg = msg

    def imu_callback(self, msg):
        """Process IMU data - extract angular velocity and orientation."""
        self.latest_imu_angular_z = msg.angular_velocity.z

        # Try to extract yaw from IMU orientation quaternion
        # ROS convention: orientation_covariance[0] == -1 means no orientation data
        if msg.orientation_covariance[0] < 0:
            return

        q = msg.orientation
        norm_sq = q.x**2 + q.y**2 + q.z**2 + q.w**2

        if norm_sq > 0.5:  # Valid unit quaternion
            _, _, yaw = tf_transformations.euler_from_quaternion(
                [q.x, q.y, q.z, q.w])

            if self.imu_initial_yaw is None:
                self.imu_initial_yaw = yaw
                self.get_logger().info(
                    f'IMU orientation available (initial yaw: {np.degrees(yaw):.1f} deg)')

            if self.imu_initial_yaw is not None:
                self.imu_yaw = yaw - self.imu_initial_yaw
                self.imu_yaw = np.arctan2(np.sin(self.imu_yaw), np.cos(self.imu_yaw))
                self.has_imu_orientation = True

    # ==================== Main Loop ====================

    def ekf_update_loop(self):
        """Main EKF loop - runs at fixed rate (20 Hz)."""
        if self.latest_joint_msg is None:
            return

        current_time = self.get_clock().now()

        if self.start_time is None:
            self.start_time = current_time
            self.get_logger().info('Started processing data...')

        # Initialize on first update
        if self.last_update_time is None:
            self.last_update_time = current_time
            try:
                left_idx = self.latest_joint_msg.name.index('wheel_left_joint')
                right_idx = self.latest_joint_msg.name.index('wheel_right_joint')
                self.prev_left_pos = self.latest_joint_msg.position[left_idx]
                self.prev_right_pos = self.latest_joint_msg.position[right_idx]
                self.get_logger().info('Initialized wheel positions')
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Init error: {e}')
            return

        dt = (current_time - self.last_update_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Extract current wheel positions
        try:
            left_idx = self.latest_joint_msg.name.index('wheel_left_joint')
            right_idx = self.latest_joint_msg.name.index('wheel_right_joint')
            left_pos = self.latest_joint_msg.position[left_idx]
            right_pos = self.latest_joint_msg.position[right_idx]
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Joint error: {e}', throttle_duration_sec=5.0)
            return

        if self.prev_left_pos is not None and self.prev_right_pos is not None:
            # Compute wheel displacements and velocities
            delta_left = left_pos - self.prev_left_pos
            delta_right = right_pos - self.prev_right_pos
            v_left = (delta_left * self.wheel_radius) / dt
            v_right = (delta_right * self.wheel_radius) / dt

            # Differential drive kinematics
            v = (v_right + v_left) / 2.0
            omega_wheel = (v_right - v_left) / self.wheel_base
            omega_imu = self.latest_imu_angular_z

            # === EKF Prediction (IMU heading + wheel velocity) ===
            self.ekf_predict(v, omega_imu, dt)

            # === EKF Update 1: Wheel angular velocity measurement ===
            self.ekf_update_wheel(omega_wheel, omega_imu, dt)

            # === EKF Update 2: IMU orientation measurement (if available) ===
            if self.has_imu_orientation:
                self.ekf_update_orientation()

            # Record trajectory point
            timestamp = current_time.nanoseconds / 1e9
            self.trajectory.append([
                timestamp, self.state[0], self.state[1], self.state[2]
            ])

            if self.first_msg_time is None:
                self.first_msg_time = timestamp
            self.last_msg_time = timestamp

            # Publish odometry and path
            self.publish_odometry(current_time)
            self.publish_path(current_time)

            # Periodic debug output
            self._update_count += 1
            if self._update_count % 200 == 0:
                elapsed = (current_time - self.start_time).nanoseconds / 1e9
                mode = "ori+wheel" if self.has_imu_orientation else "wheel"
                self.get_logger().info(
                    f'[{elapsed:.1f}s] #{self._update_count} | '
                    f'Pos: ({self.state[0]:.2f}, {self.state[1]:.2f}) | '
                    f'theta: {np.degrees(self.state[2]):.1f} deg | '
                    f'bias: {np.degrees(self.state[3]):.3f} deg/s | '
                    f'sigma: ({np.sqrt(self.P[0,0]):.3f}, {np.sqrt(self.P[1,1]):.3f}, '
                    f'{np.degrees(np.sqrt(self.P[2,2])):.2f} deg) | '
                    f'meas: {mode} | rej: {self._rejected_count}'
                )

        # Update previous values for next iteration
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.last_update_time = current_time

    # ==================== EKF Algorithm ====================

    def ekf_predict(self, v, omega_imu, dt):
        """
        EKF Prediction Step

        Uses de-biased IMU angular velocity for heading prediction
        and wheel-derived linear velocity for position prediction.
        Process noise is velocity-dependent (Thrun's alpha model).

        State transition:
          x'     = x + v * cos(theta) * dt
          y'     = y + v * sin(theta) * dt
          theta' = theta + (omega_imu - b_g) * dt
          b_g'   = b_g                              (random walk)

        Jacobian F creates cross-correlation between theta and b_g
        (F[2,3] = -dt), enabling bias observability through wheel updates.
        """
        x, y, theta, b_g = self.state

        # De-biased IMU angular velocity
        omega_corrected = omega_imu - b_g

        # State prediction
        x_new = x + v * np.cos(theta) * dt
        y_new = y + v * np.sin(theta) * dt
        theta_new = theta + omega_corrected * dt
        theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))
        b_g_new = b_g  # bias persists (random walk via Q)

        self.state = np.array([x_new, y_new, theta_new, b_g_new])

        # State transition Jacobian
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt, 0],
            [0, 1,  v * np.cos(theta) * dt, 0],
            [0, 0, 1, -dt],
            [0, 0, 0, 1]
        ])

        # Velocity-dependent process noise
        Q = self._compute_process_noise(v, omega_corrected, theta, dt)

        # Covariance prediction
        self.P = F @ self.P @ F.T + Q

    def _compute_process_noise(self, v, omega, theta, dt):
        """
        Compute velocity-dependent process noise (Thrun's alpha model).

        Noise scales with motion magnitude:
          sigma_v^2 = alpha1*v^2 + alpha2*omega^2
          sigma_w^2 = alpha3*v^2 + alpha4*omega^2

        Then projected into state space via the control Jacobian V.
        """
        # Motion-dependent noise variances with minimum floor
        sigma_v_sq = self.alpha1 * v**2 + self.alpha2 * omega**2 + 1e-6
        sigma_w_sq = self.alpha3 * v**2 + self.alpha4 * omega**2 + 1e-6

        # Control noise covariance
        M = np.diag([sigma_v_sq, sigma_w_sq])

        # Jacobian of motion model w.r.t. control noise [4x2]
        V = np.array([
            [np.cos(theta) * dt, 0],
            [np.sin(theta) * dt, 0],
            [0, dt],
            [0, 0]
        ])

        # Project control noise into state space
        Q = V @ M @ V.T

        # Add bias random walk noise
        Q[3, 3] = self.Q_bias

        return Q

    def ekf_update_wheel(self, omega_wheel, omega_imu, dt):
        """
        EKF Update Step 1 - Wheel Angular Velocity Measurement

        Compares wheel-derived angular velocity with de-biased IMU angular
        velocity. This corrects both heading and gyro bias.

        Measurement model:
          z    = omega_wheel               (measured by wheel encoders)
          h(x) = omega_imu - x[3]          (de-biased IMU = expected true omega)
          H    = [0, 0, 0, -1]             (only depends on bias state)

        The theta-bias cross-correlation from prediction (F[2,3] = -dt)
        propagates the bias correction into theta via the Kalman gain.
        """
        # Measurement and expected measurement
        z = np.array([omega_wheel])
        h_x = np.array([omega_imu - self.state[3]])

        # Innovation
        innovation = z - h_x  # omega_wheel - (omega_imu - b_g)

        # Observation matrix: h(x) = omega_imu - x[3]  =>  dh/dx = [0,0,0,-1]
        H = np.array([[0.0, 0.0, 0.0, -1.0]])

        # Innovation covariance
        S = H @ self.P @ H.T + self.R_wheel

        # Mahalanobis distance gating
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
        self.P = (self.P + self.P.T) / 2.0

    def ekf_update_orientation(self):
        """
        EKF Update Step 2 - IMU Orientation Measurement (optional)

        Uses IMU orientation as a direct heading measurement.
        Applied with higher R (less trust) since IMU yaw drifts over time.

        Measurement model:
          z    = imu_yaw                   (from IMU orientation filter)
          h(x) = x[2]                      (predicted theta)
          H    = [0, 0, 1, 0]
        """
        # Innovation
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

    # ==================== Publishing ====================

    def publish_odometry(self, timestamp):
        """Publish odometry message and TF transform."""
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint_ekf'

        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Fill covariance from EKF (6x6 for x,y,z,roll,pitch,yaw)
        odom.pose.covariance[0] = self.P[0, 0]   # x variance
        odom.pose.covariance[7] = self.P[1, 1]   # y variance
        odom.pose.covariance[35] = self.P[2, 2]  # yaw variance

        self.odom_pub.publish(odom)

        # Broadcast TF: odom -> base_footprint
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint_ekf'
        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_path(self, timestamp):
        """Publish path for RViz visualization."""
        pose = PoseStamped()
        pose.header.stamp = timestamp.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.state[0]
        pose.pose.position.y = self.state[1]
        pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)

        self.path_msg.header.stamp = timestamp.to_msg()
        self.path_pub.publish(self.path_msg)

    # ==================== Data Saving ====================

    def save_trajectory(self):
        """Save trajectory to file and display summary statistics."""
        if len(self.trajectory) == 0:
            self.get_logger().warn('No trajectory data to save!')
            return

        if self.end_time is None:
            self.end_time = self.get_clock().now()

        # Ensure output directory exists
        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        np.savetxt(
            self.output_file,
            np.array(self.trajectory),
            fmt='%.6f',
            header='timestamp x y theta',
            comments=''
        )

        # Compute statistics
        traj = np.array(self.trajectory)
        total_dist = np.sum(np.sqrt(np.diff(traj[:, 1])**2 + np.diff(traj[:, 2])**2))
        proc_time = (self.end_time - self.start_time).nanoseconds / 1e9 if self.start_time else 0
        data_dur = self.last_msg_time - self.first_msg_time if self.first_msg_time and self.last_msg_time else 0

        self.get_logger().info(f'')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'EKF ODOMETRY SUMMARY - Sequence {self.sequence}')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'Processing time:    {proc_time:.2f}s')
        self.get_logger().info(f'Data duration:      {data_dur:.2f}s')
        if proc_time > 0:
            self.get_logger().info(f'Processing rate:    {data_dur/proc_time:.2f}x real-time')
        self.get_logger().info(f'Total updates:      {self._update_count}')
        self.get_logger().info(f'Rejected updates:   {self._rejected_count}')
        self.get_logger().info(f'Trajectory points:  {len(self.trajectory)}')
        self.get_logger().info(f'Total distance:     {total_dist:.2f}m')
        self.get_logger().info(f'Final position:     ({self.state[0]:.3f}, {self.state[1]:.3f})m')
        self.get_logger().info(f'Final heading:      {np.degrees(self.state[2]):.2f} deg')
        self.get_logger().info(f'Estimated bias:     {np.degrees(self.state[3]):.4f} deg/s')
        self.get_logger().info(f'Final uncertainty:  x={np.sqrt(self.P[0,0]):.4f}m, '
                               f'y={np.sqrt(self.P[1,1]):.4f}m, '
                               f'theta={np.degrees(np.sqrt(self.P[2,2])):.2f} deg')
        self.get_logger().info(f'IMU orientation:    {"available" if self.has_imu_orientation else "not available"}')
        self.get_logger().info(f'Output file:        {self.output_file}')
        self.get_logger().info(f'{"="*60}')

    def __del__(self):
        try:
            self.save_trajectory()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = EKFOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_trajectory()
        except Exception as e:
            print(f"Error saving trajectory: {e}")
        try:
            node.destroy_node()
        except:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
