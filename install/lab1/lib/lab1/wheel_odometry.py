#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations
import os

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')

        # Declare parameters
        self.declare_parameter('sequence', '00')
        self.declare_parameter('output_file', 'trajectory_wheel.txt')

        # Get parameters
        self.sequence = self.get_parameter('sequence').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value

        # Robot parameters
        self.wheel_radius = 0.033  # meters
        self.wheel_base = 0.160    # meters

        # State: [x, y, theta]
        self.state = np.array([0.0, 0.0, 0.0])

        # Previous wheel positions
        self.prev_left_pos = None
        self.prev_right_pos = None

        # Timing
        self.last_update_time = None
        self.start_time = None
        self.end_time = None
        self.first_msg_time = None
        self.last_msg_time = None

        # Trajectory storage
        self.trajectory = []

        # Path for visualization
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        # Counter
        self._update_count = 0

        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom_wheel', 10)
        self.path_pub = self.create_publisher(Path, '/path_wheel', 10)

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'WHEEL ODOMETRY (Baseline) - Sequence {self.sequence}')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'Output: {self.output_file}')
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'Wheel base: {self.wheel_base}m')
        self.get_logger().info(f'{"="*60}')

    def joint_callback(self, msg):
        """Process joint states and compute wheel odometry"""

        current_time = self.get_clock().now()

        # Record start time
        if self.start_time is None:
            self.start_time = current_time
            self.get_logger().info('✓ Started processing data...')

        # Initialize on first message
        if self.last_update_time is None:
            self.last_update_time = current_time

            try:
                left_idx = msg.name.index('wheel_left_joint')
                right_idx = msg.name.index('wheel_right_joint')

                self.prev_left_pos = msg.position[left_idx]
                self.prev_right_pos = msg.position[right_idx]

                self.get_logger().info('✓ Initialized wheel positions')
            except (ValueError, IndexError) as e:
                self.get_logger().warn(f'Could not initialize: {e}')

            return

        # Calculate dt
        dt = (current_time - self.last_update_time).nanoseconds / 1e9

        if dt <= 0:
            return

        # Extract wheel positions
        try:
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')

            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]

        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Could not find wheel joints: {e}',
                                  throttle_duration_sec=5.0)
            return

        # Compute odometry if we have previous positions
        if self.prev_left_pos is not None and self.prev_right_pos is not None:
            # Wheel displacement
            delta_left = left_pos - self.prev_left_pos
            delta_right = right_pos - self.prev_right_pos

            # Linear velocities
            v_left = (delta_left * self.wheel_radius) / dt
            v_right = (delta_right * self.wheel_radius) / dt

            # Differential drive kinematics
            v = (v_right + v_left) / 2.0
            omega = (v_right - v_left) / self.wheel_base

            # Update pose
            x, y, theta = self.state

            x_new = x + v * np.cos(theta) * dt
            y_new = y + v * np.sin(theta) * dt
            theta_new = theta + omega * dt

            # Normalize theta
            theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

            self.state = np.array([x_new, y_new, theta_new])

            # Save trajectory
            timestamp = current_time.nanoseconds / 1e9
            self.trajectory.append([
                timestamp,
                self.state[0],
                self.state[1],
                self.state[2]
            ])

            # Track message timestamps
            if self.first_msg_time is None:
                self.first_msg_time = timestamp
            self.last_msg_time = timestamp

            # Publish odometry and path
            self.publish_odometry(current_time)
            self.publish_path(current_time)

            # Debug
            self._update_count += 1
            if self._update_count % 200 == 0:
                elapsed = (current_time - self.start_time).nanoseconds / 1e9
                self.get_logger().info(
                    f'[{elapsed:.1f}s] Updates: {self._update_count} | '
                    f'Points: {len(self.trajectory)} | '
                    f'Pos: ({self.state[0]:.2f}, {self.state[1]:.2f}) | '
                    f'θ: {np.degrees(self.state[2]):.1f}°'
                )

        # Update for next iteration
        self.prev_left_pos = left_pos
        self.prev_right_pos = right_pos
        self.last_update_time = current_time

    def publish_odometry(self, timestamp):
        """Publish odometry message and TF"""

        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint_wheel'

        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.state[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom)

        # TF
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint_wheel'

        t.transform.translation.x = self.state[0]
        t.transform.translation.y = self.state[1]
        t.transform.translation.z = 0.0

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def publish_path(self, timestamp):
        """Publish path for visualization"""
        # Create pose for current position
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

        # Add to path (keep last 5000 points to avoid memory issues)
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)

        # Update header and publish
        self.path_msg.header.stamp = timestamp.to_msg()
        self.path_pub.publish(self.path_msg)

    def save_trajectory(self):
        """Save trajectory to file and display statistics"""
        if len(self.trajectory) > 0:
            # Record end time
            if self.end_time is None:
                self.end_time = self.get_clock().now()

            # Ensure output directory exists
            output_dir = os.path.dirname(self.output_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)

            # Save trajectory
            np.savetxt(
                self.output_file,
                np.array(self.trajectory),
                fmt='%.6f',
                header='timestamp x y theta',
                comments=''
            )

            # Calculate statistics
            traj_array = np.array(self.trajectory)
            total_distance = 0.0
            for i in range(1, len(traj_array)):
                dx = traj_array[i, 1] - traj_array[i-1, 1]
                dy = traj_array[i, 2] - traj_array[i-1, 2]
                total_distance += np.sqrt(dx**2 + dy**2)

            processing_time = (self.end_time - self.start_time).nanoseconds / 1e9 if self.start_time else 0
            data_duration = self.last_msg_time - self.first_msg_time if self.first_msg_time and self.last_msg_time else 0

            # Display summary
            self.get_logger().info(f'')
            self.get_logger().info(f'{"="*60}')
            self.get_logger().info(f'WHEEL ODOMETRY SUMMARY - Sequence {self.sequence}')
            self.get_logger().info(f'{"="*60}')
            self.get_logger().info(f'Processing time:    {processing_time:.2f}s')
            self.get_logger().info(f'Data duration:      {data_duration:.2f}s')
            self.get_logger().info(f'Processing rate:    {data_duration/processing_time:.2f}x real-time' if processing_time > 0 else 'N/A')
            self.get_logger().info(f'Total updates:      {self._update_count}')
            self.get_logger().info(f'Trajectory points:  {len(self.trajectory)}')
            self.get_logger().info(f'Total distance:     {total_distance:.2f}m')
            self.get_logger().info(f'Final position:     ({self.state[0]:.3f}, {self.state[1]:.3f})m')
            self.get_logger().info(f'Final heading:      {np.degrees(self.state[2]):.2f}°')
            self.get_logger().info(f'Output file:        {self.output_file}')
            self.get_logger().info(f'{"="*60}')
        else:
            self.get_logger().warn('⚠ No trajectory data to save!')

    def __del__(self):
        """Destructor - save trajectory when node is destroyed"""
        try:
            self.save_trajectory()
        except:
            pass  # Ignore errors during shutdown


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometry()

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
