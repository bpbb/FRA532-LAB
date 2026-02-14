#!/usr/bin/env python3
"""
Records SLAM trajectory and map from slam_toolbox.

- Publishes base_footprint_ekf -> base_scan TF periodically
- Looks up map -> base_footprint_ekf TF and records SLAM trajectory
- Subscribes to /map and saves occupancy grid as PNG on shutdown
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
from tf2_ros import TransformBroadcaster
import numpy as np
import math
import os


class Slam(Node):
    def __init__(self):
        super().__init__('slam')

        # Parameters
        self.declare_parameter('sequence', '00')
        self.declare_parameter('output_file', '')
        self.declare_parameter('map_file', '')

        self.sequence = self.get_parameter('sequence').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value

        # TF broadcaster for base_footprint_ekf -> base_scan
        # Published periodically (not static) to survive TF buffer clears
        # caused by sim time clock jumps during bag playback
        self.tf_broadcaster = TransformBroadcaster(self)

        # TF listener for SLAM pose lookup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Path publisher
        self.path_pub = self.create_publisher(Path, '/path_slam', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        # Subscribe to /map (Transient Local durability to get latched map)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.latest_map = None

        # Timer to periodically publish TF and look up SLAM pose (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Trajectory data
        self.trajectory = []
        self.initialized = False
        self.start_time = None
        self._update_count = 0

        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'SLAM TRAJECTORY RECORDER - Sequence {self.sequence}')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'Output: {self.output_file}')
        self.get_logger().info(f'Map:    {self.map_file}')
        self.get_logger().info(f'Looking up TF: map -> base_footprint_ekf')
        self.get_logger().info(f'{"="*60}')

    def map_callback(self, msg):
        self.latest_map = msg

    def publish_base_to_scan_tf(self):
        """Publish base_footprint_ekf -> base_scan transform.
        TurtleBot3 Burger: base_footprint->base_link (z=0.010)
                           base_link->base_scan (x=-0.032, z=0.172)
                           Combined: x=-0.032, y=0, z=0.182
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint_ekf'
        t.child_frame_id = 'base_scan'
        t.transform.translation.x = -0.032
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.182
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def timer_callback(self):
        # Always publish base->scan TF so slam_toolbox can find it
        self.publish_base_to_scan_tf()

        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint_ekf', rclpy.time.Time())

            x = t.transform.translation.x
            y = t.transform.translation.y

            # Extract yaw from quaternion
            q = t.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            stamp = t.header.stamp
            time_sec = stamp.sec + stamp.nanosec * 1e-9

            self.trajectory.append([time_sec, x, y, theta])

            # Publish path
            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation = t.transform.rotation

            self.path_msg.poses.append(pose)
            if len(self.path_msg.poses) > 5000:
                self.path_msg.poses.pop(0)

            self.path_msg.header.stamp = stamp
            self.path_pub.publish(self.path_msg)

            if not self.initialized:
                self.start_time = self.get_clock().now()
                self.get_logger().info(
                    f'SLAM trajectory recording started at ({x:.3f}, {y:.3f})')
                self.initialized = True

            self._update_count += 1
            if self._update_count % 100 == 0:
                self.get_logger().info(
                    f'SLAM poses recorded: {len(self.trajectory)} | '
                    f'Pos: ({x:.2f}, {y:.2f}) | theta: {math.degrees(theta):.1f} deg')

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            pass

    def save_map(self):
        """Save the latest SLAM occupancy grid as PNG image."""
        if self.latest_map is None or not self.map_file:
            self.get_logger().warn('No SLAM map data to save!')
            return

        output_dir = os.path.dirname(self.map_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        grid = self.latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution

        # Convert OccupancyGrid to image
        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
        # Image: 254=free, 0=occupied, 205=unknown
        img_data = np.zeros(width * height, dtype=np.uint8)
        for i, val in enumerate(grid.data):
            if val == -1:
                img_data[i] = 205  # unknown
            elif val == 0:
                img_data[i] = 254  # free
            else:
                img_data[i] = max(0, int(253 * (1.0 - val / 100.0)))

        img_data = img_data.reshape((height, width))
        img_data = np.flipud(img_data)

        from PIL import Image
        img = Image.fromarray(img_data, mode='L')
        img.save(self.map_file)

        self.get_logger().info(f'SLAM map saved: {self.map_file} ({width}x{height}, res={resolution}m)')

    def save_trajectory(self):
        if len(self.trajectory) > 0 and self.output_file:
            output_dir = os.path.dirname(self.output_file)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir, exist_ok=True)

            data = np.array(self.trajectory)
            np.savetxt(
                self.output_file, data, fmt='%.6f',
                header='timestamp x y theta', comments='')

            # Statistics
            total_distance = 0.0
            for i in range(1, len(data)):
                dx = data[i, 1] - data[i-1, 1]
                dy = data[i, 2] - data[i-1, 2]
                total_distance += np.sqrt(dx**2 + dy**2)

            self.get_logger().info(f'')
            self.get_logger().info(f'{"="*60}')
            self.get_logger().info(f'SLAM TRAJECTORY SUMMARY - Sequence {self.sequence}')
            self.get_logger().info(f'{"="*60}')
            self.get_logger().info(f'Trajectory points:  {len(self.trajectory)}')
            self.get_logger().info(f'Total distance:     {total_distance:.2f}m')
            self.get_logger().info(f'Final position:     ({data[-1,1]:.3f}, {data[-1,2]:.3f})m')
            self.get_logger().info(f'Final heading:      {np.degrees(data[-1,3]):.2f} deg')
            self.get_logger().info(f'Output file:        {self.output_file}')
            self.get_logger().info(f'{"="*60}')
        else:
            self.get_logger().warn('No SLAM trajectory data to save!')


def main(args=None):
    rclpy.init(args=args)
    node = Slam()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_trajectory()
            node.save_map()
        except Exception as e:
            print(f"Error saving SLAM data: {e}")
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
