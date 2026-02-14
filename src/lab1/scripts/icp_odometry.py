#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
from tf2_ros import TransformBroadcaster
import tf_transformations
import os
from scipy.spatial import KDTree
from collections import deque
import yaml


class ICPOdometry(Node):
    def __init__(self):
        super().__init__('icp_odometry')

        self.declare_parameter('sequence', '00')
        self.declare_parameter('output_file', 'trajectory_icp.txt')
        self.declare_parameter('map_file', 'map_icp.png')
        self.declare_parameter('build_map', True)

        self.sequence = self.get_parameter('sequence').get_parameter_value().string_value
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        self.map_file = self.get_parameter('map_file').get_parameter_value().string_value
        self.build_map = self.get_parameter('build_map').get_parameter_value().bool_value

        # Local map parameters
        self.local_map_size = 15         # number of keyframe scans to keep
        self.local_map_voxel_size = 0.05 # meters - downsample local map grid
        self.keyframe_dist_thresh = 0.15 # meters between keyframes
        self.keyframe_angle_thresh = np.radians(5.0)

        # ICP parameters
        self.icp_max_iterations = 50
        self.icp_tolerance = 1e-6
        self.icp_max_correspondence_distance = 0.5
        self.min_correspondences = 30
        self.max_scan_points = 300

        # Constraints: max ICP deviation from EKF guess
        self.max_translation_correction = 0.15
        self.max_rotation_correction = np.radians(2.0)  # tight on rotation

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Local map: list of (points_in_odom_frame) from recent keyframes
        self.local_map_scans = deque(maxlen=self.local_map_size)
        self.local_map_points = None  # cached merged + downsampled
        self.local_map_tree = None    # cached KDTree
        self.local_map_dirty = True

        # Keyframe tracking
        self.last_kf_x = 0.0
        self.last_kf_y = 0.0
        self.last_kf_theta = 0.0

        # EKF tracking
        self.prev_ekf_x = None
        self.prev_ekf_y = None
        self.prev_ekf_theta = None
        self.latest_ekf_odom = None

        self.start_time = None
        self.end_time = None
        self.first_msg_time = None
        self.last_msg_time = None

        self.trajectory = []
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/odom_ekf', self.ekf_odom_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom_icp', 10)
        self.path_pub = self.create_publisher(Path, '/path_icp', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Map building
        if self.build_map:
            self.map_pub = self.create_publisher(OccupancyGrid, '/map_icp', 10)
            self.map_resolution = 0.05  # meters per cell
            self.map_size = 200  # meters (will create 200m x 200m map)
            self.map_origin_x = -100.0  # map center
            self.map_origin_y = -100.0
            self.occupancy_grid = None
            self.hit_count = None
            self.miss_count = None
            self.initialize_map()
            # Publish map periodically
            self.map_timer = self.create_timer(2.0, self.publish_map)

        self._scan_count = 0
        self._update_count = 0
        self._fallback_count = 0
        self._icp_success_count = 0
        self._keyframe_count = 0

    def ekf_odom_callback(self, msg):
        self.latest_ekf_odom = msg

    def scan_callback(self, msg):
        current_time = self.get_clock().now()

        if self.start_time is None:
            self.start_time = current_time
            self.publish_odom_and_path(current_time)

        self._scan_count += 1

        current_points = self.scan_to_pointcloud(msg)
        if len(current_points) < self.min_correspondences:
            return

        if self.latest_ekf_odom is None:
            return

        ekf_x, ekf_y, ekf_theta = self.extract_pose(self.latest_ekf_odom)

        # Initialize
        if self.prev_ekf_x is None:
            self.prev_ekf_x = ekf_x
            self.prev_ekf_y = ekf_y
            self.prev_ekf_theta = ekf_theta

            # Add first scan to local map
            odom_points = self.transform_to_odom(current_points, self.x, self.y, self.theta)
            self.local_map_scans.append(odom_points)
            self.local_map_dirty = True
            self.last_kf_x = self.x
            self.last_kf_y = self.y
            self.last_kf_theta = self.theta
            self._keyframe_count = 1
            return

        # Step 1: Apply EKF delta (always - this is our verified baseline)
        ekf_dx = ekf_x - self.prev_ekf_x
        ekf_dy = ekf_y - self.prev_ekf_y
        ekf_dtheta = np.arctan2(
            np.sin(ekf_theta - self.prev_ekf_theta),
            np.cos(ekf_theta - self.prev_ekf_theta))

        self.x += ekf_dx
        self.y += ekf_dy
        self.theta += ekf_dtheta
        self.theta = np.arctan2(np.sin(self.theta), np.cos(self.theta))

        # Step 2: Check if keyframe - run ICP against local map
        dist_from_kf = np.sqrt((self.x - self.last_kf_x)**2 +
                               (self.y - self.last_kf_y)**2)
        angle_from_kf = abs(np.arctan2(
            np.sin(self.theta - self.last_kf_theta),
            np.cos(self.theta - self.last_kf_theta)))

        if dist_from_kf > self.keyframe_dist_thresh or \
           angle_from_kf > self.keyframe_angle_thresh:

            if len(self.local_map_scans) >= 3:  # need enough map
                # Build/update local map KDTree
                if self.local_map_dirty:
                    self.rebuild_local_map()

                if self.local_map_tree is not None:
                    # Subsample current scan
                    if len(current_points) > self.max_scan_points:
                        idx = np.random.choice(len(current_points),
                                               self.max_scan_points, replace=False)
                        pts_icp = current_points[idx]
                    else:
                        pts_icp = current_points

                    # ICP: match current scan (in body frame) against local map (in odom frame)
                    # Initial guess: current ICP pose
                    try:
                        refined_x, refined_y, refined_theta, success = \
                            self.icp_scan_to_map(pts_icp, self.x, self.y, self.theta)

                        if success:
                            corr_t = np.sqrt((refined_x - self.x)**2 +
                                             (refined_y - self.y)**2)
                            corr_r = abs(np.arctan2(
                                np.sin(refined_theta - self.theta),
                                np.cos(refined_theta - self.theta)))

                            if corr_t < self.max_translation_correction and \
                               corr_r < self.max_rotation_correction:
                                self.x = refined_x
                                self.y = refined_y
                                self.theta = refined_theta
                                self._icp_success_count += 1
                            else:
                                self._fallback_count += 1
                        else:
                            self._fallback_count += 1
                    except Exception as e:
                        self.get_logger().error(f'ICP err: {e}',
                                                throttle_duration_sec=5.0)
                        self._fallback_count += 1

            # Add current scan to local map (transformed to odom frame)
            odom_points = self.transform_to_odom(
                current_points, self.x, self.y, self.theta)
            self.local_map_scans.append(odom_points)
            self.local_map_dirty = True

            self.last_kf_x = self.x
            self.last_kf_y = self.y
            self.last_kf_theta = self.theta
            self._keyframe_count += 1

        # Update map with current scan
        if self.build_map:
            self.update_map(current_points)

        # Save trajectory
        timestamp = current_time.nanoseconds / 1e9
        self.trajectory.append([timestamp, self.x, self.y, self.theta])
        if self.first_msg_time is None:
            self.first_msg_time = timestamp
        self.last_msg_time = timestamp

        self.publish_odom_and_path(current_time)

        self._update_count += 1
        interval = 10 if self._update_count <= 50 else 50
        if self._update_count % interval == 0:
            elapsed = (current_time - self.start_time).nanoseconds / 1e9
            diff = np.sqrt((self.x - ekf_x)**2 + (self.y - ekf_y)**2)
            self.get_logger().info(
                f'[{elapsed:.1f}s] #{self._update_count} | '
                f'ICP: ({self.x:.3f}, {self.y:.3f}) θ={np.degrees(self.theta):.1f} | '
                f'EKF: ({ekf_x:.3f}, {ekf_y:.3f}) θ={np.degrees(ekf_theta):.1f} | '
                f'diff: {diff:.3f}m | '
                f'ok: {self._icp_success_count} fb: {self._fallback_count} '
                f'kf: {self._keyframe_count} map: {len(self.local_map_scans)}')

        self.prev_ekf_x = ekf_x
        self.prev_ekf_y = ekf_y
        self.prev_ekf_theta = ekf_theta

    # ==================== Map Building ====================

    def initialize_map(self):
        """Initialize occupancy grid map."""
        grid_width = int(self.map_size / self.map_resolution)
        grid_height = int(self.map_size / self.map_resolution)
        self.occupancy_grid = np.ones((grid_height, grid_width), dtype=np.int8) * -1  # Unknown
        self.hit_count = np.zeros((grid_height, grid_width), dtype=np.float32)
        self.miss_count = np.zeros((grid_height, grid_width), dtype=np.float32)

    def update_map(self, scan_points):
        """Update occupancy grid with current scan."""
        if not self.build_map or scan_points is None or len(scan_points) == 0:
            return

        # Robot position in world frame
        robot_x = self.x
        robot_y = self.y
        robot_theta = self.theta

        # Transform scan points to world frame
        c = np.cos(robot_theta)
        s = np.sin(robot_theta)
        world_points = np.column_stack([
            scan_points[:, 0] * c - scan_points[:, 1] * s + robot_x,
            scan_points[:, 0] * s + scan_points[:, 1] * c + robot_y
        ])

        # Convert robot position to grid coordinates
        robot_grid_x = int((robot_x - self.map_origin_x) / self.map_resolution)
        robot_grid_y = int((robot_y - self.map_origin_y) / self.map_resolution)

        if not (0 <= robot_grid_x < self.occupancy_grid.shape[1] and
                0 <= robot_grid_y < self.occupancy_grid.shape[0]):
            return  # Robot outside map bounds

        # Ray tracing for each point
        for point in world_points:
            # Convert point to grid coordinates
            point_grid_x = int((point[0] - self.map_origin_x) / self.map_resolution)
            point_grid_y = int((point[1] - self.map_origin_y) / self.map_resolution)

            # Check bounds
            if not (0 <= point_grid_x < self.occupancy_grid.shape[1] and
                    0 <= point_grid_y < self.occupancy_grid.shape[0]):
                continue

            # Bresenham's line algorithm for ray tracing
            cells = self.bresenham_line(robot_grid_x, robot_grid_y, point_grid_x, point_grid_y)

            # Mark cells along ray as free (except last one)
            for i, (cx, cy) in enumerate(cells[:-1]):
                if 0 <= cx < self.occupancy_grid.shape[1] and 0 <= cy < self.occupancy_grid.shape[0]:
                    self.miss_count[cy, cx] += 1

            # Mark endpoint as occupied
            if 0 <= point_grid_x < self.occupancy_grid.shape[1] and 0 <= point_grid_y < self.occupancy_grid.shape[0]:
                self.hit_count[point_grid_y, point_grid_x] += 1

        # Update occupancy probabilities (log-odds)
        total = self.hit_count + self.miss_count
        mask = total > 0
        prob = np.zeros_like(self.occupancy_grid, dtype=np.float32)
        prob[mask] = self.hit_count[mask] / total[mask]

        # Convert to occupancy grid values [0, 100]
        self.occupancy_grid[mask] = (prob[mask] * 100).astype(np.int8)

    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

        return cells

    def publish_map(self):
        """Publish occupancy grid map."""
        if not self.build_map or self.occupancy_grid is None:
            return

        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'odom'

        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.occupancy_grid.shape[1]
        map_msg.info.height = self.occupancy_grid.shape[0]
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0

        # Flatten map (row-major order)
        map_msg.data = self.occupancy_grid.flatten().tolist()

        self.map_pub.publish(map_msg)

    def save_map(self):
        """Save map as PNG image."""
        if not self.build_map or self.occupancy_grid is None:
            return

        output_dir = os.path.dirname(self.map_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        # Convert occupancy grid to image with thresholding
        # Threshold at 50% for clean binary look (like SLAM map)
        map_img = np.full_like(self.occupancy_grid, 205, dtype=np.uint8)  # unknown (gray)
        free = (self.occupancy_grid >= 0) & (self.occupancy_grid < 50)
        occupied = (self.occupancy_grid >= 50) & (self.occupancy_grid <= 100)
        map_img[free] = 254       # free → white
        map_img[occupied] = 0     # occupied → black

        # Crop to explored area (non-unknown cells) with padding
        explored = self.occupancy_grid != -1
        rows = np.any(explored, axis=1)
        cols = np.any(explored, axis=0)
        if rows.any() and cols.any():
            rmin, rmax = np.where(rows)[0][[0, -1]]
            cmin, cmax = np.where(cols)[0][[0, -1]]
            pad = 20  # pixels padding
            rmin = max(0, rmin - pad)
            rmax = min(map_img.shape[0] - 1, rmax + pad)
            cmin = max(0, cmin - pad)
            cmax = min(map_img.shape[1] - 1, cmax + pad)
            map_img = map_img[rmin:rmax+1, cmin:cmax+1]

        map_img = np.flipud(map_img)

        from PIL import Image
        img = Image.fromarray(map_img, mode='L')
        img.save(self.map_file)

        self.get_logger().info(f'Map saved to: {self.map_file} ({map_img.shape[1]}x{map_img.shape[0]})')

    # ==================== Local Map ====================

    def transform_to_odom(self, points, x, y, theta):
        """Transform body-frame points to odom frame."""
        c = np.cos(theta)
        s = np.sin(theta)
        return np.column_stack([
            points[:, 0] * c - points[:, 1] * s + x,
            points[:, 0] * s + points[:, 1] * c + y
        ])

    def rebuild_local_map(self):
        """Merge all keyframe scans and downsample via voxel grid."""
        if len(self.local_map_scans) == 0:
            self.local_map_points = None
            self.local_map_tree = None
            return

        # Merge all scans
        all_points = np.vstack(list(self.local_map_scans))

        # Voxel grid downsample
        if self.local_map_voxel_size > 0:
            all_points = self.voxel_downsample(all_points, self.local_map_voxel_size)

        self.local_map_points = all_points
        self.local_map_tree = KDTree(all_points)
        self.local_map_dirty = False

    def voxel_downsample(self, points, voxel_size):
        """Simple 2D voxel grid downsampling."""
        # Quantize to voxel grid
        voxel_indices = np.floor(points / voxel_size).astype(int)

        # Use dictionary to keep one point per voxel (centroid)
        voxel_dict = {}
        for i in range(len(points)):
            key = (voxel_indices[i, 0], voxel_indices[i, 1])
            if key not in voxel_dict:
                voxel_dict[key] = []
            voxel_dict[key].append(points[i])

        # Compute centroid of each voxel
        downsampled = np.array([np.mean(pts, axis=0) for pts in voxel_dict.values()])
        return downsampled

    # ==================== Scan-to-Map ICP ====================

    def icp_scan_to_map(self, scan_points, init_x, init_y, init_theta):
        """
        Match scan (body frame) against local map (odom frame).

        Optimizes the robot pose (x, y, theta) such that the transformed
        scan best aligns with the local map.

        Returns: (x, y, theta, success)
        """
        x = init_x
        y = init_y
        theta = init_theta

        prev_error = float('inf')

        for _ in range(self.icp_max_iterations):
            # Transform scan to odom frame using current pose estimate
            c = np.cos(theta)
            s = np.sin(theta)
            transformed = np.column_stack([
                scan_points[:, 0] * c - scan_points[:, 1] * s + x,
                scan_points[:, 0] * s + scan_points[:, 1] * c + y
            ])

            # Find correspondences in local map
            dists, indices = self.local_map_tree.query(transformed)
            valid = dists < self.icp_max_correspondence_distance
            n_valid = np.sum(valid)

            if n_valid < self.min_correspondences:
                return init_x, init_y, init_theta, False

            # Trimmed: use best 80%
            if n_valid > 30:
                valid_dists = dists[valid]
                trim_thresh = np.percentile(valid_dists, 80)
                trim_mask = np.zeros(len(dists), dtype=bool)
                trim_mask[valid] = dists[valid] <= trim_thresh
                valid = trim_mask
                n_valid = np.sum(valid)
                if n_valid < self.min_correspondences:
                    return init_x, init_y, init_theta, False

            mean_error = np.mean(dists[valid])
            if abs(prev_error - mean_error) < self.icp_tolerance:
                break
            prev_error = mean_error

            # SVD on original scan points (body frame) vs map points (odom frame)
            src_m = scan_points[valid]  # body frame
            tgt_m = self.local_map_points[indices[valid]]  # odom frame

            src_c = np.mean(src_m, axis=0)
            tgt_c = np.mean(tgt_m, axis=0)

            W = (tgt_m - tgt_c).T @ (src_m - src_c)
            U, _, Vt = np.linalg.svd(W)

            d = np.linalg.det(U @ Vt)
            R_opt = U @ np.diag([1.0, np.sign(d)]) @ Vt
            t_opt = tgt_c - R_opt @ src_c

            theta = np.arctan2(R_opt[1, 0], R_opt[0, 0])
            x = t_opt[0]
            y = t_opt[1]

        return x, y, theta, True

    # ==================== Utilities ====================

    def scan_to_pointcloud(self, scan_msg):
        ranges = np.array(scan_msg.ranges, dtype=np.float64)
        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment
        valid = ((ranges > scan_msg.range_min) & (ranges < scan_msg.range_max)
                 & np.isfinite(ranges) & (ranges > 0.12))
        ranges, angles = ranges[valid], angles[valid]
        if len(ranges) == 0:
            return np.empty((0, 2))
        return np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])

    def extract_pose(self, odom_msg):
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        q = odom_msg.pose.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return x, y, yaw

    # ==================== Publishing ====================

    def publish_odom_and_path(self, timestamp):
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)

        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint_icp'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint_icp'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        pose = PoseStamped()
        pose.header.stamp = timestamp.to_msg()
        pose.header.frame_id = 'odom'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.path_msg.poses.append(pose)
        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)
        self.path_msg.header.stamp = timestamp.to_msg()
        self.path_pub.publish(self.path_msg)

    def save_trajectory(self):
        if len(self.trajectory) == 0:
            return
        if self.end_time is None:
            self.end_time = self.get_clock().now()

        output_dir = os.path.dirname(self.output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir, exist_ok=True)

        np.savetxt(self.output_file, np.array(self.trajectory),
                   fmt='%.6f', header='timestamp x y theta', comments='')

        traj = np.array(self.trajectory)
        total_dist = np.sum(np.sqrt(np.diff(traj[:, 1])**2 + np.diff(traj[:, 2])**2))
        loop_err = np.sqrt(self.x**2 + self.y**2)

        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'ICP SUMMARY [Scan-to-Map] - Seq {self.sequence}')
        self.get_logger().info(f'{"="*60}')
        self.get_logger().info(f'Updates: {self._update_count} | Keyframes: {self._keyframe_count}')
        self.get_logger().info(f'ICP ok: {self._icp_success_count} | FB: {self._fallback_count}')
        self.get_logger().info(f'Dist: {total_dist:.2f}m')
        self.get_logger().info(f'Final: ({self.x:.3f}, {self.y:.3f})m θ={np.degrees(self.theta):.2f}°')
        self.get_logger().info(f'Loop closure error: {loop_err:.3f}m')
        self.get_logger().info(f'{"="*60}')

        # Save map
        if self.build_map:
            self.save_map()

    def __del__(self):
        try:
            self.save_trajectory()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ICPOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save_trajectory()
        except:
            pass
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