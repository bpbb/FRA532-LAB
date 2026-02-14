#!/usr/bin/env python3
"""
Comprehensive Trajectory Analysis and Comparison Script

Computes accuracy, drift, and robustness metrics for different odometry methods:
- Wheel Odometry
- EKF Odometry (Wheel + IMU fusion)
- ICP Odometry (EKF + LiDAR scan matching)
- SLAM (Full SLAM with loop closure)

Generates:
- Trajectory comparison plots
- Quantitative metrics table
- Drift analysis
- Discussion and recommendations
"""

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
from pathlib import Path
import pandas as pd
from matplotlib.patches import Rectangle
from matplotlib.gridspec import GridSpec


class TrajectoryAnalyzer:
    def __init__(self, results_dir='results'):
        self.results_dir = results_dir
        self.trajectories = {}
        self.metrics = {}

    def load_trajectory(self, method, sequence):
        """Load trajectory file for a specific method and sequence."""
        filename_map = {
            'wheel': f'trajectory_wheel_seq{sequence}.txt',
            'ekf': f'trajectory_ekf_seq{sequence}.txt',
            'icp': f'trajectory_icp_seq{sequence}.txt',
            'slam': f'trajectory_slam_seq{sequence}.txt'
        }

        filepath = os.path.join(self.results_dir, filename_map[method])
        try:
            data = np.loadtxt(filepath, skiprows=1)
            if data.size == 0:
                return None
            return data
        except Exception as e:
            print(f"  Warning: Could not load {method} for seq{sequence}: {e}")
            return None

    def load_all_trajectories(self, sequence):
        """Load all available trajectories for a sequence."""
        print(f"\nLoading trajectories for sequence {sequence}...")
        methods = ['wheel', 'ekf', 'icp', 'slam']

        for method in methods:
            traj = self.load_trajectory(method, sequence)
            if traj is not None:
                self.trajectories[method] = traj
                print(f"  ✓ {method.upper()}: {len(traj)} points")

        if not self.trajectories:
            print(f"  ⚠ No trajectories found for sequence {sequence}")
            return False
        return True

    def compute_trajectory_length(self, traj):
        """Compute total path length."""
        dx = np.diff(traj[:, 1])
        dy = np.diff(traj[:, 2])
        distances = np.sqrt(dx**2 + dy**2)
        return np.sum(distances)

    def compute_loop_closure_error(self, traj):
        """Compute distance between start and end positions."""
        start = traj[0, 1:3]
        end = traj[-1, 1:3]
        return np.linalg.norm(end - start)

    def compute_drift_over_time(self, traj, reference_traj=None):
        """
        Compute drift metrics.
        If reference_traj is provided, compute error relative to reference.
        Otherwise, compute drift from start position.
        """
        if reference_traj is not None:
            # Interpolate reference to match timestamps
            ref_x = np.interp(traj[:, 0], reference_traj[:, 0], reference_traj[:, 1])
            ref_y = np.interp(traj[:, 0], reference_traj[:, 0], reference_traj[:, 2])
            errors = np.sqrt((traj[:, 1] - ref_x)**2 + (traj[:, 2] - ref_y)**2)
            return {
                'mean_error': np.mean(errors),
                'max_error': np.max(errors),
                'final_error': errors[-1],
                'std_error': np.std(errors)
            }
        else:
            # Drift from origin
            distances = np.sqrt(traj[:, 1]**2 + traj[:, 2]**2)
            return {
                'max_distance': np.max(distances),
                'final_distance': distances[-1]
            }

    def compute_heading_consistency(self, traj):
        """Compute heading variation and smoothness."""
        if traj.shape[1] < 4:
            return {'heading_std': 0, 'heading_jumps': 0}

        headings = traj[:, 3]
        # Unwrap to avoid -pi/pi discontinuities
        headings_unwrapped = np.unwrap(headings)

        # Compute heading changes
        dheading = np.diff(headings_unwrapped)

        return {
            'heading_std': np.std(headings_unwrapped),
            'heading_jumps': np.sum(np.abs(dheading) > np.radians(10)),  # Count large jumps
            'mean_heading_change': np.mean(np.abs(dheading))
        }

    def compute_velocity_statistics(self, traj):
        """Compute velocity statistics."""
        dt = np.diff(traj[:, 0])
        dx = np.diff(traj[:, 1])
        dy = np.diff(traj[:, 2])

        # Remove zero dt to avoid division issues
        valid = dt > 1e-6
        dt = dt[valid]
        dx = dx[valid]
        dy = dy[valid]

        velocities = np.sqrt(dx**2 + dy**2) / dt

        return {
            'mean_velocity': np.mean(velocities),
            'max_velocity': np.max(velocities),
            'velocity_std': np.std(velocities)
        }

    def compute_all_metrics(self, sequence):
        """Compute all metrics for all loaded trajectories."""
        print(f"\nComputing metrics for sequence {sequence}...")

        # Use SLAM as reference if available (most accurate)
        reference = self.trajectories.get('slam', None)

        for method, traj in self.trajectories.items():
            print(f"  Processing {method.upper()}...")

            metrics = {}

            # Basic metrics
            metrics['trajectory_length'] = self.compute_trajectory_length(traj)
            metrics['loop_closure_error'] = self.compute_loop_closure_error(traj)
            metrics['duration'] = traj[-1, 0] - traj[0, 0]
            metrics['num_points'] = len(traj)

            # Drift metrics
            if reference is not None and method != 'slam':
                drift = self.compute_drift_over_time(traj, reference)
                metrics.update(drift)

            # Heading consistency
            heading = self.compute_heading_consistency(traj)
            metrics.update(heading)

            # Velocity statistics
            velocity = self.compute_velocity_statistics(traj)
            metrics.update(velocity)

            # Drift rate (loop closure error per distance)
            if metrics['trajectory_length'] > 0:
                metrics['drift_rate_percent'] = (metrics['loop_closure_error'] /
                                                 metrics['trajectory_length'] * 100)
            else:
                metrics['drift_rate_percent'] = 0

            self.metrics[method] = metrics

        print("  ✓ Metrics computation complete")

    def print_metrics_table(self, sequence):
        """Print formatted metrics table."""
        if not self.metrics:
            print("No metrics to display")
            return

        print(f"\n{'='*80}")
        print(f"TRAJECTORY COMPARISON - Sequence {sequence}")
        print(f"{'='*80}\n")

        # Create pandas DataFrame for better formatting
        df_data = {}
        for method, metrics in self.metrics.items():
            df_data[method.upper()] = metrics

        df = pd.DataFrame(df_data).T

        # Select and order columns
        basic_cols = ['num_points', 'duration', 'trajectory_length', 'loop_closure_error',
                     'drift_rate_percent']
        accuracy_cols = ['mean_error', 'max_error', 'final_error'] if 'mean_error' in df.columns else []
        velocity_cols = ['mean_velocity', 'max_velocity']
        heading_cols = ['heading_jumps', 'mean_heading_change']

        print("BASIC METRICS:")
        print("-" * 80)
        if all(col in df.columns for col in basic_cols):
            print(df[basic_cols].to_string())

        if accuracy_cols and all(col in df.columns for col in accuracy_cols):
            print(f"\nACCURACY (vs SLAM reference):")
            print("-" * 80)
            print(df[accuracy_cols].to_string())

        print(f"\nVELOCITY STATISTICS:")
        print("-" * 80)
        if all(col in df.columns for col in velocity_cols):
            print(df[velocity_cols].to_string())

        print(f"\nHEADING CONSISTENCY:")
        print("-" * 80)
        if all(col in df.columns for col in heading_cols):
            print(df[heading_cols].to_string())

        print(f"\n{'='*80}\n")

    def generate_comparison_plots(self, sequence, output_dir='results/analysis'):
        """Generate comprehensive comparison plots."""
        os.makedirs(output_dir, exist_ok=True)

        # Define colors for each method
        colors = {
            'wheel': '#ff7f0e',  # Orange
            'ekf': '#1f77b4',    # Blue
            'icp': '#2ca02c',    # Green
            'slam': '#d62728'    # Red
        }

        labels = {
            'wheel': 'Wheel Odometry',
            'ekf': 'EKF (Wheel+IMU)',
            'icp': 'ICP (EKF+LiDAR)',
            'slam': 'SLAM (Full SLAM)'
        }

        # Create figure with subplots
        fig = plt.figure(figsize=(20, 12))
        gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

        # 1. Main 2D trajectory plot (large, left side)
        ax1 = fig.add_subplot(gs[:, 0:2])
        for method, traj in self.trajectories.items():
            ax1.plot(traj[:, 1], traj[:, 2], label=labels[method],
                    color=colors[method], linewidth=2.5, alpha=0.8)
            # Mark start and end
            ax1.plot(traj[0, 1], traj[0, 2], 'o', color=colors[method],
                    markersize=12, markeredgecolor='black', markeredgewidth=2)
            ax1.plot(traj[-1, 1], traj[-1, 2], 's', color=colors[method],
                    markersize=12, markeredgecolor='black', markeredgewidth=2)

        ax1.set_xlabel('X Position (m)', fontsize=14, fontweight='bold')
        ax1.set_ylabel('Y Position (m)', fontsize=14, fontweight='bold')
        ax1.set_title(f'2D Trajectory Comparison - Sequence {sequence}',
                     fontsize=16, fontweight='bold', pad=20)
        ax1.legend(fontsize=12, loc='best', frameon=True, shadow=True, fancybox=True)
        ax1.grid(True, linestyle='--', alpha=0.6)
        ax1.axis('equal')

        # 2. X position over time
        ax2 = fig.add_subplot(gs[0, 2])
        for method, traj in self.trajectories.items():
            time = traj[:, 0] - traj[0, 0]
            ax2.plot(time, traj[:, 1], label=labels[method],
                    color=colors[method], linewidth=2, alpha=0.8)
        ax2.set_ylabel('X Position (m)', fontsize=11)
        ax2.set_title('X Position vs Time', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.4)
        ax2.legend(fontsize=8)

        # 3. Y position over time
        ax3 = fig.add_subplot(gs[1, 2])
        for method, traj in self.trajectories.items():
            time = traj[:, 0] - traj[0, 0]
            ax3.plot(time, traj[:, 2], label=labels[method],
                    color=colors[method], linewidth=2, alpha=0.8)
        ax3.set_ylabel('Y Position (m)', fontsize=11)
        ax3.set_title('Y Position vs Time', fontsize=12, fontweight='bold')
        ax3.grid(True, alpha=0.4)

        # 4. Error vs SLAM (if available)
        ax4 = fig.add_subplot(gs[2, 2])
        if 'slam' in self.trajectories:
            slam_traj = self.trajectories['slam']
            for method, traj in self.trajectories.items():
                if method != 'slam':
                    # Interpolate SLAM to match timestamps
                    slam_x = np.interp(traj[:, 0], slam_traj[:, 0], slam_traj[:, 1])
                    slam_y = np.interp(traj[:, 0], slam_traj[:, 0], slam_traj[:, 2])
                    errors = np.sqrt((traj[:, 1] - slam_x)**2 + (traj[:, 2] - slam_y)**2)
                    time = traj[:, 0] - traj[0, 0]
                    ax4.plot(time, errors, label=labels[method],
                            color=colors[method], linewidth=2, alpha=0.8)
            ax4.set_xlabel('Time (s)', fontsize=11)
            ax4.set_ylabel('Error vs SLAM (m)', fontsize=11)
            ax4.set_title('Position Error Relative to SLAM', fontsize=12, fontweight='bold')
            ax4.grid(True, alpha=0.4)
            ax4.legend(fontsize=8)
        else:
            ax4.text(0.5, 0.5, 'SLAM reference not available',
                    ha='center', va='center', fontsize=12)
            ax4.axis('off')

        plt.savefig(os.path.join(output_dir, f'trajectory_comparison_seq{sequence}.png'),
                   dpi=300, bbox_inches='tight')
        print(f"  ✓ Saved comparison plots to {output_dir}")
        plt.close()

    def generate_metrics_bar_chart(self, sequence, output_dir='results/analysis'):
        """Generate bar charts comparing accuracy vs SLAM reference."""
        os.makedirs(output_dir, exist_ok=True)

        if 'slam' not in self.trajectories:
            print("  ⚠ SLAM trajectory not available, skipping metrics bar chart")
            return

        slam_traj = self.trajectories['slam']
        methods_no_slam = [m for m in self.metrics.keys() if m != 'slam']
        colors_map = {'wheel': '#ff7f0e', 'ekf': '#1f77b4', 'icp': '#2ca02c'}
        colors = [colors_map.get(m, '#999999') for m in methods_no_slam]

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle(f'Accuracy vs SLAM Reference - Sequence {sequence}',
                    fontsize=16, fontweight='bold')

        # 1. Mean Position Error vs SLAM
        ax = axes[0]
        values = [self.metrics[m].get('mean_error', 0) for m in methods_no_slam]
        bars = ax.bar(methods_no_slam, values, color=colors, alpha=0.7, edgecolor='black')
        ax.set_ylabel('Mean Error (m)', fontsize=12)
        ax.set_title('Mean Position Error vs SLAM', fontsize=13, fontweight='bold')
        ax.grid(axis='y', alpha=0.4)
        for bar, val in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                   f'{val:.3f}m', ha='center', va='bottom', fontsize=10)

        # 2. Heading Drift (angular difference between start and end heading)
        ax = axes[1]
        all_methods = list(self.metrics.keys())
        all_colors = [colors_map.get(m, '#d62728') for m in all_methods]
        values = []
        for m in all_methods:
            traj = self.trajectories[m]
            drift = traj[-1, 3] - traj[0, 3]
            drift = (drift + np.pi) % (2 * np.pi) - np.pi
            values.append(np.degrees(abs(drift)))
        bars = ax.bar(all_methods, values, color=all_colors, alpha=0.7, edgecolor='black')
        ax.set_ylabel('Heading Drift (deg)', fontsize=12)
        ax.set_title('Heading Drift (Start vs End)', fontsize=13, fontweight='bold')
        ax.grid(axis='y', alpha=0.4)
        for bar, val in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                   f'{val:.1f}°', ha='center', va='bottom', fontsize=10)

        # 3. Trajectory Length comparison (include SLAM)
        ax = axes[2]
        all_methods = list(self.metrics.keys())
        all_colors = [colors_map.get(m, '#d62728') for m in all_methods]
        values = [self.metrics[m]['trajectory_length'] for m in all_methods]
        bars = ax.bar(all_methods, values, color=all_colors, alpha=0.7, edgecolor='black')
        ax.set_ylabel('Distance (m)', fontsize=12)
        ax.set_title('Total Trajectory Length', fontsize=13, fontweight='bold')
        ax.grid(axis='y', alpha=0.4)
        for bar, val in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width()/2., bar.get_height(),
                   f'{val:.2f}m', ha='center', va='bottom', fontsize=10)

        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, f'metrics_comparison_seq{sequence}.png'),
                   dpi=300, bbox_inches='tight')
        print(f"  ✓ Saved metrics bar chart")
        plt.close()

    def run_full_analysis(self, sequence):
        """Run complete analysis pipeline."""
        print(f"\n{'='*80}")
        print(f"COMPREHENSIVE TRAJECTORY ANALYSIS - SEQUENCE {sequence}")
        print(f"{'='*80}")

        # Load trajectories
        if not self.load_all_trajectories(sequence):
            return

        # Compute metrics
        self.compute_all_metrics(sequence)

        # Print metrics
        self.print_metrics_table(sequence)

        # Generate plots and discussion
        print("\nGenerating visualizations...")
        self.generate_comparison_plots(sequence)
        self.generate_metrics_bar_chart(sequence)

        print(f"\n{'='*80}")
        print("ANALYSIS COMPLETE")
        print(f"{'='*80}\n")


def main():
    parser = argparse.ArgumentParser(
        description='Comprehensive trajectory analysis and comparison')
    parser.add_argument('--sequences', nargs='+', default=['00'],
                       help='Sequence numbers to analyze (e.g., 00 01 02)')
    parser.add_argument('--results-dir', default='results',
                       help='Directory containing trajectory files')
    parser.add_argument('--output-dir', default='results/analysis',
                       help='Directory to save analysis results')

    args = parser.parse_args()

    for seq in args.sequences:
        analyzer = TrajectoryAnalyzer(results_dir=args.results_dir)
        analyzer.run_full_analysis(seq)


if __name__ == '__main__':
    main()
