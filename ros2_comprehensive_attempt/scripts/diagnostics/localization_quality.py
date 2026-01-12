#!/usr/bin/env python3
"""
Localization Quality Checker

Monitors and analyzes AMCL localization quality:
- Particle filter convergence
- Pose covariance analysis
- Position uncertainty tracking
- Localization degradation detection

Usage:
    python3 localization_quality.py
    python3 localization_quality.py --monitor 10  # Monitor for 10 seconds
    python3 localization_quality.py --export quality_report.txt
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry

import time
import math
import argparse
import numpy as np
from collections import deque
from typing import List, Tuple, Optional


class LocalizationQualityChecker(Node):
    """Monitor AMCL localization quality."""

    def __init__(self):
        super().__init__('localization_quality_checker')

        # Data storage
        self.pose_history = deque(maxlen=100)
        self.covariance_history = deque(maxlen=100)
        self.particle_cloud = None
        self.current_pose = None
        self.odom_pose = None

        # Statistics
        self.start_time = time.time()
        self.total_drift = 0.0
        self.max_uncertainty = 0.0
        self.min_uncertainty = float('inf')

        # Subscriptions
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )

        self.create_subscription(
            PoseArray,
            '/particlecloud',
            self._particle_cloud_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        self.get_logger().info("Localization quality checker started")

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose updates."""
        current_time = time.time()

        # Store pose
        pose_data = {
            'time': current_time,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w,
        }
        self.pose_history.append(pose_data)
        self.current_pose = msg

        # Store covariance
        cov = msg.pose.covariance
        cov_data = {
            'time': current_time,
            'xx': cov[0],   # x variance
            'yy': cov[7],   # y variance
            'aa': cov[35],  # yaw variance
            'xy': cov[1],   # x-y covariance
        }
        self.covariance_history.append(cov_data)

        # Calculate position uncertainty
        pos_uncertainty = math.sqrt(cov[0] + cov[7])
        self.max_uncertainty = max(self.max_uncertainty, pos_uncertainty)
        self.min_uncertainty = min(self.min_uncertainty, pos_uncertainty)

        # Calculate drift if we have history
        if len(self.pose_history) >= 2:
            prev = self.pose_history[-2]
            dx = pose_data['x'] - prev['x']
            dy = pose_data['y'] - prev['y']
            drift = math.sqrt(dx**2 + dy**2)
            self.total_drift += drift

    def _particle_cloud_callback(self, msg: PoseArray):
        """Handle particle cloud updates."""
        self.particle_cloud = msg

    def _odom_callback(self, msg: Odometry):
        """Handle odometry updates."""
        self.odom_pose = msg

    def calculate_particle_spread(self) -> Tuple[float, float]:
        """Calculate particle cloud spread."""
        if not self.particle_cloud or len(self.particle_cloud.poses) == 0:
            return 0.0, 0.0

        # Extract particle positions
        x_positions = [p.position.x for p in self.particle_cloud.poses]
        y_positions = [p.position.y for p in self.particle_cloud.poses]

        # Calculate standard deviation
        x_std = np.std(x_positions) if len(x_positions) > 1 else 0.0
        y_std = np.std(y_positions) if len(y_positions) > 1 else 0.0

        # Overall spread
        spread = math.sqrt(x_std**2 + y_std**2)

        return spread, len(self.particle_cloud.poses)

    def calculate_convergence_rate(self) -> float:
        """Calculate how quickly localization is converging."""
        if len(self.covariance_history) < 10:
            return 0.0

        # Compare recent covariance to older covariance
        recent = list(self.covariance_history)[-10:]
        older = list(self.covariance_history)[:10]

        recent_avg = np.mean([math.sqrt(c['xx'] + c['yy']) for c in recent])
        older_avg = np.mean([math.sqrt(c['xx'] + c['yy']) for c in older])

        if older_avg > 0:
            convergence = (older_avg - recent_avg) / older_avg
            return convergence
        return 0.0

    def calculate_pose_stability(self) -> float:
        """Calculate pose stability (low variance = stable)."""
        if len(self.pose_history) < 10:
            return 0.0

        recent_poses = list(self.pose_history)[-10:]
        x_positions = [p['x'] for p in recent_poses]
        y_positions = [p['y'] for p in recent_poses]

        # Calculate variance
        x_var = np.var(x_positions)
        y_var = np.var(y_positions)

        return math.sqrt(x_var + y_var)

    def get_localization_status(self) -> dict:
        """Get comprehensive localization status."""
        if not self.current_pose:
            return {'status': 'NO_DATA'}

        # Get covariance
        cov = self.current_pose.pose.covariance
        pos_uncertainty = math.sqrt(cov[0] + cov[7])
        orient_uncertainty = math.sqrt(cov[35])

        # Particle spread
        particle_spread, particle_count = self.calculate_particle_spread()

        # Convergence
        convergence = self.calculate_convergence_rate()

        # Stability
        stability = self.calculate_pose_stability()

        # Determine quality
        if pos_uncertainty < 0.1:
            quality = "EXCELLENT"
            quality_score = 5
        elif pos_uncertainty < 0.3:
            quality = "GOOD"
            quality_score = 4
        elif pos_uncertainty < 0.5:
            quality = "FAIR"
            quality_score = 3
        elif pos_uncertainty < 1.0:
            quality = "POOR"
            quality_score = 2
        else:
            quality = "VERY_POOR"
            quality_score = 1

        return {
            'status': 'ACTIVE',
            'quality': quality,
            'quality_score': quality_score,
            'position_uncertainty': pos_uncertainty,
            'orientation_uncertainty': orient_uncertainty,
            'particle_spread': particle_spread,
            'particle_count': particle_count,
            'convergence_rate': convergence,
            'pose_stability': stability,
            'total_drift': self.total_drift,
        }

    def print_report(self):
        """Print localization quality report."""
        status = self.get_localization_status()

        print("\n" + "=" * 70)
        print("LOCALIZATION QUALITY REPORT")
        print("=" * 70)

        if status['status'] == 'NO_DATA':
            print("No localization data received")
            return

        print(f"\nQuality: {status['quality']} ({status['quality_score']}/5)")
        print("\nMetrics:")
        print(f"  Position Uncertainty:    {status['position_uncertainty']:.4f} m")
        print(f"  Orientation Uncertainty: {math.degrees(status['orientation_uncertainty']):.2f}°")
        print(f"  Particle Count:          {status['particle_count']}")
        print(f"  Particle Spread:         {status['particle_spread']:.4f} m")
        print(f"  Pose Stability:          {status['pose_stability']:.4f} m")
        print(f"  Convergence Rate:        {status['convergence_rate']*100:.1f}%")

        print(f"\nStatistics:")
        print(f"  Runtime:             {time.time() - self.start_time:.1f}s")
        print(f"  Total Drift:         {status['total_drift']:.3f}m")
        print(f"  Max Uncertainty:     {self.max_uncertainty:.4f}m")
        print(f"  Min Uncertainty:     {self.min_uncertainty:.4f}m")

        # Recommendations
        print("\nRecommendations:")
        if status['quality_score'] >= 4:
            print("  ✓ Localization quality is good")
        elif status['quality_score'] == 3:
            print("  ⚠ Consider re-localizing if navigation becomes erratic")
        else:
            print("  ✗ Poor localization quality detected:")
            if status['position_uncertainty'] > 0.5:
                print("    - High position uncertainty: Use 2D Pose Estimate in RViz")
            if status['particle_spread'] > 1.0:
                print("    - High particle spread: Robot may be lost")
            if status['pose_stability'] > 0.1:
                print("    - Unstable pose: Check for sensor issues or map mismatch")

        print("=" * 70)

        return status

    def monitor(self, duration: float):
        """Monitor for specified duration."""
        print(f"Monitoring localization quality for {duration}s...")
        print("(Press Ctrl+C to stop early)\n")

        end_time = time.time() + duration

        try:
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=1.0)

                # Print status every 2 seconds
                if int(time.time()) % 2 == 0:
                    status = self.get_localization_status()
                    if status['status'] == 'ACTIVE':
                        remaining = int(end_time - time.time())
                        print(f"[{remaining:3d}s] Quality: {status['quality']:12s} "
                              f"Uncertainty: {status['position_uncertainty']:.4f}m "
                              f"Particles: {status['particle_count']:4d}",
                              end='\r')

        except KeyboardInterrupt:
            print("\nMonitoring stopped")

        # Final report
        self.print_report()


def main():
    parser = argparse.ArgumentParser(description='Localization Quality Checker')
    parser.add_argument('--monitor', '-m', type=float,
                       help='Monitor for specified duration (seconds)')
    parser.add_argument('--export', '-e', type=str,
                       help='Export report to file')

    args = parser.parse_args()

    rclpy.init()
    node = LocalizationQualityChecker()

    try:
        if args.monitor:
            node.monitor(args.monitor)
        else:
            # Single snapshot
            print("Collecting data for 5 seconds...")
            start = time.time()
            while time.time() - start < 5.0:
                rclpy.spin_once(node, timeout_sec=0.1)

        status = node.print_report()

        # Export if requested
        if args.export and status:
            with open(args.export, 'w') as f:
                f.write(f"Localization Quality Report\n")
                f.write(f"Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write(f"\n")
                f.write(f"Quality: {status['quality']}\n")
                f.write(f"Position Uncertainty: {status['position_uncertainty']:.4f}m\n")
                f.write(f"Particle Count: {status['particle_count']}\n")
                f.write(f"Particle Spread: {status['particle_spread']:.4f}m\n")
            print(f"\nReport exported to: {args.export}")

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
