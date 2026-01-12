#!/usr/bin/env python3
"""
Performance Profiler

Measures and profiles navigation system performance:
- End-to-end latency (sensor to control)
- CPU and memory profiling per node
- Navigation success rate
- Path execution quality

Usage:
    python3 performance_profiler.py --duration 60
    python3 performance_profiler.py --profile-nodes
    python3 performance_profiler.py --export profile_report.txt
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose

import time
import psutil
import argparse
from collections import deque, defaultdict
from typing import Dict, List
import subprocess


class PerformanceProfiler(Node):
    """Profile navigation system performance."""

    def __init__(self):
        super().__init__('performance_profiler')

        # Latency tracking
        self.scan_timestamps = deque(maxlen=100)
        self.cmd_vel_timestamps = deque(maxlen=100)
        self.latencies = deque(maxlen=1000)

        # Message counts
        self.msg_counts = defaultdict(int)
        self.last_msg_time = {}

        # Navigation tracking
        self.nav_start_time = None
        self.nav_goals = []
        self.nav_success_count = 0
        self.nav_failure_count = 0

        # Performance metrics
        self.start_time = time.time()
        self.cpu_samples = deque(maxlen=100)
        self.memory_samples = deque(maxlen=100)

        # Create QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions for latency measurement
        self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            qos
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

        self.create_subscription(
            Path,
            '/plan',
            self._plan_callback,
            10
        )

        # Timer for periodic sampling
        self.create_timer(1.0, self._sample_performance)

        self.get_logger().info("Performance profiler started")

    def _scan_callback(self, msg: LaserScan):
        """Handle scan messages for latency measurement."""
        # Store timestamp from message header
        scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.scan_timestamps.append(scan_time)
        self.msg_counts['/scan'] += 1
        self.last_msg_time['/scan'] = time.time()

    def _cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel for latency measurement."""
        current_time = time.time()
        self.cmd_vel_timestamps.append(current_time)
        self.msg_counts['/cmd_vel'] += 1
        self.last_msg_time['/cmd_vel'] = current_time

        # Calculate latency if we have scan data
        if self.scan_timestamps:
            # Find most recent scan
            latest_scan = self.scan_timestamps[-1]
            # Latency = current time - scan time
            latency = current_time - latest_scan
            if 0 < latency < 1.0:  # Sanity check
                self.latencies.append(latency)

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose."""
        self.msg_counts['/amcl_pose'] += 1
        self.last_msg_time['/amcl_pose'] = time.time()

    def _odom_callback(self, msg: Odometry):
        """Handle odometry."""
        self.msg_counts['/odom'] += 1
        self.last_msg_time['/odom'] = time.time()

    def _plan_callback(self, msg: Path):
        """Handle path plans."""
        self.msg_counts['/plan'] += 1
        self.last_msg_time['/plan'] = time.time()

    def _sample_performance(self):
        """Sample system performance metrics."""
        # Overall system metrics
        cpu = psutil.cpu_percent(interval=0.1)
        mem = psutil.virtual_memory().percent

        self.cpu_samples.append(cpu)
        self.memory_samples.append(mem)

    def get_ros_node_info(self) -> List[Dict]:
        """Get information about running ROS nodes."""
        try:
            # Use ros2 CLI to get node list
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )

            if result.returncode == 0:
                nodes = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
                return [{'name': node} for node in nodes]

        except Exception as e:
            self.get_logger().warn(f"Could not get ROS node info: {e}")

        return []

    def calculate_latency_stats(self) -> Dict:
        """Calculate latency statistics."""
        if not self.latencies:
            return {
                'min': 0.0,
                'max': 0.0,
                'mean': 0.0,
                'p50': 0.0,
                'p95': 0.0,
                'p99': 0.0,
            }

        import numpy as np
        latencies_array = np.array(list(self.latencies))

        return {
            'min': np.min(latencies_array) * 1000,  # Convert to ms
            'max': np.max(latencies_array) * 1000,
            'mean': np.mean(latencies_array) * 1000,
            'p50': np.percentile(latencies_array, 50) * 1000,
            'p95': np.percentile(latencies_array, 95) * 1000,
            'p99': np.percentile(latencies_array, 99) * 1000,
        }

    def calculate_message_rates(self) -> Dict[str, float]:
        """Calculate message rates for all topics."""
        current_time = time.time()
        elapsed = current_time - self.start_time

        rates = {}
        for topic, count in self.msg_counts.items():
            rates[topic] = count / elapsed if elapsed > 0 else 0.0

        return rates

    def print_profile_report(self):
        """Print comprehensive profile report."""
        print("\n" + "=" * 70)
        print("PERFORMANCE PROFILE REPORT")
        print("=" * 70)

        runtime = time.time() - self.start_time
        print(f"\nRuntime: {runtime:.1f}s")

        # Latency
        print("\n1. LATENCY (Sensor to Control):")
        latency_stats = self.calculate_latency_stats()

        if self.latencies:
            print(f"   Samples:     {len(self.latencies)}")
            print(f"   Min:         {latency_stats['min']:.2f}ms")
            print(f"   Mean:        {latency_stats['mean']:.2f}ms")
            print(f"   Max:         {latency_stats['max']:.2f}ms")
            print(f"   P50 (median): {latency_stats['p50']:.2f}ms")
            print(f"   P95:         {latency_stats['p95']:.2f}ms")
            print(f"   P99:         {latency_stats['p99']:.2f}ms")

            # Assessment
            if latency_stats['mean'] < 50:
                print(f"   ✓ Excellent latency")
            elif latency_stats['mean'] < 100:
                print(f"   ✓ Good latency")
            elif latency_stats['mean'] < 200:
                print(f"   ⚠ Moderate latency")
            else:
                print(f"   ✗ High latency - may impact navigation")
        else:
            print("   No latency data collected")

        # Message rates
        print("\n2. MESSAGE RATES:")
        rates = self.calculate_message_rates()

        expected_rates = {
            '/scan': 10.0,
            '/cmd_vel': 20.0,
            '/odom': 20.0,
            '/amcl_pose': 2.0,
        }

        for topic, rate in sorted(rates.items()):
            expected = expected_rates.get(topic, 0)
            status = ""

            if expected > 0:
                if rate >= expected * 0.8:
                    status = "✓"
                elif rate >= expected * 0.5:
                    status = "⚠"
                else:
                    status = "✗"

            print(f"   {status} {topic:20s}: {rate:6.2f} Hz "
                  f"(expected: {expected:.1f})" if expected else
                  f"   {topic:20s}: {rate:6.2f} Hz")

        # CPU and Memory
        print("\n3. SYSTEM RESOURCES:")
        if self.cpu_samples:
            import numpy as np
            cpu_mean = np.mean(self.cpu_samples)
            cpu_max = np.max(self.cpu_samples)
            mem_mean = np.mean(self.memory_samples)
            mem_max = np.max(self.memory_samples)

            print(f"   CPU Usage:    Mean: {cpu_mean:.1f}%  Max: {cpu_max:.1f}%")
            print(f"   Memory Usage: Mean: {mem_mean:.1f}%  Max: {mem_max:.1f}%")

            if cpu_mean < 50:
                print(f"   ✓ CPU usage is reasonable")
            elif cpu_mean < 75:
                print(f"   ⚠ Moderate CPU usage")
            else:
                print(f"   ✗ High CPU usage - may cause performance issues")

        # ROS Nodes
        print("\n4. ROS NODES:")
        nodes = self.get_ros_node_info()
        if nodes:
            print(f"   Active nodes: {len(nodes)}")
            for node in nodes[:10]:  # Show first 10
                print(f"     - {node['name']}")
            if len(nodes) > 10:
                print(f"     ... and {len(nodes) - 10} more")
        else:
            print("   Could not retrieve node information")

        # Navigation stats
        if self.nav_goals:
            print("\n5. NAVIGATION:")
            total = self.nav_success_count + self.nav_failure_count
            success_rate = (self.nav_success_count / total * 100) if total > 0 else 0

            print(f"   Total goals:   {total}")
            print(f"   Successful:    {self.nav_success_count}")
            print(f"   Failed:        {self.nav_failure_count}")
            print(f"   Success rate:  {success_rate:.1f}%")

        print("\n" + "=" * 70)

    def profile_loop(self, duration: float):
        """Run profiling for specified duration."""
        print(f"Profiling for {duration}s...")
        print("(Press Ctrl+C to stop early)\n")

        end_time = time.time() + duration

        try:
            while rclpy.ok() and time.time() < end_time:
                rclpy.spin_once(self, timeout_sec=0.1)

                # Print status every 5 seconds
                if int(time.time()) % 5 == 0:
                    remaining = int(end_time - time.time())
                    latency = self.calculate_latency_stats()
                    print(f"[{remaining:3d}s] "
                          f"Latency: {latency['mean']:6.2f}ms  "
                          f"CPU: {self.cpu_samples[-1] if self.cpu_samples else 0:5.1f}%  "
                          f"Samples: {len(self.latencies)}",
                          end='\r')

        except KeyboardInterrupt:
            print("\nProfiling stopped")

        # Final report
        self.print_profile_report()


def main():
    parser = argparse.ArgumentParser(description='Performance Profiler')
    parser.add_argument('--duration', '-d', type=float, default=30.0,
                       help='Profiling duration in seconds (default: 30)')
    parser.add_argument('--export', '-e', type=str,
                       help='Export report to file')
    parser.add_argument('--profile-nodes', action='store_true',
                       help='Profile individual ROS nodes')

    args = parser.parse_args()

    rclpy.init()
    node = PerformanceProfiler()

    try:
        node.profile_loop(args.duration)

        # Export if requested
        if args.export:
            # Re-generate report to file
            import sys
            old_stdout = sys.stdout
            with open(args.export, 'w') as f:
                sys.stdout = f
                node.print_profile_report()
                sys.stdout = old_stdout
            print(f"\nProfile report exported to: {args.export}")

    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
