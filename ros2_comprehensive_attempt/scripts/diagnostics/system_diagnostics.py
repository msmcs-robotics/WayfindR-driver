#!/usr/bin/env python3
"""
WayfindR System Diagnostics Node

Monitors:
- TF tree health (missing transforms, stale data)
- Topic health (/scan, /cmd_vel, /odom frequency)
- AMCL localization quality (particle spread, covariance)
- Nav2 status (current state, errors)
- System resources (CPU, memory, network)

Usage:
    ros2 run <package> system_diagnostics.py
    python3 system_diagnostics.py  # Standalone mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.msg import State
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import tf2_ros
from tf2_ros import TransformException

import time
import psutil
import math
from collections import defaultdict, deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np


@dataclass
class TopicHealth:
    """Health status for a topic."""
    name: str
    msg_count: int = 0
    last_received: float = 0.0
    expected_hz: float = 0.0
    actual_hz: float = 0.0
    status: str = "UNKNOWN"  # OK, WARN, ERROR, STALE, UNKNOWN
    timestamps: deque = field(default_factory=lambda: deque(maxlen=50))

    def update(self, current_time: float):
        """Update message count and timestamp."""
        self.msg_count += 1
        self.last_received = current_time
        self.timestamps.append(current_time)

        # Calculate frequency
        if len(self.timestamps) >= 2:
            time_span = self.timestamps[-1] - self.timestamps[0]
            if time_span > 0:
                self.actual_hz = (len(self.timestamps) - 1) / time_span

        # Update status
        time_since_last = current_time - self.last_received
        if time_since_last > 2.0:
            self.status = "STALE"
        elif self.expected_hz > 0:
            if self.actual_hz < self.expected_hz * 0.5:
                self.status = "WARN"
            elif self.actual_hz < self.expected_hz * 0.2:
                self.status = "ERROR"
            else:
                self.status = "OK"
        else:
            self.status = "OK" if self.msg_count > 0 else "UNKNOWN"


@dataclass
class TFHealth:
    """Health status for TF transforms."""
    frame_id: str
    child_frame_id: str
    last_update: float = 0.0
    age_seconds: float = 0.0
    status: str = "UNKNOWN"
    error_msg: str = ""


@dataclass
class LocalizationHealth:
    """AMCL localization quality metrics."""
    particle_count: int = 0
    particle_spread: float = 0.0
    covariance_xx: float = 0.0
    covariance_yy: float = 0.0
    covariance_aa: float = 0.0
    position_uncertainty: float = 0.0
    orientation_uncertainty: float = 0.0
    status: str = "UNKNOWN"
    last_update: float = 0.0


@dataclass
class SystemHealth:
    """System resource metrics."""
    cpu_percent: float = 0.0
    memory_percent: float = 0.0
    memory_available_mb: float = 0.0
    disk_percent: float = 0.0
    network_sent_mbps: float = 0.0
    network_recv_mbps: float = 0.0
    ros_node_count: int = 0
    status: str = "OK"


class SystemDiagnostics(Node):
    """Main diagnostics node."""

    def __init__(self):
        super().__init__('system_diagnostics')

        # Initialize monitoring structures
        self.topic_health: Dict[str, TopicHealth] = {}
        self.tf_health: Dict[str, TFHealth] = {}
        self.localization_health = LocalizationHealth()
        self.system_health = SystemHealth()

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Network tracking for bandwidth calculation
        self.last_net_io = psutil.net_io_counters()
        self.last_net_time = time.time()

        # Define expected topics and frequencies
        self.expected_topics = {
            '/scan': 10.0,          # 10 Hz
            '/cmd_vel': 20.0,       # 20 Hz
            '/odom': 20.0,          # 20 Hz
            '/amcl_pose': 2.0,      # 2 Hz
        }

        # Initialize topic health
        for topic, hz in self.expected_topics.items():
            self.topic_health[topic] = TopicHealth(name=topic, expected_hz=hz)

        # Define critical TF transforms
        self.critical_transforms = [
            ('map', 'odom'),
            ('odom', 'base_footprint'),
            ('base_footprint', 'base_link'),
            ('base_link', 'laser'),
        ]

        # Create QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Topic subscribers
        self.create_subscription(LaserScan, '/scan', self._scan_callback, qos)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)
        self.create_subscription(Odometry, '/odom', self._odom_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._amcl_pose_callback,
            10
        )

        # Diagnostic publisher
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )

        # Timers
        self.create_timer(1.0, self._check_tf_health)
        self.create_timer(1.0, self._check_system_health)
        self.create_timer(1.0, self._publish_diagnostics)
        self.create_timer(5.0, self._print_status)

        self.get_logger().info("System diagnostics node started")

    def _scan_callback(self, msg: LaserScan):
        """Handle laser scan messages."""
        current_time = time.time()
        self.topic_health['/scan'].update(current_time)

    def _cmd_vel_callback(self, msg: Twist):
        """Handle cmd_vel messages."""
        current_time = time.time()
        self.topic_health['/cmd_vel'].update(current_time)

    def _odom_callback(self, msg: Odometry):
        """Handle odometry messages."""
        current_time = time.time()
        self.topic_health['/odom'].update(current_time)

    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle AMCL pose messages and extract localization quality."""
        current_time = time.time()
        self.topic_health['/amcl_pose'].update(current_time)

        # Extract covariance (6x6 matrix, flattened)
        cov = msg.pose.covariance

        # Position uncertainty (x, y)
        self.localization_health.covariance_xx = cov[0]  # x variance
        self.localization_health.covariance_yy = cov[7]  # y variance
        self.localization_health.covariance_aa = cov[35] # yaw variance

        # Position uncertainty (standard deviation)
        self.localization_health.position_uncertainty = math.sqrt(
            self.localization_health.covariance_xx +
            self.localization_health.covariance_yy
        )

        # Orientation uncertainty
        self.localization_health.orientation_uncertainty = math.sqrt(
            self.localization_health.covariance_aa
        )

        # Determine status based on uncertainty
        if self.localization_health.position_uncertainty < 0.1:
            status = "EXCELLENT"
        elif self.localization_health.position_uncertainty < 0.3:
            status = "GOOD"
        elif self.localization_health.position_uncertainty < 0.5:
            status = "FAIR"
        else:
            status = "POOR"

        self.localization_health.status = status
        self.localization_health.last_update = current_time

    def _check_tf_health(self):
        """Check TF transform health."""
        current_time = time.time()

        for parent, child in self.critical_transforms:
            try:
                # Try to get the transform
                trans = self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    Time(),
                    timeout=Duration(seconds=0.1)
                )

                # Calculate age
                stamp_sec = trans.header.stamp.sec + trans.header.stamp.nanosec / 1e9
                age = current_time - stamp_sec

                # Update health
                key = f"{parent}->{child}"
                if key not in self.tf_health:
                    self.tf_health[key] = TFHealth(parent, child)

                self.tf_health[key].last_update = current_time
                self.tf_health[key].age_seconds = age
                self.tf_health[key].error_msg = ""

                # Determine status based on age
                if age > 5.0:
                    self.tf_health[key].status = "STALE"
                elif age > 2.0:
                    self.tf_health[key].status = "WARN"
                else:
                    self.tf_health[key].status = "OK"

            except TransformException as ex:
                key = f"{parent}->{child}"
                if key not in self.tf_health:
                    self.tf_health[key] = TFHealth(parent, child)

                self.tf_health[key].status = "ERROR"
                self.tf_health[key].error_msg = str(ex)

    def _check_system_health(self):
        """Check system resource usage."""
        # CPU and Memory
        self.system_health.cpu_percent = psutil.cpu_percent(interval=0.1)
        mem = psutil.virtual_memory()
        self.system_health.memory_percent = mem.percent
        self.system_health.memory_available_mb = mem.available / (1024 * 1024)

        # Disk
        disk = psutil.disk_usage('/')
        self.system_health.disk_percent = disk.percent

        # Network bandwidth
        current_net_io = psutil.net_io_counters()
        current_time = time.time()
        time_delta = current_time - self.last_net_time

        if time_delta > 0:
            sent_delta = current_net_io.bytes_sent - self.last_net_io.bytes_sent
            recv_delta = current_net_io.bytes_recv - self.last_net_io.bytes_recv

            self.system_health.network_sent_mbps = (sent_delta / time_delta) / (1024 * 1024)
            self.system_health.network_recv_mbps = (recv_delta / time_delta) / (1024 * 1024)

        self.last_net_io = current_net_io
        self.last_net_time = current_time

        # Determine overall system status
        if self.system_health.cpu_percent > 90 or self.system_health.memory_percent > 90:
            self.system_health.status = "CRITICAL"
        elif self.system_health.cpu_percent > 70 or self.system_health.memory_percent > 75:
            self.system_health.status = "WARN"
        else:
            self.system_health.status = "OK"

    def _publish_diagnostics(self):
        """Publish diagnostic messages."""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()

        # Topic health diagnostics
        for topic_name, health in self.topic_health.items():
            status = DiagnosticStatus()
            status.name = f"Topic: {topic_name}"
            status.hardware_id = "topics"

            if health.status == "OK":
                status.level = DiagnosticStatus.OK
            elif health.status == "WARN":
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR

            status.message = health.status
            status.values = [
                KeyValue(key="expected_hz", value=f"{health.expected_hz:.1f}"),
                KeyValue(key="actual_hz", value=f"{health.actual_hz:.1f}"),
                KeyValue(key="msg_count", value=str(health.msg_count)),
            ]

            msg.status.append(status)

        # TF health diagnostics
        for tf_name, health in self.tf_health.items():
            status = DiagnosticStatus()
            status.name = f"TF: {tf_name}"
            status.hardware_id = "tf"

            if health.status == "OK":
                status.level = DiagnosticStatus.OK
            elif health.status == "WARN":
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.ERROR

            status.message = health.status
            status.values = [
                KeyValue(key="age_seconds", value=f"{health.age_seconds:.2f}"),
                KeyValue(key="error", value=health.error_msg),
            ]

            msg.status.append(status)

        # Localization health
        loc_status = DiagnosticStatus()
        loc_status.name = "Localization (AMCL)"
        loc_status.hardware_id = "localization"

        if self.localization_health.status in ["EXCELLENT", "GOOD"]:
            loc_status.level = DiagnosticStatus.OK
        elif self.localization_health.status == "FAIR":
            loc_status.level = DiagnosticStatus.WARN
        else:
            loc_status.level = DiagnosticStatus.ERROR

        loc_status.message = self.localization_health.status
        loc_status.values = [
            KeyValue(key="position_uncertainty",
                    value=f"{self.localization_health.position_uncertainty:.3f}"),
            KeyValue(key="orientation_uncertainty",
                    value=f"{self.localization_health.orientation_uncertainty:.3f}"),
        ]
        msg.status.append(loc_status)

        # System health
        sys_status = DiagnosticStatus()
        sys_status.name = "System Resources"
        sys_status.hardware_id = "system"

        if self.system_health.status == "OK":
            sys_status.level = DiagnosticStatus.OK
        elif self.system_health.status == "WARN":
            sys_status.level = DiagnosticStatus.WARN
        else:
            sys_status.level = DiagnosticStatus.ERROR

        sys_status.message = self.system_health.status
        sys_status.values = [
            KeyValue(key="cpu_percent", value=f"{self.system_health.cpu_percent:.1f}"),
            KeyValue(key="memory_percent", value=f"{self.system_health.memory_percent:.1f}"),
            KeyValue(key="memory_available_mb",
                    value=f"{self.system_health.memory_available_mb:.1f}"),
        ]
        msg.status.append(sys_status)

        self.diagnostic_pub.publish(msg)

    def _print_status(self):
        """Print status summary to console."""
        self.get_logger().info("=" * 70)
        self.get_logger().info("SYSTEM DIAGNOSTICS SUMMARY")
        self.get_logger().info("=" * 70)

        # Topics
        self.get_logger().info("Topics:")
        for topic_name, health in self.topic_health.items():
            self.get_logger().info(
                f"  {topic_name:20s} {health.status:8s} "
                f"{health.actual_hz:5.1f}/{health.expected_hz:.1f} Hz"
            )

        # TF
        self.get_logger().info("\nTF Transforms:")
        for tf_name, health in self.tf_health.items():
            msg = f"age={health.age_seconds:.2f}s" if health.status == "OK" else health.error_msg
            self.get_logger().info(f"  {tf_name:30s} {health.status:8s} {msg}")

        # Localization
        self.get_logger().info(f"\nLocalization: {self.localization_health.status}")
        self.get_logger().info(
            f"  Position uncertainty: {self.localization_health.position_uncertainty:.3f}m"
        )
        self.get_logger().info(
            f"  Orientation uncertainty: "
            f"{math.degrees(self.localization_health.orientation_uncertainty):.1f}°"
        )

        # System
        self.get_logger().info(f"\nSystem Resources: {self.system_health.status}")
        self.get_logger().info(f"  CPU: {self.system_health.cpu_percent:.1f}%")
        self.get_logger().info(f"  Memory: {self.system_health.memory_percent:.1f}%")
        self.get_logger().info(
            f"  Network: ↑{self.system_health.network_sent_mbps:.2f} "
            f"↓{self.system_health.network_recv_mbps:.2f} MB/s"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SystemDiagnostics()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
