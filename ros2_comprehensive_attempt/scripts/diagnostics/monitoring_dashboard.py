#!/usr/bin/env python3
"""
WayfindR Monitoring Dashboard

Real-time terminal-based monitoring dashboard for WayfindR navigation system.
Displays health checks, performance metrics, and alerts.

Usage:
    python3 monitoring_dashboard.py

Requirements:
    pip3 install rich psutil
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus

import time
import math
from collections import deque, defaultdict
from datetime import datetime

try:
    from rich.console import Console
    from rich.layout import Layout
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from rich.live import Live
    from rich.progress import Progress, SpinnerColumn, TextColumn
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False
    print("Warning: 'rich' library not available. Install with: pip3 install rich")


class MonitoringDashboard(Node):
    """Real-time monitoring dashboard."""

    def __init__(self):
        super().__init__('monitoring_dashboard')

        # Data storage
        self.diagnostics = {}
        self.topics_status = {}
        self.robot_pose = None
        self.robot_velocity = None
        self.scan_data = None
        self.alerts = deque(maxlen=10)

        # Statistics
        self.start_time = time.time()
        self.total_distance = 0.0
        self.last_pose = None

        # Create QoS profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._diagnostics_callback,
            10
        )

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self._pose_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10
        )

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

        self.get_logger().info("Monitoring dashboard started")

    def _diagnostics_callback(self, msg: DiagnosticArray):
        """Process diagnostic messages."""
        for status in msg.status:
            self.diagnostics[status.name] = {
                'level': status.level,
                'message': status.message,
                'values': {kv.key: kv.value for kv in status.values}
            }

            # Generate alerts for warnings and errors
            if status.level >= DiagnosticStatus.WARN:
                alert_msg = f"{status.name}: {status.message}"
                if alert_msg not in [a[1] for a in self.alerts]:
                    self.alerts.append((time.time(), alert_msg, status.level))

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """Process robot pose."""
        self.robot_pose = msg

        # Calculate distance traveled
        if self.last_pose:
            dx = msg.pose.pose.position.x - self.last_pose.pose.pose.position.x
            dy = msg.pose.pose.position.y - self.last_pose.pose.pose.position.y
            dist = math.sqrt(dx**2 + dy**2)
            if dist < 1.0:  # Sanity check
                self.total_distance += dist

        self.last_pose = msg

    def _odom_callback(self, msg: Odometry):
        """Process odometry."""
        pass

    def _scan_callback(self, msg: LaserScan):
        """Process laser scan."""
        self.scan_data = msg

    def _cmd_vel_callback(self, msg: Twist):
        """Process velocity commands."""
        self.robot_velocity = msg

    def get_status_symbol(self, level):
        """Get status symbol for diagnostic level."""
        if level == DiagnosticStatus.OK:
            return "✓", "green"
        elif level == DiagnosticStatus.WARN:
            return "⚠", "yellow"
        elif level == DiagnosticStatus.ERROR:
            return "✗", "red"
        else:
            return "?", "gray"

    def create_header_panel(self):
        """Create header panel."""
        uptime = time.time() - self.start_time
        hours = int(uptime // 3600)
        minutes = int((uptime % 3600) // 60)
        seconds = int(uptime % 60)

        header_text = Text()
        header_text.append("WayfindR Navigation System", style="bold cyan")
        header_text.append(f" | Uptime: {hours:02d}:{minutes:02d}:{seconds:02d}", style="white")
        header_text.append(f" | {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}", style="gray")

        return Panel(header_text, style="cyan")

    def create_diagnostics_table(self):
        """Create diagnostics status table."""
        table = Table(title="System Diagnostics", show_header=True, header_style="bold")
        table.add_column("Component", style="cyan", width=30)
        table.add_column("Status", width=10)
        table.add_column("Details", style="white")

        # Group diagnostics by category
        categories = {
            'Topics': [],
            'TF': [],
            'Localization': [],
            'System': []
        }

        for name, diag in self.diagnostics.items():
            if name.startswith('Topic:'):
                categories['Topics'].append((name, diag))
            elif name.startswith('TF:'):
                categories['TF'].append((name, diag))
            elif 'Localization' in name:
                categories['Localization'].append((name, diag))
            elif 'System' in name:
                categories['System'].append((name, diag))

        # Add rows by category
        for category, items in categories.items():
            if items:
                for name, diag in items:
                    symbol, color = self.get_status_symbol(diag['level'])
                    status_text = Text(f"{symbol} {diag['message']}", style=color)

                    # Format details
                    details = []
                    for key, value in diag['values'].items():
                        if key in ['expected_hz', 'actual_hz']:
                            details.append(f"{key}={value}")
                        elif key == 'position_uncertainty':
                            details.append(f"pos_unc={value}m")
                        elif key in ['cpu_percent', 'memory_percent']:
                            details.append(f"{key}={value}%")

                    table.add_row(name.replace('Topic: ', '').replace('TF: ', ''),
                                 status_text,
                                 ", ".join(details) if details else "")

        if not self.diagnostics:
            table.add_row("No diagnostics data", Text("?", style="gray"), "Waiting...")

        return Panel(table, title="Diagnostics", border_style="blue")

    def create_robot_status_panel(self):
        """Create robot status panel."""
        table = Table(show_header=False, box=None)
        table.add_column("Metric", style="cyan", width=20)
        table.add_column("Value", style="white")

        if self.robot_pose:
            x = self.robot_pose.pose.pose.position.x
            y = self.robot_pose.pose.pose.position.y

            # Extract yaw from quaternion
            qz = self.robot_pose.pose.pose.orientation.z
            qw = self.robot_pose.pose.pose.orientation.w
            yaw = math.degrees(2 * math.atan2(qz, qw))

            table.add_row("Position", f"({x:.2f}, {y:.2f})")
            table.add_row("Orientation", f"{yaw:.1f}°")
            table.add_row("Distance Traveled", f"{self.total_distance:.2f}m")
        else:
            table.add_row("Position", "Unknown")

        if self.robot_velocity:
            linear = self.robot_velocity.linear.x
            angular = math.degrees(self.robot_velocity.angular.z)
            table.add_row("Linear Velocity", f"{linear:.2f} m/s")
            table.add_row("Angular Velocity", f"{angular:.1f} °/s")

        if self.scan_data:
            # Calculate min obstacle distance
            valid_ranges = [r for r in self.scan_data.ranges
                           if r >= self.scan_data.range_min and r <= self.scan_data.range_max]
            if valid_ranges:
                min_dist = min(valid_ranges)
                table.add_row("Nearest Obstacle", f"{min_dist:.2f}m")

        return Panel(table, title="Robot Status", border_style="green")

    def create_alerts_panel(self):
        """Create alerts panel."""
        if not self.alerts:
            return Panel("No alerts", title="Alerts", border_style="green")

        table = Table(show_header=False, box=None, expand=True)
        table.add_column("Time", style="gray", width=8)
        table.add_column("Alert", style="white")

        for timestamp, msg, level in reversed(list(self.alerts)):
            age = time.time() - timestamp
            symbol, color = self.get_status_symbol(level)
            time_str = f"{int(age)}s ago"

            alert_text = Text()
            alert_text.append(symbol, style=color)
            alert_text.append(f" {msg}", style="white")

            table.add_row(time_str, alert_text)

        return Panel(table, title="Recent Alerts", border_style="yellow")

    def create_layout(self):
        """Create dashboard layout."""
        layout = Layout()

        layout.split_column(
            Layout(name="header", size=3),
            Layout(name="body"),
            Layout(name="footer", size=12)
        )

        layout["body"].split_row(
            Layout(name="left", ratio=2),
            Layout(name="right", ratio=1)
        )

        layout["header"].update(self.create_header_panel())
        layout["left"].update(self.create_diagnostics_table())
        layout["right"].update(self.create_robot_status_panel())
        layout["footer"].update(self.create_alerts_panel())

        return layout


def main(args=None):
    if not RICH_AVAILABLE:
        print("Error: This dashboard requires the 'rich' library.")
        print("Install it with: pip3 install rich")
        return

    rclpy.init(args=args)
    node = MonitoringDashboard()

    console = Console()

    try:
        with Live(node.create_layout(), refresh_per_second=2, console=console) as live:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                live.update(node.create_layout())

    except KeyboardInterrupt:
        console.print("\n[yellow]Dashboard stopped[/yellow]")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
