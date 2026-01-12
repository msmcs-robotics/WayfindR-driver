#!/usr/bin/env python3
"""
ROS2 cmd_vel Bridge for WayfindR Robot

Bridges ROS2 Nav2 navigation commands to PI_API FastAPI motor control.
Translates geometry_msgs/Twist to differential drive motor commands
and publishes odometry feedback using dead reckoning.

Author: WayfindR System
Date: 2026-01-11
Version: 1.0.0

Usage:
    # Default parameters
    ros2 run cmd_vel_bridge cmd_vel_bridge

    # Custom parameters
    ros2 run cmd_vel_bridge cmd_vel_bridge --ros-args \\
        -p api_url:=http://192.168.1.100:8000 \\
        -p wheelbase:=0.35 \\
        -p max_linear_velocity:=0.6

    # With parameter file
    ros2 run cmd_vel_bridge cmd_vel_bridge --ros-args \\
        --params-file config/cmd_vel_bridge_params.yaml

Requirements:
    - ROS2 Humble
    - Python 3.10+
    - requests library
    - PI_API running on robot

Topics:
    Subscribed:
        /cmd_vel (geometry_msgs/Twist) - Velocity commands from Nav2
        /emergency_stop (std_msgs/Bool) - Emergency stop trigger

    Published:
        /odom (nav_msgs/Odometry) - Dead reckoning odometry
        /tf (odom -> base_link transform)

References:
    - Design Document: ros2_comprehensive_attempt/findings/CMD_VEL_BRIDGE_DESIGN.md
    - PI_API: PI_API/routers/control.py
    - ROS2 diff_drive_controller: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
"""

import math
import time
from typing import Tuple, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from geometry_msgs.msg import Twist, TransformStamped, Quaternion
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("ERROR: ROS2 not available. Install ROS2 Humble and source setup.bash")
    exit(1)

try:
    import requests
except ImportError:
    print("ERROR: requests library not installed. Run: pip3 install requests")
    exit(1)


class CmdVelBridge(Node):
    """
    Bridge between ROS2 cmd_vel and PI_API motor control.

    Responsibilities:
    1. Subscribe to /cmd_vel and convert Twist to motor commands
    2. Send HTTP requests to PI_API motor control endpoints
    3. Publish odometry using dead reckoning
    4. Broadcast odom->base_link transform
    5. Monitor command timeout (safety watchdog)
    6. Handle emergency stop signals
    """

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        # Declare and load parameters
        self._declare_parameters()
        self._load_parameters()

        # Initialize state variables
        self._init_state()

        # Create QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            qos_profile
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            qos_profile
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers
        odom_period = 1.0 / self.odom_frequency
        self.odom_timer = self.create_timer(odom_period, self.odometry_timer_callback)

        watchdog_period = 0.1  # 10 Hz watchdog
        self.watchdog_timer = self.create_timer(watchdog_period, self.watchdog_callback)

        # Log startup
        self.get_logger().info('=' * 60)
        self.get_logger().info('cmd_vel Bridge Initialized')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'API URL: {self.api_url}')
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.3f} m')
        self.get_logger().info(f'Max Linear Vel: {self.max_linear_vel:.2f} m/s')
        self.get_logger().info(f'Max Angular Vel: {self.max_angular_vel:.2f} rad/s')
        self.get_logger().info(f'cmd_vel Timeout: {self.cmd_vel_timeout:.2f} s')
        self.get_logger().info(f'Odometry Frequency: {self.odom_frequency} Hz')
        self.get_logger().info('=' * 60)

        # Test API connection
        self._test_api_connection()

    def _declare_parameters(self):
        """Declare ROS2 parameters with default values."""
        # Network parameters
        self.declare_parameter('api_url', 'http://localhost:8000')
        self.declare_parameter('api_timeout', 0.1)

        # Robot physical parameters
        self.declare_parameter('wheelbase', 0.30)  # meters
        self.declare_parameter('wheel_radius', 0.065)  # meters

        # Velocity limits
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s

        # Control parameters
        self.declare_parameter('cmd_vel_timeout', 0.5)  # seconds

        # Odometry parameters
        self.declare_parameter('odom_frequency', 50)  # Hz
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Covariance values (dead reckoning uncertainty)
        self.declare_parameter('pose_covariance_x', 0.1)
        self.declare_parameter('pose_covariance_y', 0.1)
        self.declare_parameter('pose_covariance_yaw', 0.5)
        self.declare_parameter('twist_covariance_vx', 0.05)
        self.declare_parameter('twist_covariance_vyaw', 0.1)

    def _load_parameters(self):
        """Load parameters from parameter server."""
        self.api_url = self.get_parameter('api_url').value
        self.api_timeout = self.get_parameter('api_timeout').value

        self.wheelbase = self.get_parameter('wheelbase').value
        self.wheel_radius = self.get_parameter('wheel_radius').value

        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value

        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        self.odom_frequency = self.get_parameter('odom_frequency').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.pose_cov_x = self.get_parameter('pose_covariance_x').value
        self.pose_cov_y = self.get_parameter('pose_covariance_y').value
        self.pose_cov_yaw = self.get_parameter('pose_covariance_yaw').value
        self.twist_cov_vx = self.get_parameter('twist_covariance_vx').value
        self.twist_cov_vyaw = self.get_parameter('twist_covariance_vyaw').value

    def _init_state(self):
        """Initialize state variables."""
        # Odometry state (dead reckoning)
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians

        # Current velocities (last commanded)
        self.current_v_x = 0.0  # m/s
        self.current_omega_z = 0.0  # rad/s

        # Timing
        self.last_cmd_time = time.time()
        self.last_odom_time = time.time()

        # Connection monitoring
        self.connection_failures = 0
        self.max_connection_failures = 5
        self.api_connected = False

    def _test_api_connection(self):
        """Test connection to PI_API."""
        try:
            response = requests.get(
                f'{self.api_url}/api/health',
                timeout=1.0
            )

            if response.status_code == 200:
                self.api_connected = True
                self.get_logger().info('API connection successful')
            else:
                self.get_logger().warn(f'API connection failed: {response.status_code}')

        except Exception as e:
            self.get_logger().error(f'Cannot connect to API: {e}')
            self.get_logger().warn('Bridge will continue, but motor commands will fail')

    # ========================================================================
    # cmd_vel Callback
    # ========================================================================

    def cmd_vel_callback(self, msg: Twist):
        """
        Handle incoming cmd_vel messages from Nav2.

        Args:
            msg: Twist message with linear.x and angular.z
        """
        self.last_cmd_time = time.time()

        # Extract velocities
        v_x = msg.linear.x  # m/s (forward/backward)
        omega_z = msg.angular.z  # rad/s (rotation)

        # Limit velocities to safe values
        v_x, omega_z = self._limit_velocity(v_x, omega_z)

        # Convert to motor commands
        throttle, steering = self._twist_to_motor_commands(v_x, omega_z)

        # Send to robot
        success = self._send_motor_command(throttle, steering)

        if success:
            # Update state for odometry
            self.current_v_x = v_x
            self.current_omega_z = omega_z

            # Log at debug level (reduce spam)
            if abs(v_x) > 0.01 or abs(omega_z) > 0.01:
                self.get_logger().debug(
                    f'cmd_vel: v={v_x:.2f} ω={omega_z:.2f} -> '
                    f'throttle={throttle:.2f} steering={steering:.2f}'
                )
        else:
            # Command failed - keep previous velocities for odometry
            pass

    # ========================================================================
    # Emergency Stop
    # ========================================================================

    def estop_callback(self, msg: Bool):
        """
        Handle emergency stop messages.

        Args:
            msg: Bool message (True = emergency stop)
        """
        if msg.data:
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
            self._emergency_stop()

    def _emergency_stop(self):
        """Execute emergency stop."""
        try:
            response = requests.post(
                f'{self.api_url}/api/emergency_stop',
                timeout=0.5
            )

            if response.status_code == 200:
                self.get_logger().info('Emergency stop successful')
            else:
                self.get_logger().error(f'Emergency stop failed: {response.status_code}')

        except Exception as e:
            self.get_logger().error(f'Emergency stop error: {e}')

        # Reset state
        self.current_v_x = 0.0
        self.current_omega_z = 0.0

    # ========================================================================
    # Watchdog Timer
    # ========================================================================

    def watchdog_callback(self):
        """
        Safety watchdog - stops robot if no cmd_vel received within timeout.
        Prevents runaway if ROS2 crashes or network fails.
        """
        time_since_cmd = time.time() - self.last_cmd_time

        if time_since_cmd > self.cmd_vel_timeout:
            # Command timeout - stop robot
            if self.current_v_x != 0.0 or self.current_omega_z != 0.0:
                self.get_logger().warn(
                    f'cmd_vel timeout ({time_since_cmd:.2f}s) - stopping robot'
                )
                self._send_stop_command()
                self.current_v_x = 0.0
                self.current_omega_z = 0.0

    # ========================================================================
    # Motor Command Conversion
    # ========================================================================

    def _limit_velocity(self, v_x: float, omega_z: float) -> Tuple[float, float]:
        """
        Limit velocities to robot capabilities.

        Args:
            v_x: Linear velocity (m/s)
            omega_z: Angular velocity (rad/s)

        Returns:
            (limited_v_x, limited_omega_z)
        """
        v_x = max(-self.max_linear_vel, min(self.max_linear_vel, v_x))
        omega_z = max(-self.max_angular_vel, min(self.max_angular_vel, omega_z))
        return v_x, omega_z

    def _twist_to_motor_commands(self, v_x: float, omega_z: float) -> Tuple[float, float]:
        """
        Convert Twist message to motor commands using differential drive kinematics.

        Based on inverse kinematics:
            v_left = v_x - (omega_z * wheelbase / 2)
            v_right = v_x + (omega_z * wheelbase / 2)

        Then normalize to PI_API format (throttle, steering) in [-1.0, 1.0]

        Args:
            v_x: Linear velocity (m/s)
            omega_z: Angular velocity (rad/s)

        Returns:
            (throttle, steering) in range [-1.0, 1.0]
        """
        # Calculate wheel velocities (m/s)
        v_left = v_x - (omega_z * self.wheelbase / 2.0)
        v_right = v_x + (omega_z * self.wheelbase / 2.0)

        # Normalize to [-1.0, 1.0] range
        # throttle = average of wheels (forward/backward)
        # steering = difference between wheels (left/right)
        throttle = (v_left + v_right) / (2.0 * self.max_linear_vel)

        # Calculate steering
        if abs(v_x) > 0.01:
            # Normal steering (arc turn)
            # Steering is based on wheel speed difference
            steering = (v_right - v_left) / (2.0 * self.max_linear_vel)
        else:
            # Pivot turn (rotation in place)
            # Use angular velocity directly
            steering = omega_z / self.max_angular_vel

        # Clamp to valid range
        throttle = max(-1.0, min(1.0, throttle))
        steering = max(-1.0, min(1.0, steering))

        return throttle, steering

    # ========================================================================
    # HTTP Communication
    # ========================================================================

    def _send_motor_command(self, throttle: float, steering: float) -> bool:
        """
        Send motor command to PI_API via HTTP.

        Args:
            throttle: Throttle value [-1.0, 1.0]
            steering: Steering value [-1.0, 1.0]

        Returns:
            True if successful, False otherwise
        """
        try:
            response = requests.post(
                f'{self.api_url}/api/control/move',
                json={
                    'throttle': float(throttle),
                    'steering': float(steering),
                    'duration': None  # Continuous until next command
                },
                timeout=self.api_timeout
            )

            if response.status_code == 200:
                self.connection_failures = 0
                if not self.api_connected:
                    self.api_connected = True
                    self.get_logger().info('API connection restored')
                return True
            else:
                self.get_logger().warn(f'Motor command failed: {response.status_code}')
                self._handle_connection_error()
                return False

        except requests.exceptions.Timeout:
            self.get_logger().error('Motor command timeout')
            self._handle_connection_error()
            return False

        except requests.exceptions.ConnectionError:
            self.get_logger().error('Cannot connect to API')
            self._handle_connection_error()
            return False

        except Exception as e:
            self.get_logger().error(f'Motor command error: {e}')
            self._handle_connection_error()
            return False

    def _send_stop_command(self):
        """Send stop command to robot."""
        try:
            response = requests.post(
                f'{self.api_url}/api/control/stop',
                json={'mode': 'normal'},
                timeout=self.api_timeout
            )

            if response.status_code == 200:
                self.get_logger().debug('Stop command sent')

        except Exception as e:
            self.get_logger().error(f'Stop command failed: {e}')

    def _handle_connection_error(self):
        """Handle connection errors to PI_API."""
        self.connection_failures += 1

        if self.connection_failures >= self.max_connection_failures:
            if self.api_connected:
                self.api_connected = False
                self.get_logger().error(
                    f'Lost connection to PI_API after {self.connection_failures} failures'
                )

    # ========================================================================
    # Odometry Publishing
    # ========================================================================

    def odometry_timer_callback(self):
        """
        Timer callback to publish odometry at fixed rate.
        Called at odom_frequency (default 50 Hz).
        """
        current_time = time.time()
        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        # Update pose using dead reckoning
        self._update_odometry(dt)

        # Publish odometry message
        self._publish_odometry()

        # Publish TF transform
        if self.publish_tf:
            self._publish_transform()

    def _update_odometry(self, dt: float):
        """
        Update robot pose using dead reckoning (integration).

        This is open-loop estimation without encoder feedback.
        Assumes commanded velocity equals actual velocity.

        Args:
            dt: Time delta since last update (seconds)
        """
        # Get current velocities (last commanded)
        v_x = self.current_v_x  # m/s
        omega_z = self.current_omega_z  # rad/s

        # Update heading first (yaw angle)
        self.theta += omega_z * dt

        # Wrap angle to [-π, π]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Update position in odom frame
        # Robot moves in its own frame, then transform to odom frame
        delta_x = v_x * math.cos(self.theta) * dt
        delta_y = v_x * math.sin(self.theta) * dt

        self.x += delta_x
        self.y += delta_y

    def _publish_odometry(self):
        """Publish odometry message to /odom topic."""
        odom_msg = Odometry()

        # Header
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (convert yaw to quaternion)
        quat = self._quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]

        # Velocity (in child_frame_id = base_link)
        odom_msg.twist.twist.linear.x = self.current_v_x
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = self.current_omega_z

        # Covariance (dead reckoning is inaccurate)
        # Pose covariance (6x6 matrix, row-major)
        odom_msg.pose.covariance = [
            self.pose_cov_x, 0.0, 0.0, 0.0, 0.0, 0.0,  # x
            0.0, self.pose_cov_y, 0.0, 0.0, 0.0, 0.0,  # y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,              # z (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,              # roll (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,              # pitch (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, self.pose_cov_yaw # yaw
        ]

        # Twist covariance (6x6 matrix, row-major)
        odom_msg.twist.covariance = [
            self.twist_cov_vx, 0.0, 0.0, 0.0, 0.0, 0.0,  # vx
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                # vy (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                # vz (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                # vroll (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,                # vpitch (not used)
            0.0, 0.0, 0.0, 0.0, 0.0, self.twist_cov_vyaw # vyaw
        ]

        # Publish
        self.odom_pub.publish(odom_msg)

    def _publish_transform(self):
        """Publish TF transform from odom to base_link."""
        transform = TransformStamped()

        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame

        # Translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        # Rotation (convert yaw to quaternion)
        quat = self._quaternion_from_euler(0.0, 0.0, self.theta)
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]

        # Broadcast
        self.tf_broadcaster.sendTransform(transform)

    # ========================================================================
    # Utility Functions
    # ========================================================================

    def _quaternion_from_euler(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert Euler angles to quaternion.

        For 2D robots: roll=0, pitch=0, yaw=θ

        Args:
            roll: Rotation around X-axis (radians)
            pitch: Rotation around Y-axis (radians)
            yaw: Rotation around Z-axis (radians)

        Returns:
            (qx, qy, qz, qw)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return (qx, qy, qz, qw)

    def destroy_node(self):
        """Cleanup on node shutdown - stop robot safely."""
        self.get_logger().info('Shutting down cmd_vel bridge - stopping robot')
        self._send_stop_command()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    if not ROS2_AVAILABLE:
        print("ERROR: ROS2 not available")
        return

    rclpy.init(args=args)

    try:
        node = CmdVelBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"ERROR: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
