#!/usr/bin/env python3
"""
Wandering Demo 1 - LiDAR Obstacle Avoidance + Motor Control

Uses LiDAR-based obstacle detection to drive motors for autonomous wandering.
Supports multiple behaviors (max clearance, wall following, random wander).
Can run in simulation mode (no motors) or with real hardware.

This is the LiDAR-only demo. See wandering_demo_2.py for camera face tracking.

Usage:
    # Simulation mode (no motors, prints commands)
    python3 wandering_demo_1.py --simulate

    # Real mode with motors
    python3 wandering_demo_1.py

    # Select behavior
    python3 wandering_demo_1.py --behavior max_clearance
    python3 wandering_demo_1.py --behavior wall_follower_right
    python3 wandering_demo_1.py --behavior random_wander

    # Adjust speeds
    python3 wandering_demo_1.py --forward-speed 0.3 --turn-speed 0.4
"""

import argparse
import logging
import sys
import time
from datetime import datetime
from pathlib import Path

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RobotAdapter:
    """
    Adapts pathfinder's RobotInterface to locomotion's DifferentialDrive.

    Converts:
    - Float speeds (-1.0 to 1.0) → Integer speeds (-100 to 100)
    - set_motors(left, right) → robot.drive(left, right)
    """

    def __init__(self, robot=None, simulate: bool = False, log_commands: bool = True):
        """
        Initialize adapter.

        Args:
            robot: DifferentialDrive instance (or None for simulation)
            simulate: If True, don't send commands to motors
            log_commands: If True, log all motor commands
        """
        self.robot = robot
        self.simulate = simulate
        self.log_commands = log_commands
        self._last_command_time = 0
        self._command_count = 0
        self._command_log = []

    def set_motors(self, left: float, right: float) -> None:
        """
        Set motor speeds (pathfinder interface).

        Args:
            left: Left motor speed (-1.0 to 1.0)
            right: Right motor speed (-1.0 to 1.0)
        """
        # Convert to integer percentage
        left_speed = int(left * 100)
        right_speed = int(right * 100)

        # Clamp to valid range
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        self._command_count += 1
        self._last_command_time = time.time()

        if self.log_commands:
            cmd = f"L:{left_speed:+4d} R:{right_speed:+4d}"
            self._command_log.append((time.time(), left_speed, right_speed))
            # Keep log manageable
            if len(self._command_log) > 1000:
                self._command_log = self._command_log[-500:]

        if self.simulate:
            # Print command visualization
            self._print_command(left_speed, right_speed)
        elif self.robot:
            self.robot.drive(left_speed, right_speed)

    def stop(self) -> None:
        """Emergency stop."""
        if self.simulate:
            print("🛑 STOP")
        elif self.robot:
            self.robot.stop()

        self._command_log.append((time.time(), 0, 0))

    def _print_command(self, left: int, right: int):
        """Print visual representation of motor command."""
        # Direction indicator
        if left > 0 and right > 0:
            direction = "↑ FWD"
        elif left < 0 and right < 0:
            direction = "↓ REV"
        elif left < 0 and right > 0:
            direction = "↰ LEFT"
        elif left > 0 and right < 0:
            direction = "↱ RIGHT"
        else:
            direction = "• STOP"

        # Speed bars
        left_bar = "█" * (abs(left) // 10) if left != 0 else ""
        right_bar = "█" * (abs(right) // 10) if right != 0 else ""

        print(f"\r{direction:8s} L:{left:+4d}|{left_bar:10s}| R:{right:+4d}|{right_bar:10s}|", end="")

    def get_stats(self) -> dict:
        """Get command statistics."""
        return {
            "command_count": self._command_count,
            "last_command_time": self._last_command_time,
            "log_size": len(self._command_log),
        }

    def cleanup(self):
        """Cleanup resources."""
        if self.robot:
            self.robot.cleanup()


class SimulatedObstacleDetector:
    """Simulated detector for testing without LiDAR."""

    def __init__(self):
        self.sectors = {}

    def process_scan(self, scan):
        """Process scan and return simulated detection."""
        from pathfinder.obstacle_detector import DetectionResult, SectorReading, SafetyLevel

        # Create simulated sectors with random-ish data
        import random

        sectors = {
            "front": SectorReading(
                name="front",
                angle_range=(337.5, 22.5),
                min_distance=random.uniform(0.5, 3.0),
                safety_level=SafetyLevel.CLEAR,
            ),
            "right": SectorReading(
                name="right",
                angle_range=(22.5, 112.5),
                min_distance=random.uniform(0.3, 2.0),
                safety_level=SafetyLevel.CLEAR,
            ),
            "back": SectorReading(
                name="back",
                angle_range=(112.5, 247.5),
                min_distance=random.uniform(1.0, 5.0),
                safety_level=SafetyLevel.CLEAR,
            ),
            "left": SectorReading(
                name="left",
                angle_range=(247.5, 337.5),
                min_distance=random.uniform(0.3, 2.0),
                safety_level=SafetyLevel.CLEAR,
            ),
        }

        return DetectionResult(sectors=sectors)


def create_lidar():
    """Create LiDAR instance based on available driver."""
    try:
        from pathfinder.lidar_ld19 import LD19Lidar
        lidar = LD19Lidar()
        logger.info("Using LD19 LiDAR driver")
        return lidar
    except ImportError:
        pass

    try:
        from pathfinder.lidar import RPLidar
        lidar = RPLidar()
        logger.info("Using RPLidar driver")
        return lidar
    except ImportError:
        pass

    raise RuntimeError("No LiDAR driver available")


def create_behavior(behavior_name: str, forward_speed: float, turn_speed: float):
    """Create behavior instance by name."""
    from pathfinder.behaviors import (
        NaturalWanderBehavior,
        MaxClearanceBehavior,
        WallFollowerBehavior,
        RandomWanderBehavior,
        AvoidAndGoBehavior,
        SafetyWrapper,
        DynamicObstacleMonitor,
        create_safe_wanderer,
        WallSide,
    )

    if behavior_name == "safe_wanderer":
        return create_safe_wanderer(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
    elif behavior_name == "natural_wander":
        inner = NaturalWanderBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "max_clearance":
        inner = MaxClearanceBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        # Wrap with safety
        return SafetyWrapper(inner)
    elif behavior_name == "wall_follower_right":
        inner = WallFollowerBehavior(
            wall_side=WallSide.RIGHT,
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "wall_follower_left":
        inner = WallFollowerBehavior(
            wall_side=WallSide.LEFT,
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "random_wander":
        inner = RandomWanderBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    elif behavior_name == "avoid_and_go":
        inner = AvoidAndGoBehavior(
            forward_speed=forward_speed,
            turn_speed=turn_speed,
        )
        return SafetyWrapper(inner)
    else:
        raise ValueError(f"Unknown behavior: {behavior_name}")


def run_wandering_demo(
    simulate: bool = False,
    behavior_name: str = "safe_wanderer",
    forward_speed: float = 0.4,
    turn_speed: float = 0.5,
    duration: float = 0,
    output_file: str = None,
):
    """
    Run the wandering demo.

    Args:
        simulate: If True, don't use real motors
        behavior_name: Name of behavior to use
        forward_speed: Forward speed (0.0-1.0)
        turn_speed: Turn speed (0.0-1.0)
        duration: Run duration in seconds (0 = forever)
        output_file: File to write results to
    """
    from pathfinder.obstacle_detector import SectorBasedDetector
    from pathfinder.behaviors import BehaviorRunner

    print("=" * 60)
    print("AMBOT WANDERING DEMO")
    print("=" * 60)
    print(f"Mode: {'SIMULATION' if simulate else 'REAL HARDWARE'}")
    print(f"Behavior: {behavior_name}")
    print(f"Speeds: forward={forward_speed}, turn={turn_speed}")
    print(f"Duration: {duration}s" if duration > 0 else "Duration: until Ctrl+C")
    print("=" * 60)

    robot = None
    robot_adapter = None
    lidar = None
    start_time = time.time()

    try:
        # Create robot adapter
        if not simulate:
            try:
                from locomotion.rpi_motors.factory import create_robot
                from locomotion.rpi_motors.drivers import DriverType

                logger.info("Creating motor controller...")
                robot = create_robot(driver_type=DriverType.L298N)
                robot_adapter = RobotAdapter(robot=robot, simulate=False)
                logger.info("Motor controller ready")
            except Exception as e:
                logger.warning(f"Failed to create motors: {e}")
                logger.info("Falling back to simulation mode")
                simulate = True

        if simulate:
            robot_adapter = RobotAdapter(robot=None, simulate=True)
            print("\n[SIMULATION MODE - Commands will be printed]\n")

        # Create LiDAR
        logger.info("Connecting to LiDAR...")
        lidar = create_lidar()
        if not lidar.connect():
            raise RuntimeError("Failed to connect to LiDAR")
        logger.info("LiDAR connected")

        # Create detector and behavior
        detector = SectorBasedDetector()
        behavior = create_behavior(behavior_name, forward_speed, turn_speed)

        # Create runner
        runner = BehaviorRunner(
            lidar=lidar,
            detector=detector,
            behavior=behavior,
            robot=robot_adapter,
            loop_rate=10.0,  # 10 Hz
        )

        # Add status callback
        scan_count = [0]
        def on_scan(detection):
            scan_count[0] += 1
            if scan_count[0] % 50 == 0:  # Every 5 seconds at 10Hz
                elapsed = time.time() - start_time
                print(f"\n[{elapsed:.0f}s] Scans: {scan_count[0]}, "
                      f"Safety: {detection.overall_safety.name}, "
                      f"Closest: {detection.closest_distance:.2f}m")

        runner.on_scan(on_scan)

        print("\nStarting wandering loop... (Ctrl+C to stop)\n")

        # Run with optional duration limit
        if duration > 0:
            import threading
            timer = threading.Timer(duration, runner.stop)
            timer.start()

        runner.run()

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")

    except Exception as e:
        logger.error(f"Error: {e}")
        raise

    finally:
        # Cleanup
        elapsed = time.time() - start_time
        print(f"\n\nRan for {elapsed:.1f} seconds")

        if robot_adapter:
            stats = robot_adapter.get_stats()
            print(f"Total commands: {stats['command_count']}")
            robot_adapter.stop()
            robot_adapter.cleanup()

        if lidar:
            lidar.disconnect()

        # Save results
        if output_file:
            results = {
                "timestamp": datetime.now().isoformat(),
                "duration_seconds": elapsed,
                "behavior": behavior_name,
                "simulate": simulate,
                "forward_speed": forward_speed,
                "turn_speed": turn_speed,
                "command_count": stats["command_count"] if robot_adapter else 0,
            }

            output_path = Path(output_file)
            output_path.parent.mkdir(parents=True, exist_ok=True)

            with open(output_path, "w") as f:
                import json
                json.dump(results, f, indent=2)

            print(f"Results saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Wandering Demo - Pathfinder + Locomotion Integration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Behaviors:
  safe_wanderer      - Natural wandering with dynamic obstacle safety (default)
  natural_wander     - Cycles through top clearance directions (natural movement)
  max_clearance      - Always move toward longest distance (may ping-pong)
  wall_follower_right - Follow wall on right side
  wall_follower_left  - Follow wall on left side
  random_wander      - Random exploration with avoidance
  avoid_and_go       - Simple reactive avoidance

Examples:
  # Simulation mode
  python3 wandering_demo_1.py --simulate

  # Real mode, slow speed
  python3 wandering_demo_1.py --forward-speed 0.3

  # Wall follower for 60 seconds
  python3 wandering_demo_1.py --behavior wall_follower_right --duration 60
        """
    )

    parser.add_argument(
        "--simulate", "-s",
        action="store_true",
        help="Simulation mode (no real motors)"
    )

    parser.add_argument(
        "--behavior", "-b",
        default="safe_wanderer",
        choices=[
            "safe_wanderer",
            "natural_wander",
            "max_clearance",
            "wall_follower_right",
            "wall_follower_left",
            "random_wander",
            "avoid_and_go",
        ],
        help="Behavior to use (default: safe_wanderer)"
    )

    parser.add_argument(
        "--forward-speed",
        type=float,
        default=0.4,
        help="Forward speed 0.0-1.0 (default: 0.4)"
    )

    parser.add_argument(
        "--turn-speed",
        type=float,
        default=0.5,
        help="Turn speed 0.0-1.0 (default: 0.5)"
    )

    parser.add_argument(
        "--duration", "-d",
        type=float,
        default=0,
        help="Run duration in seconds (0 = until Ctrl+C)"
    )

    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="Output file for results (JSON)"
    )

    args = parser.parse_args()

    run_wandering_demo(
        simulate=args.simulate,
        behavior_name=args.behavior,
        forward_speed=args.forward_speed,
        turn_speed=args.turn_speed,
        duration=args.duration,
        output_file=args.output,
    )


if __name__ == "__main__":
    main()
