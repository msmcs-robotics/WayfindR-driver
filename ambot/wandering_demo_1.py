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
import time
from datetime import datetime
from pathlib import Path

from demos_common import RobotAdapter, create_lidar, create_behavior, setup_imu

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def run_wandering_demo(
    simulate: bool = False,
    behavior_name: str = "safe_wanderer",
    forward_speed: float = 0.4,
    turn_speed: float = 0.5,
    duration: float = 0,
    no_imu: bool = False,
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
        no_imu: If True, skip IMU setup
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
    imu = None
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

        # Setup IMU
        if not no_imu and not simulate:
            imu = setup_imu()
            if imu:
                print(f"IMU: heading={imu.heading:.1f} deg")
        elif simulate:
            logger.info("Skipping IMU in simulation mode")

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
            if imu:
                imu.update()
            if scan_count[0] % 50 == 0:  # Every 5 seconds at 10Hz
                elapsed = time.time() - start_time
                status = (f"\n[{elapsed:.0f}s] Scans: {scan_count[0]}, "
                          f"Safety: {detection.overall_safety.name}, "
                          f"Closest: {detection.closest_distance:.2f}m")
                if imu:
                    status += f", Heading: {imu.heading:.1f} deg"
                print(status)

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

        if imu:
            imu.cleanup()

        if lidar:
            lidar.disconnect()

        # Save results
        if output_file and robot_adapter:
            results = {
                "timestamp": datetime.now().isoformat(),
                "duration_seconds": elapsed,
                "behavior": behavior_name,
                "simulate": simulate,
                "forward_speed": forward_speed,
                "turn_speed": turn_speed,
                "command_count": robot_adapter.get_stats()["command_count"],
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
        "--no-imu",
        action="store_true",
        help="Disable IMU (skip gyro heading)"
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
        no_imu=args.no_imu,
        output_file=args.output,
    )


if __name__ == "__main__":
    main()
