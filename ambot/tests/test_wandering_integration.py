#!/usr/bin/env python3
"""
Test Wandering Integration - Pathfinder + Locomotion

Tests the integration between pathfinder behaviors and locomotion motor control.
Runs in simulation mode and verifies motor commands are generated correctly.

Usage:
    python3 test_wandering_integration.py
"""

import json
import sys
import time
from datetime import datetime
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

RESULTS_DIR = Path(__file__).parent / "results"


def test_robot_adapter():
    """Test the RobotAdapter speed conversion."""
    print("\n" + "=" * 60)
    print("TEST: RobotAdapter Speed Conversion")
    print("=" * 60)

    # Import after path setup
    sys.path.insert(0, str(Path(__file__).parent.parent))

    from wandering_demo import RobotAdapter

    adapter = RobotAdapter(robot=None, simulate=True, log_commands=True)

    test_cases = [
        # (left_float, right_float, expected_left_int, expected_right_int)
        (0.0, 0.0, 0, 0),
        (1.0, 1.0, 100, 100),
        (-1.0, -1.0, -100, -100),
        (0.5, 0.5, 50, 50),
        (-0.5, 0.5, -50, 50),  # Turn left
        (0.5, -0.5, 50, -50),  # Turn right
        (0.3, 0.3, 30, 30),
        (1.5, 1.5, 100, 100),  # Clamped
        (-1.5, -1.5, -100, -100),  # Clamped
    ]

    results = []
    all_passed = True

    for left_f, right_f, exp_left, exp_right in test_cases:
        # Capture the conversion
        adapter.set_motors(left_f, right_f)

        # Get last logged command
        _, actual_left, actual_right = adapter._command_log[-1]

        passed = (actual_left == exp_left and actual_right == exp_right)
        status = "PASS" if passed else "FAIL"
        all_passed = all_passed and passed

        results.append({
            "input": (left_f, right_f),
            "expected": (exp_left, exp_right),
            "actual": (actual_left, actual_right),
            "passed": passed,
        })

        print(f"  [{status}] ({left_f:+.1f}, {right_f:+.1f}) -> "
              f"expected ({exp_left:+4d}, {exp_right:+4d}), "
              f"got ({actual_left:+4d}, {actual_right:+4d})")

    print(f"\nResult: {'ALL PASSED' if all_passed else 'SOME FAILED'}")
    print()

    return all_passed, results


def test_lidar_connection():
    """Test LiDAR connection."""
    print("\n" + "=" * 60)
    print("TEST: LiDAR Connection")
    print("=" * 60)

    try:
        from pathfinder.lidar_ld19 import LD19Lidar

        lidar = LD19Lidar()
        connected = lidar.connect()

        if connected:
            print("  [PASS] LiDAR connected")

            # Get a few scans
            scan_count = 0
            point_count = 0
            start = time.time()

            for scan in lidar.iter_scans(min_points=100):
                scan_count += 1
                point_count += len(scan)
                if scan_count >= 3 or time.time() - start > 5:
                    break

            lidar.disconnect()

            print(f"  [PASS] Got {scan_count} scans with {point_count} total points")
            return True, {
                "connected": True,
                "scans": scan_count,
                "points": point_count,
            }
        else:
            print("  [FAIL] LiDAR connection failed")
            return False, {"connected": False}

    except Exception as e:
        print(f"  [FAIL] LiDAR error: {e}")
        return False, {"connected": False, "error": str(e)}


def test_obstacle_detector():
    """Test obstacle detector with LiDAR data."""
    print("\n" + "=" * 60)
    print("TEST: Obstacle Detector")
    print("=" * 60)

    try:
        from pathfinder.lidar_ld19 import LD19Lidar
        from pathfinder.obstacle_detector import SectorBasedDetector

        lidar = LD19Lidar()
        if not lidar.connect():
            print("  [SKIP] LiDAR not available")
            return None, {"skipped": True}

        detector = SectorBasedDetector()

        # Get one scan and process it
        for scan in lidar.iter_scans(min_points=100):
            detection = detector.process_scan(scan)

            print(f"  Overall safety: {detection.overall_safety.name}")
            print(f"  Closest obstacle: {detection.closest_distance:.2f}m "
                  f"in sector '{detection.closest_sector}'")
            print("  Sectors:")

            for name, sector in detection.sectors.items():
                print(f"    {name:8s}: {sector.min_distance:6.2f}m "
                      f"({sector.safety_level.name})")

            lidar.disconnect()

            print("\n  [PASS] Obstacle detection working")
            return True, {
                "overall_safety": detection.overall_safety.name,
                "closest_distance": detection.closest_distance,
                "closest_sector": detection.closest_sector,
                "sectors": {
                    name: {
                        "distance": s.min_distance,
                        "safety": s.safety_level.name,
                    }
                    for name, s in detection.sectors.items()
                }
            }

        lidar.disconnect()
        print("  [FAIL] No scans received")
        return False, {"error": "No scans"}

    except Exception as e:
        print(f"  [FAIL] Error: {e}")
        return False, {"error": str(e)}


def test_behavior_output():
    """Test behavior motor command output."""
    print("\n" + "=" * 60)
    print("TEST: Behavior Motor Commands")
    print("=" * 60)

    try:
        from pathfinder.lidar_ld19 import LD19Lidar
        from pathfinder.obstacle_detector import SectorBasedDetector
        from pathfinder.behaviors import (
            MaxClearanceBehavior,
            AvoidAndGoBehavior,
            RandomWanderBehavior,
        )

        lidar = LD19Lidar()
        if not lidar.connect():
            print("  [SKIP] LiDAR not available")
            return None, {"skipped": True}

        detector = SectorBasedDetector()
        behaviors = [
            ("MaxClearance", MaxClearanceBehavior(forward_speed=0.4, turn_speed=0.5)),
            ("AvoidAndGo", AvoidAndGoBehavior(forward_speed=0.4, turn_speed=0.5)),
            ("RandomWander", RandomWanderBehavior(forward_speed=0.4, turn_speed=0.5)),
        ]

        results = {}

        # Get one scan
        for scan in lidar.iter_scans(min_points=100):
            detection = detector.process_scan(scan)

            for name, behavior in behaviors:
                command = behavior.step(detection)
                results[name] = {
                    "left_speed": command.left_speed,
                    "right_speed": command.right_speed,
                    "linear": command.linear,
                    "angular": command.angular,
                }

                print(f"  {name:15s}: L={command.left_speed:+.2f} R={command.right_speed:+.2f} "
                      f"(linear={command.linear:+.2f}, angular={command.angular:+.2f})")

            break

        lidar.disconnect()

        print("\n  [PASS] All behaviors generating commands")
        return True, results

    except Exception as e:
        print(f"  [FAIL] Error: {e}")
        return False, {"error": str(e)}


def test_full_integration_simulation():
    """Test full integration in simulation mode."""
    print("\n" + "=" * 60)
    print("TEST: Full Integration (Simulation Mode)")
    print("=" * 60)

    try:
        from wandering_demo import RobotAdapter, create_lidar, create_behavior
        from pathfinder.obstacle_detector import SectorBasedDetector
        from pathfinder.behaviors import BehaviorRunner

        # Create components
        lidar = create_lidar()
        if not lidar.connect():
            print("  [FAIL] LiDAR connection failed")
            return False, {"error": "LiDAR connection failed"}

        detector = SectorBasedDetector()
        behavior = create_behavior("safe_wanderer", forward_speed=0.4, turn_speed=0.5)
        adapter = RobotAdapter(robot=None, simulate=True, log_commands=True)

        # Create runner
        runner = BehaviorRunner(
            lidar=lidar,
            detector=detector,
            behavior=behavior,
            robot=adapter,
            loop_rate=10.0,
        )

        # Run for a few iterations
        print("  Running 20 behavior iterations...")
        start = time.time()
        iterations = 0

        for scan in lidar.iter_scans(min_points=100):
            detection = detector.process_scan(scan)
            command = behavior.step(detection)
            adapter.set_motors(command.left_speed, command.right_speed)

            iterations += 1
            if iterations >= 20 or time.time() - start > 10:
                break

        adapter.stop()
        lidar.disconnect()

        elapsed = time.time() - start
        stats = adapter.get_stats()

        print(f"\n  Iterations: {iterations}")
        print(f"  Commands sent: {stats['command_count']}")
        print(f"  Elapsed time: {elapsed:.2f}s")
        print(f"  Command rate: {stats['command_count']/elapsed:.1f} Hz")

        # Verify commands were generated
        if stats['command_count'] > 0:
            print("\n  [PASS] Full integration working in simulation")
            return True, {
                "iterations": iterations,
                "commands": stats['command_count'],
                "elapsed": elapsed,
                "rate_hz": stats['command_count'] / elapsed,
            }
        else:
            print("\n  [FAIL] No commands generated")
            return False, {"error": "No commands generated"}

    except Exception as e:
        print(f"  [FAIL] Error: {e}")
        import traceback
        traceback.print_exc()
        return False, {"error": str(e)}


def run_all_tests():
    """Run all integration tests."""
    print("=" * 60)
    print("WANDERING INTEGRATION TEST SUITE")
    print("=" * 60)
    print(f"Time: {datetime.now().isoformat()}")
    print()

    results = {
        "timestamp": datetime.now().isoformat(),
        "tests": {},
    }

    # Run tests
    tests = [
        ("robot_adapter", test_robot_adapter),
        ("lidar_connection", test_lidar_connection),
        ("obstacle_detector", test_obstacle_detector),
        ("behavior_output", test_behavior_output),
        ("full_integration", test_full_integration_simulation),
    ]

    passed = 0
    failed = 0
    skipped = 0

    for name, test_func in tests:
        try:
            success, data = test_func()
            results["tests"][name] = {
                "success": success,
                "data": data,
            }

            if success is None:
                skipped += 1
            elif success:
                passed += 1
            else:
                failed += 1

        except Exception as e:
            results["tests"][name] = {
                "success": False,
                "error": str(e),
            }
            failed += 1

    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"  Passed:  {passed}")
    print(f"  Failed:  {failed}")
    print(f"  Skipped: {skipped}")

    results["summary"] = {
        "passed": passed,
        "failed": failed,
        "skipped": skipped,
        "total": len(tests),
    }

    # Save results
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    output_file = RESULTS_DIR / f"wandering_integration_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

    with open(output_file, "w") as f:
        json.dump(results, f, indent=2, default=str)

    print(f"\nResults saved to: {output_file}")

    return failed == 0


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
