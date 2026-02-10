#!/usr/bin/env python3
"""
Comprehensive Import and Syntax Verification Script

Verifies all Python modules in the ambot project can be imported
without syntax or import errors. Run this after deploying to verify
the codebase is functional.

Usage:
    python3 tests/verify_all_imports.py
    python3 tests/verify_all_imports.py --verbose
    python3 tests/verify_all_imports.py --json  # JSON output for automation

Exit codes:
    0 - All imports successful
    1 - One or more imports failed
"""

import sys
import os
import json
import argparse
from datetime import datetime
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

# Results directory
RESULTS_DIR = Path(__file__).parent / "results"
RESULTS_DIR.mkdir(exist_ok=True)


def test_syntax(file_path: str) -> tuple:
    """Test a Python file for syntax errors."""
    try:
        with open(file_path, 'r') as f:
            source = f.read()
        compile(source, file_path, 'exec')
        return True, None
    except SyntaxError as e:
        return False, f"SyntaxError at line {e.lineno}: {e.msg}"
    except Exception as e:
        return False, str(e)


def test_import(module_name: str) -> tuple:
    """Test importing a module."""
    try:
        __import__(module_name)
        return True, None
    except ImportError as e:
        return False, f"ImportError: {e}"
    except Exception as e:
        return False, f"{type(e).__name__}: {e}"


def find_python_files(root_dir: str) -> list:
    """Find all Python files in directory tree."""
    files = []
    for root, dirs, filenames in os.walk(root_dir):
        # Skip __pycache__ and hidden directories
        dirs[:] = [d for d in dirs if not d.startswith('.') and d != '__pycache__']
        for filename in filenames:
            if filename.endswith('.py'):
                files.append(os.path.join(root, filename))
    return files


def main():
    parser = argparse.ArgumentParser(description="Verify all Python imports")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--json", action="store_true", help="JSON output")
    parser.add_argument("--save", action="store_true", help="Save results to file")
    args = parser.parse_args()

    # Get root directory (ambot/)
    root_dir = Path(__file__).parent.parent

    results = {
        "timestamp": datetime.now().isoformat(),
        "syntax_checks": [],
        "import_checks": [],
        "summary": {
            "syntax_passed": 0,
            "syntax_failed": 0,
            "import_passed": 0,
            "import_failed": 0,
        }
    }

    if not args.json:
        print("=" * 60)
        print("AMBOT Import and Syntax Verification")
        print("=" * 60)
        print()

    # Find all Python files
    python_files = find_python_files(root_dir)

    if not args.json:
        print(f"Found {len(python_files)} Python files")
        print()
        print("--- Syntax Checks ---")

    # Syntax check all files
    for filepath in python_files:
        rel_path = os.path.relpath(filepath, root_dir)
        passed, error = test_syntax(filepath)

        result = {"file": rel_path, "passed": passed}
        if error:
            result["error"] = error

        results["syntax_checks"].append(result)

        if passed:
            results["summary"]["syntax_passed"] += 1
            if args.verbose and not args.json:
                print(f"  OK: {rel_path}")
        else:
            results["summary"]["syntax_failed"] += 1
            if not args.json:
                print(f"  FAIL: {rel_path}")
                print(f"        {error}")

    # Define modules to import test
    # Note: GPIO-dependent modules marked with (gpio) will be skipped on non-embedded systems
    modules_to_test = [
        # Pathfinder (no GPIO required)
        ("pathfinder", False),
        ("pathfinder.config", False),
        ("pathfinder.lidar", False),
        ("pathfinder.lidar_ld19", False),
        ("pathfinder.obstacle_detector", False),
        ("pathfinder.behaviors", False),
        ("pathfinder.imu", False),
        # Demos Common
        ("demos_common", False),
        ("demos_common.robot", False),
        ("demos_common.sensors", False),
        ("demos_common.behaviors", False),
        # Locomotion - rpi_motors
        ("locomotion.rpi_motors", False),
        ("locomotion.rpi_motors.config", False),
        ("locomotion.rpi_motors.drivers", True),  # GPIO-dependent
        ("locomotion.rpi_motors.factory", True),  # GPIO-dependent
        ("locomotion.rpi_motors.motor", True),  # GPIO-dependent
        ("locomotion.rpi_motors.test_motors", True),  # GPIO-dependent
        # Locomotion - yahboomg1 (all GPIO-dependent)
        ("locomotion.yahboomg1", True),
        ("locomotion.yahboomg1.config", True),
        ("locomotion.yahboomg1.motor", True),
        # Tests (some need hardware)
        ("tests.gui_camera", False),
        ("tests.gui_lidar", False),
        ("tests.gui_wandering", False),
        ("tests.gui_face_tracker", False),
        ("tests.gui_lidar_nav", False),
        ("tests.test_imu_calibrate", False),
        ("tests.test_gpio", True),  # GPIO-dependent
        ("tests.test_ld19_lidar", False),
        ("tests.test_usb_camera", False),
        ("tests.test_wandering_integration", False),
        # Main scripts
        ("live_monitor", False),
        ("wandering_demo_1", False),
        ("wandering_demo_2", False),
        ("tests.test_motors", False),
    ]

    # Check if we're on embedded hardware
    has_gpio = False
    try:
        import RPi.GPIO
        has_gpio = True
    except ImportError:
        try:
            import Jetson.GPIO
            has_gpio = True
        except ImportError:
            pass

    results["summary"]["gpio_available"] = has_gpio
    results["summary"]["import_skipped"] = 0

    if not args.json:
        print()
        print(f"--- Import Checks (GPIO available: {has_gpio}) ---")

    # Import test all modules
    for module, requires_gpio in modules_to_test:
        # Skip GPIO-dependent modules if GPIO not available
        if requires_gpio and not has_gpio:
            result = {"module": module, "passed": True, "skipped": True, "reason": "GPIO not available"}
            results["import_checks"].append(result)
            results["summary"]["import_skipped"] += 1
            if args.verbose and not args.json:
                print(f"  SKIP: {module} (requires GPIO)")
            continue

        passed, error = test_import(module)

        result = {"module": module, "passed": passed, "skipped": False}
        if error:
            result["error"] = error

        results["import_checks"].append(result)

        if passed:
            results["summary"]["import_passed"] += 1
            if args.verbose and not args.json:
                print(f"  OK: {module}")
        else:
            results["summary"]["import_failed"] += 1
            if not args.json:
                print(f"  FAIL: {module}")
                print(f"        {error}")

    # Print summary
    if args.json:
        print(json.dumps(results, indent=2))
    else:
        print()
        print("=" * 60)
        print("SUMMARY")
        print("=" * 60)
        s = results["summary"]
        print(f"Syntax checks: {s['syntax_passed']} passed, {s['syntax_failed']} failed")
        skipped_str = f", {s['import_skipped']} skipped" if s.get('import_skipped', 0) > 0 else ""
        print(f"Import checks: {s['import_passed']} passed, {s['import_failed']} failed{skipped_str}")
        if not s.get('gpio_available', False) and s.get('import_skipped', 0) > 0:
            print(f"  (GPIO modules skipped - run on RPi/Jetson to test)")

        total_failed = s["syntax_failed"] + s["import_failed"]
        if total_failed == 0:
            print()
            print("All checks PASSED!")
        else:
            print()
            print(f"WARNING: {total_failed} check(s) FAILED")

    # Save results if requested
    if args.save or args.json:
        output_file = RESULTS_DIR / f"verify_imports_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(output_file, 'w') as f:
            json.dump(results, f, indent=2)
        if not args.json:
            print(f"\nResults saved to: {output_file}")

    # Return exit code
    total_failed = results["summary"]["syntax_failed"] + results["summary"]["import_failed"]
    return 0 if total_failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
