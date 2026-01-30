#!/usr/bin/env python3
"""
YAHBOOM G1 Motor Test Script

Interactive test script for motor control.
Run directly on the Jetson Orin Nano or Raspberry Pi.

Usage:
    python3 test_motors.py
    python3 test_motors.py --speed 30
    python3 test_motors.py --interactive
"""

import sys
import time
import argparse


def check_platform():
    """Check platform and GPIO availability."""
    print("=" * 50)
    print("Motor Test - Platform Check")
    print("=" * 50)

    import platform
    print(f"Platform: {platform.system()} {platform.machine()}")

    # Try to import GPIO
    gpio_lib = None
    try:
        import Jetson.GPIO as GPIO
        gpio_lib = "Jetson.GPIO"
        print(f"GPIO Library: {gpio_lib} (Jetson)")
    except ImportError:
        try:
            import RPi.GPIO as GPIO
            gpio_lib = "RPi.GPIO"
            print(f"GPIO Library: {gpio_lib} (Raspberry Pi)")
        except ImportError:
            print("ERROR: No GPIO library found!")
            print("Install with:")
            print("  Jetson: sudo pip3 install Jetson.GPIO")
            print("  RPi:    sudo pip3 install RPi.GPIO")
            return None

    # Check if we can access GPIO
    try:
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        print("GPIO Access: OK")
        GPIO.cleanup()
        return GPIO
    except Exception as e:
        print(f"GPIO Access: FAILED - {e}")
        print("Make sure you're in the 'gpio' group:")
        print("  sudo usermod -a -G gpio $USER")
        return None


def test_basic(speed=50, duration=2.0):
    """Run basic motor test sequence."""
    from motor import create_robot, GPIO

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    print()
    print("=" * 50)
    print(f"Basic Motor Test (speed={speed}, duration={duration}s)")
    print("=" * 50)
    print()

    robot = None
    try:
        print("Creating robot...")
        robot = create_robot(setup_gpio=False)

        print("Enabling motor driver...")
        robot.enable()
        time.sleep(0.5)

        # Test sequence
        tests = [
            ("Forward", lambda: robot.forward(speed)),
            ("Stop", lambda: robot.stop()),
            ("Reverse", lambda: robot.reverse(speed)),
            ("Stop", lambda: robot.stop()),
            ("Turn Left", lambda: robot.turn_left(speed)),
            ("Turn Right", lambda: robot.turn_right(speed)),
            ("Arc Left", lambda: robot.arc_left(speed, 0.3)),
            ("Arc Right", lambda: robot.arc_right(speed, 0.3)),
            ("Stop", lambda: robot.stop()),
        ]

        for name, action in tests:
            print(f"  {name}...")
            action()
            if name == "Stop":
                time.sleep(0.5)
            else:
                time.sleep(duration)

        print()
        print("Test complete!")
        return True

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        if robot:
            print("Cleaning up...")
            robot.cleanup()
        GPIO.cleanup()


def test_individual_motors(speed=50, duration=1.0):
    """Test each motor individually."""
    from motor import Motor, GPIO

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    print()
    print("=" * 50)
    print("Individual Motor Test")
    print("=" * 50)
    print()

    # Standby pin
    STBY = 11
    GPIO.setup(STBY, GPIO.OUT, initial=GPIO.HIGH)

    try:
        # Test left motor
        print("Testing LEFT motor (pins 13, 15, 33)...")
        left = Motor(in1=13, in2=15, pwm=33)

        print("  Forward...")
        left.forward(speed)
        time.sleep(duration)

        print("  Reverse...")
        left.reverse(speed)
        time.sleep(duration)

        print("  Stop...")
        left.stop()
        left.cleanup()

        time.sleep(0.5)

        # Test right motor
        print("Testing RIGHT motor (pins 16, 18, 32)...")
        right = Motor(in1=16, in2=18, pwm=32)

        print("  Forward...")
        right.forward(speed)
        time.sleep(duration)

        print("  Reverse...")
        right.reverse(speed)
        time.sleep(duration)

        print("  Stop...")
        right.stop()
        right.cleanup()

        print()
        print("Individual motor test complete!")
        return True

    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False

    finally:
        GPIO.cleanup()


def test_interactive():
    """Interactive motor control via keyboard."""
    from motor import create_robot, GPIO

    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)

    print()
    print("=" * 50)
    print("Interactive Motor Control")
    print("=" * 50)
    print()
    print("Controls:")
    print("  w/s    - Forward/Reverse")
    print("  a/d    - Turn Left/Right")
    print("  q/e    - Arc Left/Right")
    print("  space  - Stop")
    print("  +/-    - Increase/Decrease speed")
    print("  x      - Exit")
    print()

    robot = None
    try:
        robot = create_robot(setup_gpio=False)
        robot.enable()

        speed = 50

        import sys
        import tty
        import termios

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(sys.stdin.fileno())

            while True:
                char = sys.stdin.read(1)

                if char == 'x' or char == '\x03':  # x or Ctrl+C
                    break
                elif char == 'w':
                    print(f"\rForward ({speed})   ", end="")
                    robot.forward(speed)
                elif char == 's':
                    print(f"\rReverse ({speed})   ", end="")
                    robot.reverse(speed)
                elif char == 'a':
                    print(f"\rTurn Left ({speed}) ", end="")
                    robot.turn_left(speed)
                elif char == 'd':
                    print(f"\rTurn Right ({speed})", end="")
                    robot.turn_right(speed)
                elif char == 'q':
                    print(f"\rArc Left ({speed})  ", end="")
                    robot.arc_left(speed, 0.3)
                elif char == 'e':
                    print(f"\rArc Right ({speed}) ", end="")
                    robot.arc_right(speed, 0.3)
                elif char == ' ':
                    print("\rStop               ", end="")
                    robot.stop()
                elif char == '+' or char == '=':
                    speed = min(100, speed + 10)
                    print(f"\rSpeed: {speed}       ", end="")
                elif char == '-':
                    speed = max(10, speed - 10)
                    print(f"\rSpeed: {speed}       ", end="")

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        print()
        print("Exiting...")

    except ImportError:
        print("Interactive mode requires a terminal with tty support.")
        print("Run with --basic instead.")

    except Exception as e:
        print(f"ERROR: {e}")

    finally:
        if robot:
            robot.cleanup()
        GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser(description="YAHBOOM G1 Motor Test")
    parser.add_argument("--speed", type=int, default=50, help="Motor speed (0-100)")
    parser.add_argument("--duration", type=float, default=1.5, help="Test duration per action")
    parser.add_argument("--basic", action="store_true", help="Run basic test sequence")
    parser.add_argument("--individual", action="store_true", help="Test each motor individually")
    parser.add_argument("--interactive", action="store_true", help="Interactive keyboard control")
    parser.add_argument("--check", action="store_true", help="Only check platform/GPIO")
    args = parser.parse_args()

    # Always check platform first
    gpio = check_platform()
    if gpio is None:
        sys.exit(1)

    if args.check:
        sys.exit(0)

    if args.interactive:
        test_interactive()
    elif args.individual:
        test_individual_motors(args.speed, args.duration)
    else:
        # Default to basic test
        test_basic(args.speed, args.duration)


if __name__ == "__main__":
    main()
