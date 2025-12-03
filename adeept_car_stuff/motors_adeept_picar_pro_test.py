#!/usr/bin/env python3
"""
Adeept Robot HAT - Motor Controller (No Servo)
Tests motors without requiring I2C/servo setup

SETUP INSTRUCTIONS:
===================

1. Create virtual environment with system packages access:
   cd ~/Downloads
   python3 -m venv --system-site-packages venv

2. Activate virtual environment:
   source venv/bin/activate

3. Install required packages (if not already available):
   pip install RPi.GPIO

4. Run the script:
   python motor_only_test.py

Note: The --system-site-packages flag allows the venv to access
system-installed packages like RPi.GPIO while still keeping the
environment isolated.

Alternative - Run without venv:
   sudo python3 motor_only_test.py

WIRING:
=======
- Motors should be connected to Motor 1 and Motor 2 ports on Robot HAT
- Ensure Robot HAT power switch is ON
- Use battery power (two 18650 batteries recommended) when running motors
"""

import RPi.GPIO as GPIO
import time

# ========== MOTOR PINS (Adeept Robot HAT) ==========
MOTOR_A_EN = 4
MOTOR_B_EN = 17
MOTOR_A_PIN1 = 26
MOTOR_A_PIN2 = 21
MOTOR_B_PIN1 = 27
MOTOR_B_PIN2 = 18

# Global variables
pwm_A = None
pwm_B = None

# ========== SETUP ==========
def setup():
    """Initialize motors only"""
    global pwm_A, pwm_B
    
    print("Initializing Motors...")
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(MOTOR_A_EN, GPIO.OUT)
    GPIO.setup(MOTOR_B_EN, GPIO.OUT)
    GPIO.setup(MOTOR_A_PIN1, GPIO.OUT)
    GPIO.setup(MOTOR_A_PIN2, GPIO.OUT)
    GPIO.setup(MOTOR_B_PIN1, GPIO.OUT)
    GPIO.setup(MOTOR_B_PIN2, GPIO.OUT)
    
    motor_stop()
    
    pwm_A = GPIO.PWM(MOTOR_A_EN, 1000)
    pwm_B = GPIO.PWM(MOTOR_B_EN, 1000)
    
    print("âœ“ Motors initialized!")
    print()

# ========== MOTOR CONTROL ==========
def motor_stop():
    """Stop both motors"""
    GPIO.output(MOTOR_A_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
    GPIO.output(MOTOR_B_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
    GPIO.output(MOTOR_A_EN, GPIO.LOW)
    GPIO.output(MOTOR_B_EN, GPIO.LOW)

def motor_forward(speed=70):
    """Drive forward"""
    GPIO.output(MOTOR_A_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_A_PIN2, GPIO.HIGH)
    GPIO.output(MOTOR_B_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_B_PIN2, GPIO.HIGH)
    
    pwm_A.start(100)
    pwm_B.start(100)
    pwm_A.ChangeDutyCycle(speed)
    pwm_B.ChangeDutyCycle(speed)

def motor_backward(speed=70):
    """Drive backward"""
    GPIO.output(MOTOR_A_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
    GPIO.output(MOTOR_B_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
    
    pwm_A.start(100)
    pwm_B.start(100)
    pwm_A.ChangeDutyCycle(speed)
    pwm_B.ChangeDutyCycle(speed)

def motor_turn_left(speed=70):
    """Turn left (left motor backward, right forward)"""
    GPIO.output(MOTOR_A_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
    GPIO.output(MOTOR_B_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_B_PIN2, GPIO.HIGH)
    
    pwm_A.start(100)
    pwm_B.start(100)
    pwm_A.ChangeDutyCycle(speed)
    pwm_B.ChangeDutyCycle(speed)

def motor_turn_right(speed=70):
    """Turn right (right motor backward, left forward)"""
    GPIO.output(MOTOR_A_PIN1, GPIO.LOW)
    GPIO.output(MOTOR_A_PIN2, GPIO.HIGH)
    GPIO.output(MOTOR_B_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
    
    pwm_A.start(100)
    pwm_B.start(100)
    pwm_A.ChangeDutyCycle(speed)
    pwm_B.ChangeDutyCycle(speed)

def test_motor_A():
    """Test Motor A only"""
    print("Testing Motor A (rear right)...")
    GPIO.output(MOTOR_A_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_A_PIN2, GPIO.LOW)
    pwm_A.start(100)
    pwm_A.ChangeDutyCycle(70)
    time.sleep(2)
    motor_stop()

def test_motor_B():
    """Test Motor B only"""
    print("Testing Motor B (rear left)...")
    GPIO.output(MOTOR_B_PIN1, GPIO.HIGH)
    GPIO.output(MOTOR_B_PIN2, GPIO.LOW)
    pwm_B.start(100)
    pwm_B.ChangeDutyCycle(70)
    time.sleep(2)
    motor_stop()

# ========== DEMOS ==========
def demo_motor_test():
    """Test each motor individually"""
    print("\n=== Individual Motor Test ===\n")
    
    test_motor_A()
    time.sleep(1)
    test_motor_B()
    time.sleep(1)
    
    print("Testing both motors together...")
    motor_forward(70)
    time.sleep(2)
    motor_stop()
    
    print("âœ“ Motor test complete!\n")

def demo_basic_movements():
    """Basic movement demo"""
    print("\n=== Basic Movement Demo ===\n")
    
    print("Moving FORWARD for 2 seconds...")
    motor_forward(70)
    time.sleep(2)
    motor_stop()
    time.sleep(0.5)
    
    print("Moving BACKWARD for 2 seconds...")
    motor_backward(70)
    time.sleep(2)
    motor_stop()
    time.sleep(0.5)
    
    print("Turning LEFT for 1 second...")
    motor_turn_left(70)
    time.sleep(1)
    motor_stop()
    time.sleep(0.5)
    
    print("Turning RIGHT for 1 second...")
    motor_turn_right(70)
    time.sleep(1)
    motor_stop()
    
    print("âœ“ Basic movements complete!\n")

def demo_square_pattern():
    """Drive in a square"""
    print("\n=== Square Pattern Demo ===\n")
    
    for i in range(4):
        print(f"Side {i+1}...")
        motor_forward(70)
        time.sleep(2)
        motor_stop()
        time.sleep(0.3)
        
        motor_turn_right(70)
        time.sleep(0.8)
        motor_stop()
        time.sleep(0.5)
    
    print("âœ“ Square complete!\n")

def interactive_control():
    """Interactive WASD control"""
    print("\n=== Interactive Control ===")
    print("Commands:")
    print("  w = forward")
    print("  s = backward")
    print("  a = turn left")
    print("  d = turn right")
    print("  x = stop")
    print("  q = quit")
    print("\nPress Enter after each command\n")
    
    while True:
        cmd = input("Command: ").lower().strip()
        
        if cmd == 'w':
            print("â†’ Forward")
            motor_forward(70)
        elif cmd == 's':
            print("â†’ Backward")
            motor_backward(70)
        elif cmd == 'a':
            print("â†’ Turn left")
            motor_turn_left(70)
        elif cmd == 'd':
            print("â†’ Turn right")
            motor_turn_right(70)
        elif cmd == 'x':
            print("â†’ Stop")
            motor_stop()
        elif cmd == 'q':
            print("â†’ Quitting...")
            motor_stop()
            break
        else:
            print("âš  Invalid command")

# ========== MAIN ==========
def main():
    """Main program"""
    try:
        setup()
        
        print("\n" + "="*50)
        print("  ADEEPT ROBOT HAT - MOTOR CONTROLLER")
        print("  (No servo - I2C not required)")
        print("="*50)
        print("\nChoose a demo:")
        print("  1 - Motor Test (test each motor)")
        print("  2 - Basic Movements")
        print("  3 - Square Pattern")
        print("  4 - Interactive Control (WASD)")
        print("  5 - Run All Demos")
        print("  q - Quit")
        
        while True:
            choice = input("\nEnter choice (1-5 or q): ").strip()
            
            if choice == '1':
                demo_motor_test()
            elif choice == '2':
                demo_basic_movements()
            elif choice == '3':
                demo_square_pattern()
            elif choice == '4':
                interactive_control()
            elif choice == '5':
                demo_motor_test()
                time.sleep(1)
                demo_basic_movements()
                time.sleep(1)
                demo_square_pattern()
            elif choice.lower() == 'q':
                print("\nExiting...")
                break
            else:
                print("âš  Invalid choice")
        
    except KeyboardInterrupt:
        print("\n\nâš  Interrupted by user")
    
    finally:
        motor_stop()
        GPIO.cleanup()
        print("âœ“ Cleanup complete. Goodbye!\n")

if __name__ == "__main__":
    main()