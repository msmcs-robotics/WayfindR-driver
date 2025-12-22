"""
Motor Driver Service

Low-level motor control using GPIO on Raspberry Pi.
Supports L298N motor drivers with 4-motor skid steer configuration.

Hardware Configuration:
    Left L298N:
        - IN1/IN2: Left Front Motor
        - IN3/IN4: Left Rear Motor
        - ENA/ENB: Jumpered (always enabled)

    Right L298N:
        - IN1/IN2: Right Front Motor
        - IN3/IN4: Right Rear Motor
        - ENA/ENB: Jumpered (always enabled)
"""

import asyncio
from typing import Optional
import os

# Try to import GPIO - will fail on non-Pi systems
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    print("Warning: RPi.GPIO not available - running in simulation mode")


class MotorDriver:
    """
    Low-level motor driver for L298N-based skid steer.

    Uses PWM for speed control and direction pins for forward/reverse.
    """

    # Default GPIO pin configuration
    # Adjust these to match your wiring
    DEFAULT_PINS = {
        # Left motors
        "left_front_pwm": 12,    # PWM capable pin
        "left_front_dir": 16,
        "left_rear_pwm": 13,     # PWM capable pin
        "left_rear_dir": 19,

        # Right motors
        "right_front_pwm": 18,   # PWM capable pin
        "right_front_dir": 23,
        "right_rear_pwm": 21,    # PWM capable pin (GPIO21)
        "right_rear_dir": 24,
    }

    # PWM frequency
    PWM_FREQ = 1000  # Hz

    def __init__(self, pins: Optional[dict] = None):
        """
        Initialize motor driver.

        Args:
            pins: Optional GPIO pin configuration dict
        """
        self.pins = pins or self.DEFAULT_PINS
        self._initialized = False
        self._pwm_objects = {}
        self._simulation_mode = not GPIO_AVAILABLE

        # Current motor states (-1.0 to 1.0)
        self._left_speed = 0.0
        self._right_speed = 0.0

    @property
    def is_connected(self) -> bool:
        """Check if GPIO is available and initialized."""
        return self._initialized

    async def initialize(self):
        """Initialize GPIO pins for motor control."""
        if self._initialized:
            return

        if self._simulation_mode:
            print("Motor driver: Running in SIMULATION mode")
            self._initialized = True
            return

        print("Initializing GPIO for motor control...")

        # Use BCM pin numbering
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Setup all pins
        for pin_name, pin_num in self.pins.items():
            GPIO.setup(pin_num, GPIO.OUT)

            if "pwm" in pin_name:
                # Create PWM object
                pwm = GPIO.PWM(pin_num, self.PWM_FREQ)
                pwm.start(0)
                self._pwm_objects[pin_name] = pwm
            else:
                # Direction pin - set LOW initially
                GPIO.output(pin_num, GPIO.LOW)

        self._initialized = True
        print("GPIO initialized successfully")

    async def shutdown(self):
        """Cleanup GPIO resources."""
        if not self._initialized:
            return

        if not self._simulation_mode:
            # Stop all PWM
            for pwm in self._pwm_objects.values():
                pwm.stop()

            # Cleanup GPIO
            GPIO.cleanup()

        self._initialized = False
        print("GPIO cleanup complete")

    async def set_motors(self, left_speed: float, right_speed: float):
        """
        Set motor speeds.

        Args:
            left_speed: -1.0 (backward) to 1.0 (forward)
            right_speed: -1.0 (backward) to 1.0 (forward)
        """
        self._left_speed = max(-1.0, min(1.0, left_speed))
        self._right_speed = max(-1.0, min(1.0, right_speed))

        if self._simulation_mode:
            # Just log in simulation mode
            # print(f"SIM: Left={self._left_speed:.2f}, Right={self._right_speed:.2f}")
            return

        # Set left motors
        await self._set_motor_pair(
            "left_front", "left_rear",
            self._left_speed
        )

        # Set right motors
        await self._set_motor_pair(
            "right_front", "right_rear",
            self._right_speed
        )

    async def _set_motor_pair(self, front_name: str, rear_name: str, speed: float):
        """
        Set a pair of motors (front + rear on same side).

        Args:
            front_name: Name prefix for front motor pins
            rear_name: Name prefix for rear motor pins
            speed: -1.0 to 1.0
        """
        pwm_front = self._pwm_objects.get(f"{front_name}_pwm")
        pwm_rear = self._pwm_objects.get(f"{rear_name}_pwm")
        dir_front = self.pins.get(f"{front_name}_dir")
        dir_rear = self.pins.get(f"{rear_name}_dir")

        # Calculate PWM duty cycle (0-100)
        duty = abs(speed) * 100

        # Set direction
        if speed >= 0:
            # Forward
            GPIO.output(dir_front, GPIO.LOW)
            GPIO.output(dir_rear, GPIO.LOW)
        else:
            # Backward
            GPIO.output(dir_front, GPIO.HIGH)
            GPIO.output(dir_rear, GPIO.HIGH)

        # Set PWM
        if pwm_front:
            pwm_front.ChangeDutyCycle(duty)
        if pwm_rear:
            pwm_rear.ChangeDutyCycle(duty)

    async def stop(self):
        """Stop all motors."""
        self._left_speed = 0.0
        self._right_speed = 0.0

        if self._simulation_mode:
            return

        # Set all PWM to 0
        for pwm in self._pwm_objects.values():
            pwm.ChangeDutyCycle(0)

        # Set all direction pins LOW
        for pin_name, pin_num in self.pins.items():
            if "dir" in pin_name:
                GPIO.output(pin_num, GPIO.LOW)

    async def emergency_stop(self):
        """
        Emergency stop with braking.

        For L298N, briefly apply reverse PWM then stop
        to achieve faster stopping.
        """
        # Quick reverse pulse for braking effect
        if not self._simulation_mode:
            # Reverse current direction briefly
            for pwm in self._pwm_objects.values():
                pwm.ChangeDutyCycle(100)
            await asyncio.sleep(0.05)  # 50ms brake pulse

        # Then full stop
        await self.stop()

    def get_motor_speeds(self) -> tuple:
        """
        Get current motor speeds.

        Returns:
            (left_speed, right_speed) as floats -1.0 to 1.0
        """
        return (self._left_speed, self._right_speed)


# Alternative: Serial-based motor driver for Arduino
class SerialMotorDriver:
    """
    Motor driver that communicates with Arduino over serial.

    Use this if your L298N drivers are connected to an Arduino
    rather than directly to the Pi's GPIO.

    Protocol:
        Send: "M<left>,<right>\n"
        Where left/right are -255 to 255

        Receive: "OK\n" or "ERR:<message>\n"
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self._serial = None
        self._initialized = False

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    async def initialize(self):
        try:
            import serial
            self._serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )
            await asyncio.sleep(2)  # Wait for Arduino reset
            self._initialized = True
            print(f"Serial motor driver connected on {self.port}")
        except Exception as e:
            print(f"Failed to connect serial motor driver: {e}")
            self._initialized = False

    async def shutdown(self):
        if self._serial:
            self._serial.close()
        self._initialized = False

    async def set_motors(self, left_speed: float, right_speed: float):
        if not self._serial:
            return

        # Convert to -255 to 255 range
        left = int(left_speed * 255)
        right = int(right_speed * 255)

        command = f"M{left},{right}\n"
        self._serial.write(command.encode())

    async def stop(self):
        await self.set_motors(0, 0)

    async def emergency_stop(self):
        if self._serial:
            self._serial.write(b"STOP\n")
        await self.stop()
