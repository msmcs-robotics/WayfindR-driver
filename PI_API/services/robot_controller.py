"""
Robot Controller Service

High-level robot control interface that coordinates:
- Motor control
- Navigation
- State management
- Safety features
"""

import asyncio
import time
from typing import Optional
import math

from models.robot_state import (
    RobotState, RobotMode, MovementState,
    DriveState, MotorState, Position, Velocity
)
from services.motor_driver import MotorDriver


class RobotController:
    """
    Main robot controller.

    Provides high-level control interface for the robot, handling:
    - Movement commands (throttle, steering)
    - Rotation in place
    - Emergency stop
    - State management
    """

    def __init__(self):
        self.state = RobotState()
        self.motor_driver = MotorDriver()
        self._initialized = False
        self._command_lock = asyncio.Lock()

        # Movement parameters
        self.max_speed = 255
        self.turn_sensitivity = 0.8
        self.deadzone = 0.05

        # Safety
        self.watchdog_timeout = 0.5  # seconds
        self._last_command_time = time.time()
        self._watchdog_task: Optional[asyncio.Task] = None

    @property
    def is_connected(self) -> bool:
        """Check if robot hardware is connected."""
        return self.motor_driver.is_connected

    @property
    def uptime(self) -> float:
        """Get robot uptime in seconds."""
        return self.state.uptime

    async def initialize(self):
        """Initialize robot hardware."""
        if self._initialized:
            return

        print("Initializing robot controller...")

        # Initialize motor driver
        await self.motor_driver.initialize()

        # Start watchdog
        self._watchdog_task = asyncio.create_task(self._watchdog_loop())

        self._initialized = True
        self.state.is_connected = True
        print("Robot controller initialized")

    async def shutdown(self):
        """Shutdown robot controller."""
        if not self._initialized:
            return

        print("Shutting down robot controller...")

        # Stop watchdog
        if self._watchdog_task:
            self._watchdog_task.cancel()
            try:
                await self._watchdog_task
            except asyncio.CancelledError:
                pass

        # Stop motors
        await self.stop()

        # Shutdown motor driver
        await self.motor_driver.shutdown()

        self._initialized = False
        self.state.is_connected = False
        print("Robot controller shutdown complete")

    async def _watchdog_loop(self):
        """
        Safety watchdog - stops robot if no commands received.
        Prevents runaway if connection is lost.
        """
        while True:
            await asyncio.sleep(0.1)

            if self.state.mode == RobotMode.MANUAL:
                time_since_command = time.time() - self._last_command_time
                if time_since_command > self.watchdog_timeout:
                    if self.state.movement_state != MovementState.STOPPED:
                        print("Watchdog: No command received, stopping")
                        await self.stop()

    def get_state(self) -> RobotState:
        """Get current robot state."""
        return self.state

    async def set_movement(self, throttle: float, steering: float):
        """
        Set robot movement with throttle and steering.

        Uses differential/skid steering:
        - Throttle controls forward/backward
        - Steering reduces speed on one side to turn

        Args:
            throttle: -1.0 (backward) to 1.0 (forward)
            steering: -1.0 (left) to 1.0 (right)
        """
        async with self._command_lock:
            self._last_command_time = time.time()

            # Apply deadzone
            if abs(throttle) < self.deadzone:
                throttle = 0
            if abs(steering) < self.deadzone:
                steering = 0

            # Update state
            self.state.mode = RobotMode.MANUAL
            self.state.drive.throttle = throttle
            self.state.drive.steering = steering
            self.state.add_command(f"move({throttle:.2f}, {steering:.2f})")

            # Calculate motor speeds using differential drive
            left_speed, right_speed = self._calculate_differential(throttle, steering)

            # Update movement state
            self._update_movement_state(throttle, steering)

            # Send to motors
            await self.motor_driver.set_motors(left_speed, right_speed)

            # Update motor states
            self._update_motor_states(left_speed, right_speed)

    def _calculate_differential(self, throttle: float, steering: float) -> tuple:
        """
        Calculate left and right motor speeds for differential/skid steering.

        Returns:
            (left_speed, right_speed) as floats from -1.0 to 1.0
        """
        # Base speed from throttle
        left_speed = throttle
        right_speed = throttle

        # Apply steering with turn sensitivity
        if steering != 0:
            steer_amount = abs(steering) * self.turn_sensitivity

            if abs(throttle) < 0.1:
                # Pivot turn when stopped/slow
                if steering > 0:
                    # Turn right - left forward, right backward
                    left_speed = steer_amount
                    right_speed = -steer_amount
                else:
                    # Turn left - right forward, left backward
                    left_speed = -steer_amount
                    right_speed = steer_amount
            else:
                # Normal turn - reduce inner wheel speed
                if steering > 0:
                    # Turn right - reduce right speed
                    right_speed = throttle - steer_amount
                    # Clamp to prevent direction reversal during forward motion
                    if throttle > 0:
                        right_speed = max(0, right_speed)
                    else:
                        right_speed = min(0, right_speed)
                else:
                    # Turn left - reduce left speed
                    left_speed = throttle - steer_amount
                    if throttle > 0:
                        left_speed = max(0, left_speed)
                    else:
                        left_speed = min(0, left_speed)

        # Clamp to valid range
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        return left_speed, right_speed

    def _update_movement_state(self, throttle: float, steering: float):
        """Update the movement state based on control inputs."""
        if throttle == 0 and steering == 0:
            self.state.movement_state = MovementState.STOPPED
        elif throttle > 0 and steering == 0:
            self.state.movement_state = MovementState.FORWARD
        elif throttle < 0 and steering == 0:
            self.state.movement_state = MovementState.BACKWARD
        elif throttle == 0 and steering > 0:
            self.state.movement_state = MovementState.ROTATING_RIGHT
        elif throttle == 0 and steering < 0:
            self.state.movement_state = MovementState.ROTATING_LEFT
        elif throttle > 0 and steering > 0:
            self.state.movement_state = MovementState.CURVE_FORWARD_RIGHT
        elif throttle > 0 and steering < 0:
            self.state.movement_state = MovementState.CURVE_FORWARD_LEFT
        elif throttle < 0 and steering > 0:
            self.state.movement_state = MovementState.CURVE_BACKWARD_RIGHT
        elif throttle < 0 and steering < 0:
            self.state.movement_state = MovementState.CURVE_BACKWARD_LEFT

    def _update_motor_states(self, left_speed: float, right_speed: float):
        """Update motor state objects."""
        def speed_to_motor_state(speed: float) -> MotorState:
            if abs(speed) < 0.01:
                return MotorState(speed=0, direction="stopped", pwm=0)

            direction = "forward" if speed > 0 else "backward"
            pwm = int(abs(speed) * 255)
            return MotorState(
                speed=int(speed * 255),
                direction=direction,
                pwm=pwm
            )

        # All left motors same speed
        left_state = speed_to_motor_state(left_speed)
        self.state.drive.left_front = left_state
        self.state.drive.left_rear = left_state

        # All right motors same speed
        right_state = speed_to_motor_state(right_speed)
        self.state.drive.right_front = right_state
        self.state.drive.right_rear = right_state

    async def rotate(self, speed: float):
        """
        Rotate in place.

        Args:
            speed: -1.0 (counter-clockwise) to 1.0 (clockwise)
        """
        async with self._command_lock:
            self._last_command_time = time.time()
            self.state.mode = RobotMode.MANUAL
            self.state.add_command(f"rotate({speed:.2f})")

            # Rotate by driving wheels in opposite directions
            if speed > 0:
                # Clockwise - left forward, right backward
                await self.motor_driver.set_motors(abs(speed), -abs(speed))
                self.state.movement_state = MovementState.ROTATING_RIGHT
            else:
                # Counter-clockwise - right forward, left backward
                await self.motor_driver.set_motors(-abs(speed), abs(speed))
                self.state.movement_state = MovementState.ROTATING_LEFT

            self._update_motor_states(
                speed if speed > 0 else -abs(speed),
                -speed if speed > 0 else abs(speed)
            )

    async def stop(self):
        """Stop all motors."""
        async with self._command_lock:
            self.state.add_command("stop")
            self.state.movement_state = MovementState.STOPPED
            self.state.drive.throttle = 0
            self.state.drive.steering = 0

            await self.motor_driver.stop()

            # Update motor states
            self._update_motor_states(0, 0)

    async def emergency_stop(self):
        """Emergency stop - immediate halt with braking."""
        async with self._command_lock:
            print("EMERGENCY STOP ACTIVATED")
            self.state.mode = RobotMode.EMERGENCY
            self.state.add_command("EMERGENCY_STOP")
            self.state.movement_state = MovementState.STOPPED
            self.state.drive.throttle = 0
            self.state.drive.steering = 0

            await self.motor_driver.emergency_stop()

            # Update motor states
            self._update_motor_states(0, 0)

    async def move_for_duration(self, throttle: float, steering: float, duration: float):
        """
        Move for a specific duration then stop.

        Args:
            throttle: -1.0 to 1.0
            steering: -1.0 to 1.0
            duration: seconds
        """
        await self.set_movement(throttle, steering)
        await asyncio.sleep(duration)
        await self.stop()

    async def rotate_angle(self, angle_degrees: float, speed: float = 0.5):
        """
        Rotate by a specific angle.

        Note: Without encoders/IMU, this is approximate based on timing.

        Args:
            angle_degrees: Degrees to rotate (positive = clockwise)
            speed: Rotation speed (0.0 to 1.0)
        """
        # Approximate time per 360 degrees at full speed
        # This needs calibration for your specific robot
        time_per_360 = 3.0  # seconds at full speed

        # Calculate duration
        duration = abs(angle_degrees) / 360 * time_per_360 / speed

        # Rotate
        direction = 1.0 if angle_degrees > 0 else -1.0
        await self.rotate(direction * speed)
        await asyncio.sleep(duration)
        await self.stop()
