"""
Robot adapter - bridges pathfinder float speeds to locomotion int speeds.
"""

import time


class RobotAdapter:
    """
    Adapts pathfinder's RobotInterface to locomotion's DifferentialDrive.

    Converts:
    - Float speeds (-1.0 to 1.0) -> Integer speeds (-100 to 100)
    - set_motors(left, right) -> robot.drive(left, right)
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
            print("STOP")
        elif self.robot:
            self.robot.stop()

        self._command_log.append((time.time(), 0, 0))

    def _print_command(self, left: int, right: int):
        """Print visual representation of motor command."""
        if left > 0 and right > 0:
            direction = "FWD"
        elif left < 0 and right < 0:
            direction = "REV"
        elif left < 0 and right > 0:
            direction = "LEFT"
        elif left > 0 and right < 0:
            direction = "RIGHT"
        else:
            direction = "STOP"

        left_bar = "#" * (abs(left) // 10) if left != 0 else ""
        right_bar = "#" * (abs(right) // 10) if right != 0 else ""

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
