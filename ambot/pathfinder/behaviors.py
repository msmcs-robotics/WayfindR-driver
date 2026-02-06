"""
Pathfinder Navigation Behaviors

Wandering and navigation algorithms using LiDAR obstacle detection.
These behaviors work with the SectorBasedDetector to generate motor commands.

Behaviors:
    - NaturalWanderBehavior: Cycles through top-N clearance directions for natural wandering
    - MaxClearanceBehavior: Move toward the direction with longest distance (simple, may ping-pong)
    - WallFollowerBehavior: Follow walls at a set distance
    - RandomWanderBehavior: Random exploration with obstacle avoidance
    - AvoidAndGoBehavior: Simple reactive obstacle avoidance
    - DynamicObstacleMonitor: Detects sudden/fast-approaching obstacles (people walking by)
    - SafetyWrapper: Wraps any behavior with emergency stop for dynamic obstacles

Usage:
    from pathfinder.behaviors import NaturalWanderBehavior, SafetyWrapper, BehaviorRunner
    from pathfinder import RPLidar, SectorBasedDetector

    behavior = NaturalWanderBehavior()
    safe_behavior = SafetyWrapper(behavior)  # Add dynamic obstacle detection
    runner = BehaviorRunner(lidar, detector, safe_behavior, robot_interface)
    runner.run()
"""

import math
import random
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple, List, Callable, Protocol

from .obstacle_detector import DetectionResult, SafetyLevel, SectorReading
from . import config


# =============================================================================
# Motor Command Types
# =============================================================================

@dataclass
class MotorCommand:
    """
    Motor command output from behaviors.

    Can be interpreted as either:
    - Differential drive: (left_speed, right_speed) where speeds are -1.0 to 1.0
    - Twist command: (linear, angular) velocities
    """
    left_speed: float = 0.0
    right_speed: float = 0.0

    @property
    def linear(self) -> float:
        """Linear velocity (average of left and right)."""
        return (self.left_speed + self.right_speed) / 2.0

    @property
    def angular(self) -> float:
        """Angular velocity (difference between left and right)."""
        return (self.right_speed - self.left_speed) / 2.0

    @classmethod
    def from_twist(cls, linear: float, angular: float) -> "MotorCommand":
        """Create from linear and angular velocities."""
        left = linear - angular
        right = linear + angular
        return cls(left_speed=left, right_speed=right)

    @classmethod
    def stop(cls) -> "MotorCommand":
        """Create a stop command."""
        return cls(0.0, 0.0)

    @classmethod
    def forward(cls, speed: float = 1.0) -> "MotorCommand":
        """Create a forward command."""
        return cls(speed, speed)

    @classmethod
    def backward(cls, speed: float = 1.0) -> "MotorCommand":
        """Create a backward command."""
        return cls(-speed, -speed)

    @classmethod
    def turn_left(cls, speed: float = 1.0) -> "MotorCommand":
        """Create a left turn command (in place)."""
        return cls(-speed, speed)

    @classmethod
    def turn_right(cls, speed: float = 1.0) -> "MotorCommand":
        """Create a right turn command (in place)."""
        return cls(speed, -speed)


# =============================================================================
# Robot Interface Protocol
# =============================================================================

class RobotInterface(Protocol):
    """Protocol for robot motor control interface."""

    def set_motors(self, left: float, right: float) -> None:
        """Set motor speeds (-1.0 to 1.0)."""
        ...

    def stop(self) -> None:
        """Emergency stop."""
        ...


# =============================================================================
# Base Behavior Class
# =============================================================================

class Behavior(ABC):
    """
    Abstract base class for navigation behaviors.

    Each behavior takes obstacle detection results and returns motor commands.
    """

    def __init__(self, name: str = "behavior"):
        self.name = name
        self._enabled = True

    @abstractmethod
    def step(self, detection: DetectionResult) -> MotorCommand:
        """
        Execute one step of the behavior.

        Args:
            detection: Current obstacle detection result

        Returns:
            MotorCommand to execute
        """
        pass

    def reset(self) -> None:
        """Reset behavior state. Override if behavior has state."""
        pass

    @property
    def enabled(self) -> bool:
        return self._enabled

    def enable(self) -> None:
        self._enabled = True

    def disable(self) -> None:
        self._enabled = False


# =============================================================================
# MaxClearanceBehavior
# =============================================================================

class MaxClearanceBehavior(Behavior):
    """
    Move toward the direction with the longest distance reading.

    This behavior finds the sector with maximum clearance and generates
    commands to turn toward that direction. Once facing clear space,
    it moves forward.

    Parameters:
        forward_speed: Speed when moving forward (0.0-1.0)
        turn_speed: Speed when turning (0.0-1.0)
        alignment_threshold: Angle tolerance for "facing" a direction (degrees)
        min_clearance: Minimum clearance to consider moving forward (meters)
    """

    def __init__(
        self,
        forward_speed: float = 0.5,
        turn_speed: float = 0.4,
        alignment_threshold: float = 30.0,
        min_clearance: float = 0.5
    ):
        super().__init__("max_clearance")
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.alignment_threshold = alignment_threshold
        self.min_clearance = min_clearance

    def step(self, detection: DetectionResult) -> MotorCommand:
        """Find clearest direction and navigate toward it."""
        if not self._enabled:
            return MotorCommand.stop()

        # Find sector with maximum clearance
        max_sector = None
        max_distance = 0.0

        for name, sector in detection.sectors.items():
            if sector.min_distance > max_distance:
                max_distance = sector.min_distance
                max_sector = sector

        if max_sector is None:
            return MotorCommand.stop()

        # Get center angle of the clearest sector
        target_angle = self._get_sector_center(max_sector)

        # Check if front sector is the clearest and has enough clearance
        front_sector = detection.get_sector("front")
        if front_sector and front_sector.name == max_sector.name:
            if front_sector.min_distance >= self.min_clearance:
                # Already facing clear space, move forward
                speed = self._calculate_speed(front_sector.min_distance)
                return MotorCommand.forward(speed)

        # Need to turn toward the clearest direction
        turn_direction = self._calculate_turn_direction(target_angle)

        if turn_direction > 0:
            return MotorCommand.turn_right(self.turn_speed)
        elif turn_direction < 0:
            return MotorCommand.turn_left(self.turn_speed)
        else:
            # Facing roughly the right direction, move forward
            speed = self._calculate_speed(max_distance)
            return MotorCommand.forward(speed)

    def _get_sector_center(self, sector: SectorReading) -> float:
        """Get center angle of a sector in degrees."""
        start, end = sector.angle_range

        # Handle wrap-around
        if start > end:
            # Sector wraps around 0/360
            if start > 180:
                center = (start + end + 360) / 2.0
                if center >= 360:
                    center -= 360
            else:
                center = (start + end) / 2.0
        else:
            center = (start + end) / 2.0

        return center

    def _calculate_turn_direction(self, target_angle: float) -> int:
        """
        Calculate turn direction to face target angle.

        Returns: -1 for left, 1 for right, 0 if aligned
        """
        # Normalize target angle relative to front (0 degrees)
        if target_angle > 180:
            target_angle -= 360

        if abs(target_angle) < self.alignment_threshold:
            return 0
        elif target_angle > 0:
            return 1  # Turn right
        else:
            return -1  # Turn left

    def _calculate_speed(self, clearance: float) -> float:
        """Calculate forward speed based on clearance."""
        if clearance <= config.STOP_DISTANCE:
            return 0.0
        elif clearance <= config.SLOW_DISTANCE:
            # Proportional slowdown
            factor = (clearance - config.STOP_DISTANCE) / (config.SLOW_DISTANCE - config.STOP_DISTANCE)
            return self.forward_speed * 0.5 * factor
        else:
            return self.forward_speed


# =============================================================================
# WallFollowerBehavior
# =============================================================================

class WallSide(Enum):
    """Which side to follow the wall on."""
    LEFT = "left"
    RIGHT = "right"


class WallFollowerBehavior(Behavior):
    """
    Follow walls at a set distance.

    Keeps a wall on one side (configurable) at a target distance.
    Turns when wall ends or obstacle ahead.

    Parameters:
        wall_side: Which side to keep the wall on (LEFT or RIGHT)
        target_distance: Target distance from wall (meters)
        distance_tolerance: Acceptable deviation from target (meters)
        forward_speed: Speed when moving forward (0.0-1.0)
        turn_speed: Speed when turning (0.0-1.0)
        correction_gain: How aggressively to correct distance (0.0-1.0)
    """

    def __init__(
        self,
        wall_side: WallSide = WallSide.RIGHT,
        target_distance: float = 0.5,
        distance_tolerance: float = 0.15,
        forward_speed: float = 0.4,
        turn_speed: float = 0.3,
        correction_gain: float = 0.5
    ):
        super().__init__("wall_follower")
        self.wall_side = wall_side
        self.target_distance = target_distance
        self.distance_tolerance = distance_tolerance
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.correction_gain = correction_gain

        # State for wall tracking
        self._last_wall_distance = None
        self._lost_wall_counter = 0

    def reset(self) -> None:
        """Reset wall tracking state."""
        self._last_wall_distance = None
        self._lost_wall_counter = 0

    def step(self, detection: DetectionResult) -> MotorCommand:
        """Follow wall while avoiding obstacles."""
        if not self._enabled:
            return MotorCommand.stop()

        # Check for obstacle ahead
        front = detection.get_sector("front")
        if front and front.safety_level in (SafetyLevel.STOP, SafetyLevel.SLOW):
            # Obstacle ahead - turn away from wall
            if self.wall_side == WallSide.RIGHT:
                return MotorCommand.turn_left(self.turn_speed)
            else:
                return MotorCommand.turn_right(self.turn_speed)

        # Get wall-side sensor reading
        wall_sector_name = self.wall_side.value
        wall_sector = detection.get_sector(wall_sector_name)

        if wall_sector is None:
            return MotorCommand.forward(self.forward_speed * 0.5)

        wall_distance = wall_sector.min_distance

        # Check if wall is present
        if wall_distance == float('inf') or wall_distance > self.target_distance * 3:
            # Wall lost - turn toward wall to find it
            self._lost_wall_counter += 1

            if self._lost_wall_counter > 5:
                # Definitely lost wall, turn toward wall side
                if self.wall_side == WallSide.RIGHT:
                    return MotorCommand.turn_right(self.turn_speed)
                else:
                    return MotorCommand.turn_left(self.turn_speed)
            else:
                # Keep going, might just be a gap
                return MotorCommand.forward(self.forward_speed * 0.7)

        self._lost_wall_counter = 0
        self._last_wall_distance = wall_distance

        # Calculate distance error
        distance_error = wall_distance - self.target_distance

        # Check if within tolerance
        if abs(distance_error) <= self.distance_tolerance:
            # Good distance, go forward
            return MotorCommand.forward(self.forward_speed)

        # Need to correct - use proportional control
        correction = self.correction_gain * (distance_error / self.target_distance)
        correction = max(-1.0, min(1.0, correction))  # Clamp

        # Apply correction based on wall side
        if self.wall_side == WallSide.RIGHT:
            # Positive error = too far from wall = turn right
            # Negative error = too close to wall = turn left
            left_speed = self.forward_speed + correction * self.turn_speed
            right_speed = self.forward_speed - correction * self.turn_speed
        else:
            # Opposite for left wall
            left_speed = self.forward_speed - correction * self.turn_speed
            right_speed = self.forward_speed + correction * self.turn_speed

        # Clamp speeds
        left_speed = max(-1.0, min(1.0, left_speed))
        right_speed = max(-1.0, min(1.0, right_speed))

        return MotorCommand(left_speed, right_speed)


# =============================================================================
# RandomWanderBehavior
# =============================================================================

class RandomWanderBehavior(Behavior):
    """
    Random exploration with obstacle avoidance.

    Moves forward when clear, makes random turns when obstacles detected,
    and occasionally makes random turns even when clear (for exploration).

    Parameters:
        forward_speed: Speed when moving forward (0.0-1.0)
        turn_speed: Speed when turning (0.0-1.0)
        turn_probability: Chance of random turn when clear (0.0-1.0)
        min_turn_duration: Minimum turn duration (seconds)
        max_turn_duration: Maximum turn duration (seconds)
    """

    def __init__(
        self,
        forward_speed: float = 0.5,
        turn_speed: float = 0.4,
        turn_probability: float = 0.02,
        min_turn_duration: float = 0.3,
        max_turn_duration: float = 1.5
    ):
        super().__init__("random_wander")
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.turn_probability = turn_probability
        self.min_turn_duration = min_turn_duration
        self.max_turn_duration = max_turn_duration

        # State
        self._turning = False
        self._turn_direction = 0  # -1 left, 1 right
        self._turn_end_time = 0.0

    def reset(self) -> None:
        """Reset wandering state."""
        self._turning = False
        self._turn_direction = 0
        self._turn_end_time = 0.0

    def step(self, detection: DetectionResult) -> MotorCommand:
        """Wander randomly while avoiding obstacles."""
        if not self._enabled:
            return MotorCommand.stop()

        current_time = time.time()

        # Check if we're currently in a turn
        if self._turning:
            if current_time < self._turn_end_time:
                # Continue turning
                if self._turn_direction > 0:
                    return MotorCommand.turn_right(self.turn_speed)
                else:
                    return MotorCommand.turn_left(self.turn_speed)
            else:
                # Turn complete
                self._turning = False

        # Check for obstacles
        front = detection.get_sector("front")

        if front and front.safety_level == SafetyLevel.STOP:
            # Immediate obstacle - start evasive turn
            self._start_turn(detection, forced=True)
            return self._get_turn_command()

        if front and front.safety_level == SafetyLevel.SLOW:
            # Approaching obstacle - slow down and maybe turn
            if random.random() < 0.3:  # 30% chance to start turning
                self._start_turn(detection, forced=False)
                return self._get_turn_command()
            else:
                # Slow forward
                return MotorCommand.forward(self.forward_speed * 0.5)

        # Path is clear
        if random.random() < self.turn_probability:
            # Random exploration turn
            self._start_turn(detection, forced=False)
            return self._get_turn_command()

        # Move forward
        return MotorCommand.forward(self.forward_speed)

    def _start_turn(self, detection: DetectionResult, forced: bool) -> None:
        """Start a turn maneuver."""
        self._turning = True

        # Decide turn direction
        if forced:
            # Turn toward clearer side
            left = detection.get_sector("left")
            right = detection.get_sector("right")

            left_dist = left.min_distance if left else 0.0
            right_dist = right.min_distance if right else 0.0

            if left_dist > right_dist:
                self._turn_direction = -1
            elif right_dist > left_dist:
                self._turn_direction = 1
            else:
                self._turn_direction = random.choice([-1, 1])
        else:
            # Random direction
            self._turn_direction = random.choice([-1, 1])

        # Set turn duration
        duration = random.uniform(self.min_turn_duration, self.max_turn_duration)
        self._turn_end_time = time.time() + duration

    def _get_turn_command(self) -> MotorCommand:
        """Get current turn command."""
        if self._turn_direction > 0:
            return MotorCommand.turn_right(self.turn_speed)
        else:
            return MotorCommand.turn_left(self.turn_speed)


# =============================================================================
# AvoidAndGoBehavior
# =============================================================================

class AvoidAndGoBehavior(Behavior):
    """
    Simple reactive obstacle avoidance.

    Priority-based behavior:
    1. If obstacle in front: stop and turn
    2. If clear: go forward

    This is the simplest reactive behavior, good for basic obstacle avoidance.

    Parameters:
        forward_speed: Speed when moving forward (0.0-1.0)
        turn_speed: Speed when turning (0.0-1.0)
        stop_threshold: Distance to trigger stop (meters)
        slow_threshold: Distance to trigger slowdown (meters)
    """

    def __init__(
        self,
        forward_speed: float = 0.5,
        turn_speed: float = 0.4,
        stop_threshold: float = None,
        slow_threshold: float = None
    ):
        super().__init__("avoid_and_go")
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.stop_threshold = stop_threshold or config.STOP_DISTANCE
        self.slow_threshold = slow_threshold or config.SLOW_DISTANCE

        # Track turn direction for consistency
        self._last_turn_direction = 0

    def reset(self) -> None:
        """Reset behavior state."""
        self._last_turn_direction = 0

    def step(self, detection: DetectionResult) -> MotorCommand:
        """React to obstacles with priority-based actions."""
        if not self._enabled:
            return MotorCommand.stop()

        front = detection.get_sector("front")

        # Priority 1: Stop if obstacle too close
        if front and front.min_distance <= self.stop_threshold:
            # Need to turn away
            turn_dir = self._choose_turn_direction(detection)
            self._last_turn_direction = turn_dir

            if turn_dir > 0:
                return MotorCommand.turn_right(self.turn_speed)
            else:
                return MotorCommand.turn_left(self.turn_speed)

        # Priority 2: Slow down if obstacle approaching
        if front and front.min_distance <= self.slow_threshold:
            # Slow forward with slight bias toward clearer side
            turn_bias = self._calculate_turn_bias(detection)

            base_speed = self.forward_speed * 0.5
            left_speed = base_speed - turn_bias * 0.2
            right_speed = base_speed + turn_bias * 0.2

            return MotorCommand(left_speed, right_speed)

        # Priority 3: Go forward
        self._last_turn_direction = 0  # Reset turn preference
        return MotorCommand.forward(self.forward_speed)

    def _choose_turn_direction(self, detection: DetectionResult) -> int:
        """Choose which direction to turn (-1 left, 1 right)."""
        left = detection.get_sector("left")
        right = detection.get_sector("right")

        left_dist = left.min_distance if left else 0.0
        right_dist = right.min_distance if right else 0.0

        # Strong preference for clearer side
        if left_dist > right_dist * 1.2:
            return -1
        elif right_dist > left_dist * 1.2:
            return 1

        # Maintain previous turn direction for consistency
        if self._last_turn_direction != 0:
            return self._last_turn_direction

        # Default to right
        return 1

    def _calculate_turn_bias(self, detection: DetectionResult) -> float:
        """Calculate turn bias toward clearer side (-1 to 1)."""
        left = detection.get_sector("left")
        right = detection.get_sector("right")

        left_dist = left.min_distance if left else 0.0
        right_dist = right.min_distance if right else 0.0

        total = left_dist + right_dist
        if total == 0:
            return 0.0

        # Positive = bias right, negative = bias left
        return (right_dist - left_dist) / total


# =============================================================================
# NaturalWanderBehavior
# =============================================================================

@dataclass
class ClearanceTarget:
    """A target direction with its clearance distance."""
    angle: float        # Center angle in degrees (0=front, clockwise)
    clearance: float    # Minimum distance in that direction (meters)


class NaturalWanderBehavior(Behavior):
    """
    Natural-looking wandering using fine-grained clearance analysis.

    Instead of always heading toward the single longest distance (which causes
    ping-ponging), this behavior:

    1. Bins raw scan data into angular buckets for a 360° clearance profile
    2. Identifies top N clearance peaks, each separated by min_angular_separation
    3. Cycles through them sequentially (longest → 2nd longest → 3rd → ...)
    4. Switches to the next target after a time limit or when clearance drops

    This creates a more natural wandering pattern where the robot explores
    multiple directions rather than bouncing between two open areas.

    This is a pre-SLAM behavior: no map, no odometry, just reactive navigation
    with smarter direction selection.

    Parameters:
        forward_speed: Speed when moving forward (0.0-1.0)
        turn_speed: Speed when turning (0.0-1.0)
        bin_size: Angular bin size in degrees (default 10)
        num_targets: Number of clearance targets to track (default 10)
        min_angular_separation: Minimum degrees between targets (default 30)
        target_hold_time: Seconds to pursue a target before switching (default 5)
        min_clearance: Minimum clearance to move forward (meters)
        alignment_threshold: Degrees tolerance for "facing" target (default 20)
    """

    def __init__(
        self,
        forward_speed: float = 0.4,
        turn_speed: float = 0.5,
        bin_size: float = 10.0,
        num_targets: int = 10,
        min_angular_separation: float = 30.0,
        target_hold_time: float = 5.0,
        min_clearance: float = 0.5,
        alignment_threshold: float = 20.0
    ):
        super().__init__("natural_wander")
        self.forward_speed = forward_speed
        self.turn_speed = turn_speed
        self.bin_size = bin_size
        self.num_targets = num_targets
        self.min_angular_separation = min_angular_separation
        self.target_hold_time = target_hold_time
        self.min_clearance = min_clearance
        self.alignment_threshold = alignment_threshold

        # Internal state
        self._target_queue: List[ClearanceTarget] = []
        self._current_target: Optional[ClearanceTarget] = None
        self._target_start_time: float = 0.0
        self._last_refresh_time: float = 0.0

    def reset(self) -> None:
        """Reset wandering state."""
        self._target_queue = []
        self._current_target = None
        self._target_start_time = 0.0
        self._last_refresh_time = 0.0

    def step(self, detection: DetectionResult) -> MotorCommand:
        """Execute one step of natural wandering."""
        if not self._enabled:
            return MotorCommand.stop()

        current_time = time.time()

        # Build clearance profile from raw scan data
        clearance_bins = self._build_clearance_profile(detection)

        # Check if we need a new target
        need_new_target = (
            self._current_target is None
            or (current_time - self._target_start_time) > self.target_hold_time
            or len(self._target_queue) == 0
        )

        if need_new_target:
            self._refresh_targets(clearance_bins, current_time)

        if self._current_target is None:
            # No viable targets - fall back to basic avoidance
            return self._fallback_step(detection)

        # Navigate toward current target
        target_angle = self._current_target.angle

        # Check front clearance for safety
        front = detection.get_sector("front")
        if front and front.safety_level == SafetyLevel.STOP:
            # Obstacle too close ahead - need to turn, advance to next target
            self._advance_target(current_time)
            if self._current_target:
                target_angle = self._current_target.angle
            else:
                return self._fallback_step(detection)

        # Calculate turn direction to face target
        turn_direction = self._calculate_turn_direction(target_angle)

        if turn_direction == 0:
            # Aligned with target - move forward
            speed = self._calculate_speed(detection)
            if speed <= 0:
                # Can't move forward even though aligned - advance target
                self._advance_target(current_time)
                return MotorCommand.stop()
            return MotorCommand.forward(speed)
        elif turn_direction > 0:
            return MotorCommand.turn_right(self.turn_speed)
        else:
            return MotorCommand.turn_left(self.turn_speed)

    def _build_clearance_profile(self, detection: DetectionResult) -> List[Tuple[float, float]]:
        """
        Build a 360° clearance profile from raw scan data.

        Returns list of (center_angle, min_distance_meters) for each angular bin.
        """
        num_bins = int(360 / self.bin_size)
        bins = [[] for _ in range(num_bins)]

        if detection.raw_points:
            # Use raw scan points for fine-grained analysis
            for point in detection.raw_points:
                if point.distance <= 0:
                    continue
                angle = point.angle % 360
                bin_idx = int(angle / self.bin_size) % num_bins
                bins[bin_idx].append(point.distance_m)
        else:
            # Fall back to sector data (coarser)
            for name, sector in detection.sectors.items():
                center = self._get_sector_center_angle(sector.angle_range)
                bin_idx = int(center / self.bin_size) % num_bins
                if sector.min_distance < float('inf'):
                    bins[bin_idx].append(sector.min_distance)

        # Calculate min distance per bin
        result = []
        for i in range(num_bins):
            center_angle = (i * self.bin_size) + (self.bin_size / 2.0)
            if bins[i]:
                min_dist = min(bins[i])
            else:
                # No data in this bin - treat as unknown (moderate distance)
                min_dist = config.WARN_DISTANCE
            result.append((center_angle, min_dist))

        return result

    def _refresh_targets(self, clearance_bins: List[Tuple[float, float]], current_time: float) -> None:
        """
        Pick top N clearance targets from the profile, separated by min_angular_separation.
        """
        # Sort bins by clearance (longest first)
        sorted_bins = sorted(clearance_bins, key=lambda b: b[1], reverse=True)

        # Greedily pick targets that are far enough apart
        targets = []
        for angle, clearance in sorted_bins:
            if clearance < self.min_clearance:
                continue  # Skip directions that are too close

            # Check angular separation from already-picked targets
            too_close = False
            for existing in targets:
                angular_dist = self._angular_distance(angle, existing.angle)
                if angular_dist < self.min_angular_separation:
                    too_close = True
                    break

            if not too_close:
                targets.append(ClearanceTarget(angle=angle, clearance=clearance))
                if len(targets) >= self.num_targets:
                    break

        self._target_queue = targets
        self._last_refresh_time = current_time

        # Set current target to first in queue
        if self._target_queue:
            self._current_target = self._target_queue.pop(0)
            self._target_start_time = current_time
        else:
            self._current_target = None

    def _advance_target(self, current_time: float) -> None:
        """Move to the next target in the queue."""
        if self._target_queue:
            self._current_target = self._target_queue.pop(0)
            self._target_start_time = current_time
        else:
            self._current_target = None

    def _angular_distance(self, a: float, b: float) -> float:
        """Calculate minimum angular distance between two angles (0-180)."""
        diff = abs(a - b) % 360
        return min(diff, 360 - diff)

    def _get_sector_center_angle(self, angle_range: Tuple[float, float]) -> float:
        """Get center angle of a sector range."""
        start, end = angle_range
        if start < 0:
            start += 360
        if end < 0:
            end += 360
        if start > end:
            center = (start + end + 360) / 2.0
            if center >= 360:
                center -= 360
        else:
            center = (start + end) / 2.0
        return center % 360

    def _calculate_turn_direction(self, target_angle: float) -> int:
        """
        Calculate turn direction to face target angle.
        Returns: -1 for left, 1 for right, 0 if aligned.
        """
        # Normalize to -180..180 relative to front (0°)
        relative = target_angle
        if relative > 180:
            relative -= 360

        if abs(relative) < self.alignment_threshold:
            return 0
        elif relative > 0:
            return 1  # Turn right
        else:
            return -1  # Turn left

    def _calculate_speed(self, detection: DetectionResult) -> float:
        """Calculate forward speed based on front clearance."""
        front = detection.get_sector("front")
        if not front:
            return self.forward_speed * 0.5

        if front.min_distance <= config.STOP_DISTANCE:
            return 0.0
        elif front.min_distance <= config.SLOW_DISTANCE:
            factor = (front.min_distance - config.STOP_DISTANCE) / (config.SLOW_DISTANCE - config.STOP_DISTANCE)
            return self.forward_speed * 0.5 * factor
        else:
            return self.forward_speed

    def _fallback_step(self, detection: DetectionResult) -> MotorCommand:
        """Fallback when no targets available - turn toward clearest sector."""
        best_sector = None
        best_distance = 0

        for name, sector in detection.sectors.items():
            if sector.min_distance > best_distance:
                best_distance = sector.min_distance
                best_sector = sector

        if best_sector is None:
            return MotorCommand.stop()

        center = self._get_sector_center_angle(best_sector.angle_range)
        turn_dir = self._calculate_turn_direction(center)

        if turn_dir > 0:
            return MotorCommand.turn_right(self.turn_speed)
        elif turn_dir < 0:
            return MotorCommand.turn_left(self.turn_speed)
        else:
            return MotorCommand.forward(self.forward_speed * 0.5)


# =============================================================================
# BehaviorRunner
# =============================================================================

class BehaviorRunner:
    """
    Runs a behavior loop with a robot interface.

    Takes care of:
    - Getting LiDAR scans
    - Processing through detector
    - Running behavior step
    - Sending commands to robot
    - Graceful shutdown

    Usage:
        from pathfinder import RPLidar, SectorBasedDetector
        from pathfinder.behaviors import MaxClearanceBehavior, BehaviorRunner

        lidar = RPLidar()
        detector = SectorBasedDetector()
        behavior = MaxClearanceBehavior()

        runner = BehaviorRunner(lidar, detector, behavior, robot)
        runner.run()  # Blocking
    """

    def __init__(
        self,
        lidar,
        detector,
        behavior: Behavior,
        robot: RobotInterface,
        loop_rate: float = 10.0
    ):
        """
        Initialize the behavior runner.

        Args:
            lidar: RPLidar instance
            detector: SectorBasedDetector or ContinuousDetector instance
            behavior: Behavior instance to run
            robot: Robot interface for motor control
            loop_rate: Target loop rate in Hz
        """
        self.lidar = lidar
        self.detector = detector
        self.behavior = behavior
        self.robot = robot
        self.loop_rate = loop_rate
        self.loop_period = 1.0 / loop_rate

        self._running = False
        self._paused = False

        # Callbacks
        self._on_scan_callback: Optional[Callable] = None
        self._on_command_callback: Optional[Callable] = None

    def set_behavior(self, behavior: Behavior) -> None:
        """Change the active behavior."""
        if self.behavior:
            self.behavior.reset()
        self.behavior = behavior
        self.behavior.reset()

    def on_scan(self, callback: Callable[[DetectionResult], None]) -> None:
        """Register callback for each scan processed."""
        self._on_scan_callback = callback

    def on_command(self, callback: Callable[[MotorCommand], None]) -> None:
        """Register callback for each command generated."""
        self._on_command_callback = callback

    def pause(self) -> None:
        """Pause behavior execution (robot stops)."""
        self._paused = True
        self.robot.stop()

    def resume(self) -> None:
        """Resume behavior execution."""
        self._paused = False

    def stop(self) -> None:
        """Stop the behavior loop."""
        self._running = False
        self.robot.stop()

    def run(self) -> None:
        """
        Run the behavior loop (blocking).

        Continues until stop() is called or an error occurs.
        """
        self._running = True
        self.behavior.reset()

        try:
            # Connect to LiDAR if not connected
            if not self.lidar.is_connected:
                self.lidar.connect()

            # Main loop - iterate over scans
            for scan in self.lidar.iter_scans():
                if not self._running:
                    break

                loop_start = time.time()

                # Process scan through detector
                detection = self.detector.process_scan(scan)

                if self._on_scan_callback:
                    self._on_scan_callback(detection)

                # Generate command if not paused
                if self._paused:
                    command = MotorCommand.stop()
                else:
                    command = self.behavior.step(detection)

                if self._on_command_callback:
                    self._on_command_callback(command)

                # Send to robot
                self.robot.set_motors(command.left_speed, command.right_speed)

                # Rate limiting
                elapsed = time.time() - loop_start
                if elapsed < self.loop_period:
                    time.sleep(self.loop_period - elapsed)

        except KeyboardInterrupt:
            pass

        finally:
            self._running = False
            self.robot.stop()

    def run_once(self, scan_points: List) -> MotorCommand:
        """
        Run a single step with provided scan points.

        Useful for testing or external loop control.

        Args:
            scan_points: List of ScanPoint from LiDAR

        Returns:
            MotorCommand generated by behavior
        """
        detection = self.detector.process_scan(scan_points)

        if self._on_scan_callback:
            self._on_scan_callback(detection)

        if self._paused:
            command = MotorCommand.stop()
        else:
            command = self.behavior.step(detection)

        if self._on_command_callback:
            self._on_command_callback(command)

        return command


# =============================================================================
# Utility: Behavior Selector
# =============================================================================

class BehaviorSelector:
    """
    Utility to select and switch between behaviors.

    Useful for implementing state machines or behavior arbitration.
    """

    def __init__(self):
        self.behaviors: dict[str, Behavior] = {}
        self._active_name: Optional[str] = None

    def register(self, name: str, behavior: Behavior) -> None:
        """Register a behavior with a name."""
        self.behaviors[name] = behavior

    def select(self, name: str) -> Behavior:
        """Select and return a behavior by name."""
        if name not in self.behaviors:
            raise ValueError(f"Unknown behavior: {name}")

        # Reset previous behavior
        if self._active_name and self._active_name != name:
            self.behaviors[self._active_name].reset()

        self._active_name = name
        return self.behaviors[name]

    @property
    def active(self) -> Optional[Behavior]:
        """Get the currently active behavior."""
        if self._active_name:
            return self.behaviors.get(self._active_name)
        return None

    def list_behaviors(self) -> List[str]:
        """List all registered behavior names."""
        return list(self.behaviors.keys())


# =============================================================================
# Dynamic Obstacle Detection (for people walking by)
# =============================================================================

@dataclass
class DynamicObstacle:
    """Information about a detected dynamic/sudden obstacle."""
    sector_name: str
    current_distance: float
    previous_distance: float
    approach_rate: float  # meters per second (negative = approaching)
    time_to_collision: float  # seconds until collision at current rate


class DynamicObstacleMonitor:
    """
    Monitors for sudden/fast-approaching obstacles.

    Detects when something suddenly appears close to the robot (like a person
    walking by) by tracking the rate of change of distances in each sector.

    Parameters:
        emergency_distance: Distance that triggers immediate stop (meters)
        approach_rate_threshold: Approach speed that triggers alert (m/s)
        time_to_collision_threshold: TTC that triggers alert (seconds)
        history_size: Number of samples to keep for rate calculation
    """

    def __init__(
        self,
        emergency_distance: float = 0.25,
        approach_rate_threshold: float = 0.5,  # 0.5 m/s = brisk walking
        time_to_collision_threshold: float = 1.5,
        history_size: int = 5
    ):
        self.emergency_distance = emergency_distance
        self.approach_rate_threshold = approach_rate_threshold
        self.time_to_collision_threshold = time_to_collision_threshold
        self.history_size = history_size

        # Distance history per sector: {sector_name: [(time, distance), ...]}
        self._history: dict[str, List[Tuple[float, float]]] = {}
        self._last_update = 0.0

    def reset(self) -> None:
        """Clear history."""
        self._history.clear()
        self._last_update = 0.0

    def update(self, detection: DetectionResult) -> List[DynamicObstacle]:
        """
        Update with new detection and return any dynamic obstacles found.

        Args:
            detection: Current obstacle detection result

        Returns:
            List of detected dynamic obstacles (empty if none)
        """
        current_time = time.time()
        dynamic_obstacles = []

        for sector_name, sector in detection.sectors.items():
            current_dist = sector.min_distance

            # Skip infinite distances
            if current_dist == float('inf'):
                # Clear history for this sector
                self._history.pop(sector_name, None)
                continue

            # Initialize or update history
            if sector_name not in self._history:
                self._history[sector_name] = []

            history = self._history[sector_name]
            history.append((current_time, current_dist))

            # Trim to history size
            if len(history) > self.history_size:
                history.pop(0)

            # Need at least 2 samples to calculate rate
            if len(history) < 2:
                continue

            # Calculate approach rate (negative = getting closer)
            oldest_time, oldest_dist = history[0]
            time_delta = current_time - oldest_time

            if time_delta <= 0:
                continue

            distance_change = current_dist - oldest_dist
            approach_rate = distance_change / time_delta

            # Calculate time to collision (if approaching)
            if approach_rate < 0 and current_dist > 0:
                time_to_collision = current_dist / abs(approach_rate)
            else:
                time_to_collision = float('inf')

            # Check for dynamic obstacle conditions
            is_emergency = current_dist <= self.emergency_distance
            is_fast_approach = approach_rate < -self.approach_rate_threshold
            is_imminent_collision = time_to_collision < self.time_to_collision_threshold

            if is_emergency or is_fast_approach or is_imminent_collision:
                # Get previous distance for reporting
                prev_dist = history[-2][1] if len(history) >= 2 else current_dist

                dynamic_obstacles.append(DynamicObstacle(
                    sector_name=sector_name,
                    current_distance=current_dist,
                    previous_distance=prev_dist,
                    approach_rate=approach_rate,
                    time_to_collision=time_to_collision
                ))

        self._last_update = current_time
        return dynamic_obstacles

    def get_most_urgent(self, obstacles: List[DynamicObstacle]) -> Optional[DynamicObstacle]:
        """Get the most urgent dynamic obstacle from a list."""
        if not obstacles:
            return None

        # Sort by urgency: closest first, then by TTC
        return min(obstacles, key=lambda o: (o.current_distance, o.time_to_collision))


class SafetyWrapper(Behavior):
    """
    Wraps any behavior with dynamic obstacle detection and emergency stop.

    This wrapper monitors for sudden obstacles (like people walking near the robot)
    and triggers emergency stops or evasive maneuvers when needed.

    Use this to make any behavior safe around moving people.

    Parameters:
        inner_behavior: The behavior to wrap
        monitor: DynamicObstacleMonitor instance (or creates default)
        emergency_backup_speed: Speed to back up during emergency
        emergency_turn_speed: Speed to turn during emergency evasion
        recovery_time: Time to wait after emergency before resuming (seconds)

    Usage:
        behavior = MaxClearanceBehavior()
        safe_behavior = SafetyWrapper(behavior)
        # safe_behavior will emergency stop if someone walks close to robot
    """

    def __init__(
        self,
        inner_behavior: Behavior,
        monitor: Optional[DynamicObstacleMonitor] = None,
        emergency_backup_speed: float = 0.3,
        emergency_turn_speed: float = 0.5,
        recovery_time: float = 0.5
    ):
        super().__init__(f"safe_{inner_behavior.name}")
        self.inner_behavior = inner_behavior
        self.monitor = monitor or DynamicObstacleMonitor()
        self.emergency_backup_speed = emergency_backup_speed
        self.emergency_turn_speed = emergency_turn_speed
        self.recovery_time = recovery_time

        # Emergency state
        self._in_emergency = False
        self._emergency_start_time = 0.0
        self._emergency_direction = 0  # -1 left, 1 right, 0 backup
        self._last_dynamic_obstacle: Optional[DynamicObstacle] = None

    def reset(self) -> None:
        """Reset both wrapper and inner behavior."""
        self._in_emergency = False
        self._emergency_start_time = 0.0
        self._emergency_direction = 0
        self._last_dynamic_obstacle = None
        self.monitor.reset()
        self.inner_behavior.reset()

    @property
    def last_dynamic_obstacle(self) -> Optional[DynamicObstacle]:
        """Get the last detected dynamic obstacle (for debugging/logging)."""
        return self._last_dynamic_obstacle

    def step(self, detection: DetectionResult) -> MotorCommand:
        """Execute step with dynamic obstacle checking."""
        if not self._enabled:
            return MotorCommand.stop()

        current_time = time.time()

        # Check for dynamic obstacles
        dynamic_obstacles = self.monitor.update(detection)
        most_urgent = self.monitor.get_most_urgent(dynamic_obstacles)

        if most_urgent:
            self._last_dynamic_obstacle = most_urgent

        # Handle emergency state
        if self._in_emergency:
            # Check if recovery time has passed
            if current_time - self._emergency_start_time > self.recovery_time:
                # Check if obstacle is still there
                if not most_urgent or most_urgent.current_distance > self.monitor.emergency_distance * 2:
                    # Safe to resume
                    self._in_emergency = False
                else:
                    # Still in danger, continue emergency
                    return self._emergency_action(most_urgent)
            else:
                # Still in recovery period
                return self._emergency_action(most_urgent)

        # Check for new emergency
        if most_urgent:
            # Something is approaching fast or too close!
            self._in_emergency = True
            self._emergency_start_time = current_time
            self._choose_emergency_direction(most_urgent, detection)
            return self._emergency_action(most_urgent)

        # No emergency - run inner behavior
        return self.inner_behavior.step(detection)

    def _choose_emergency_direction(
        self,
        obstacle: DynamicObstacle,
        detection: DetectionResult
    ) -> None:
        """Choose direction for emergency evasion."""
        # Prefer to turn away from the obstacle's sector
        if obstacle.sector_name == "front":
            # Obstacle ahead - choose clearer side
            left = detection.get_sector("left")
            right = detection.get_sector("right")
            left_dist = left.min_distance if left else 0
            right_dist = right.min_distance if right else 0

            if left_dist > right_dist:
                self._emergency_direction = -1  # Turn left
            else:
                self._emergency_direction = 1  # Turn right

        elif obstacle.sector_name == "left":
            self._emergency_direction = 1  # Turn right (away from left)

        elif obstacle.sector_name == "right":
            self._emergency_direction = -1  # Turn left (away from right)

        elif obstacle.sector_name == "back":
            # Something behind - move forward if possible
            front = detection.get_sector("front")
            if front and front.min_distance > config.STOP_DISTANCE:
                self._emergency_direction = 0  # Go forward
            else:
                # Pick a side
                self._emergency_direction = random.choice([-1, 1])
        else:
            # Unknown sector - back up
            self._emergency_direction = 0

    def _emergency_action(self, obstacle: Optional[DynamicObstacle]) -> MotorCommand:
        """Generate emergency evasion command."""
        if obstacle and obstacle.current_distance <= self.monitor.emergency_distance:
            # Very close - just stop!
            return MotorCommand.stop()

        if self._emergency_direction == 0:
            # Back up slowly
            return MotorCommand.backward(self.emergency_backup_speed)
        elif self._emergency_direction > 0:
            return MotorCommand.turn_right(self.emergency_turn_speed)
        else:
            return MotorCommand.turn_left(self.emergency_turn_speed)


# =============================================================================
# Convenience Functions
# =============================================================================

def create_safe_wanderer(
    forward_speed: float = 0.4,
    turn_speed: float = 0.5,
    emergency_distance: float = 0.25,
    approach_rate_threshold: float = 0.5
) -> SafetyWrapper:
    """
    Create a natural wandering behavior with dynamic obstacle safety.

    Uses NaturalWanderBehavior (cycles through top clearance directions)
    wrapped with SafetyWrapper for emergency stop around people.

    This is the recommended setup for demo robots around people.

    Args:
        forward_speed: Normal forward speed (0.0-1.0)
        turn_speed: Turn speed (0.0-1.0)
        emergency_distance: Distance that triggers emergency stop (meters)
        approach_rate_threshold: Approach speed that triggers alert (m/s)

    Returns:
        SafetyWrapper containing NaturalWanderBehavior

    Usage:
        behavior = create_safe_wanderer()
        runner = BehaviorRunner(lidar, detector, behavior, robot)
        runner.run()
    """
    inner = NaturalWanderBehavior(
        forward_speed=forward_speed,
        turn_speed=turn_speed
    )

    monitor = DynamicObstacleMonitor(
        emergency_distance=emergency_distance,
        approach_rate_threshold=approach_rate_threshold
    )

    return SafetyWrapper(inner, monitor)
