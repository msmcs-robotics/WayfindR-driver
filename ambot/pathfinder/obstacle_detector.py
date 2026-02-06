"""
Pathfinder Obstacle Detector

Sector-based obstacle detection using LiDAR scan data.
Divides the scan into sectors and returns the closest obstacle distance per sector.
"""

import math
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

from . import config
from .lidar import ScanPoint


class SafetyLevel(Enum):
    """Safety level based on obstacle proximity."""
    CLEAR = "clear"       # No obstacles in range
    WARN = "warn"         # Obstacle in warning zone
    SLOW = "slow"         # Obstacle in slow-down zone
    STOP = "stop"         # Obstacle in stop zone


@dataclass
class SectorReading:
    """Obstacle reading for a single sector."""
    name: str
    min_distance: float     # Closest obstacle distance in meters
    avg_distance: float     # Average distance of points in sector
    point_count: int        # Number of valid points in sector
    safety_level: SafetyLevel
    angle_range: Tuple[float, float]  # Start and end angles

    @property
    def is_clear(self) -> bool:
        """Check if sector is clear of obstacles."""
        return self.safety_level == SafetyLevel.CLEAR

    @property
    def requires_stop(self) -> bool:
        """Check if obstacle requires immediate stop."""
        return self.safety_level == SafetyLevel.STOP


@dataclass
class DetectionResult:
    """Complete detection result for all sectors."""
    sectors: Dict[str, SectorReading]
    closest_sector: str
    closest_distance: float
    overall_safety: SafetyLevel
    timestamp: float
    raw_points: Optional[List] = None  # Raw scan points for fine-grained analysis

    def get_sector(self, name: str) -> Optional[SectorReading]:
        """Get reading for a specific sector."""
        return self.sectors.get(name)

    def is_path_clear(self, direction: str = "front") -> bool:
        """Check if path in given direction is clear."""
        sector = self.sectors.get(direction)
        return sector is not None and sector.safety_level in (SafetyLevel.CLEAR, SafetyLevel.WARN)


class SectorBasedDetector:
    """
    Sector-based obstacle detector.

    Divides the LiDAR scan into configurable sectors (front, left, right, back, etc.)
    and reports the closest obstacle distance for each sector.

    Usage:
        detector = SectorBasedDetector()
        result = detector.process_scan(scan_points)

        if result.overall_safety == SafetyLevel.STOP:
            robot.emergency_stop()
        elif not result.is_path_clear("front"):
            robot.slow_down()
    """

    def __init__(
        self,
        stop_distance: float = None,
        slow_distance: float = None,
        warn_distance: float = None,
        use_simple_sectors: bool = None
    ):
        """
        Initialize the detector.

        Args:
            stop_distance: Distance for immediate stop (meters)
            slow_distance: Distance to slow down (meters)
            warn_distance: Distance for warning (meters)
            use_simple_sectors: Use 4-sector mode instead of 8-sector
        """
        self.stop_distance = stop_distance or config.STOP_DISTANCE
        self.slow_distance = slow_distance or config.SLOW_DISTANCE
        self.warn_distance = warn_distance or config.WARN_DISTANCE

        use_simple = use_simple_sectors if use_simple_sectors is not None else config.USE_SIMPLE_SECTORS
        self.sectors = config.SECTORS_SIMPLE if use_simple else config.SECTORS

    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to 0-360 range."""
        while angle < 0:
            angle += 360
        while angle >= 360:
            angle -= 360
        return angle

    def _angle_in_sector(self, angle: float, sector_range: Tuple[float, float]) -> bool:
        """Check if angle is within sector range."""
        angle = self._normalize_angle(angle)
        start, end = sector_range
        start = self._normalize_angle(start)
        end = self._normalize_angle(end)

        if start <= end:
            return start <= angle <= end
        else:
            # Sector wraps around 0/360
            return angle >= start or angle <= end

    def _get_safety_level(self, distance: float) -> SafetyLevel:
        """Determine safety level based on distance."""
        if distance <= self.stop_distance:
            return SafetyLevel.STOP
        elif distance <= self.slow_distance:
            return SafetyLevel.SLOW
        elif distance <= self.warn_distance:
            return SafetyLevel.WARN
        else:
            return SafetyLevel.CLEAR

    def process_scan(self, points: List[ScanPoint], timestamp: float = None) -> DetectionResult:
        """
        Process a LiDAR scan and detect obstacles.

        Args:
            points: List of scan points from LiDAR
            timestamp: Optional timestamp for the scan

        Returns:
            DetectionResult with per-sector analysis
        """
        import time
        timestamp = timestamp or time.time()

        # Initialize sector data
        sector_points: Dict[str, List[float]] = {name: [] for name in self.sectors}

        # Assign points to sectors
        for point in points:
            if point.distance <= 0:
                continue

            distance_m = point.distance_m

            for name, angle_range in self.sectors.items():
                if self._angle_in_sector(point.angle, angle_range):
                    sector_points[name].append(distance_m)
                    break

        # Calculate sector readings
        sector_readings: Dict[str, SectorReading] = {}
        closest_sector = None
        closest_distance = float('inf')
        worst_safety = SafetyLevel.CLEAR

        for name, distances in sector_points.items():
            if distances:
                min_dist = min(distances)
                avg_dist = sum(distances) / len(distances)
            else:
                min_dist = float('inf')
                avg_dist = float('inf')

            safety = self._get_safety_level(min_dist) if distances else SafetyLevel.CLEAR

            reading = SectorReading(
                name=name,
                min_distance=min_dist,
                avg_distance=avg_dist,
                point_count=len(distances),
                safety_level=safety,
                angle_range=self.sectors[name]
            )
            sector_readings[name] = reading

            # Track closest obstacle
            if min_dist < closest_distance:
                closest_distance = min_dist
                closest_sector = name

            # Track worst safety level
            if safety.value > worst_safety.value:
                worst_safety = safety

        # Handle case where no obstacles detected
        if closest_sector is None:
            closest_sector = "front"
            closest_distance = float('inf')

        return DetectionResult(
            sectors=sector_readings,
            closest_sector=closest_sector,
            closest_distance=closest_distance,
            overall_safety=worst_safety,
            timestamp=timestamp,
            raw_points=points
        )

    def get_movement_recommendation(self, result: DetectionResult) -> Dict:
        """
        Get movement recommendation based on detection result.

        Returns:
            Dictionary with movement advice
        """
        recommendation = {
            "can_move_forward": True,
            "can_move_backward": True,
            "can_turn_left": True,
            "can_turn_right": True,
            "speed_factor": 1.0,
            "suggested_action": "continue",
            "warning": None
        }

        # Check front sector
        front = result.get_sector("front")
        if front:
            if front.safety_level == SafetyLevel.STOP:
                recommendation["can_move_forward"] = False
                recommendation["suggested_action"] = "stop"
                recommendation["warning"] = f"Obstacle at {front.min_distance:.2f}m ahead"
            elif front.safety_level == SafetyLevel.SLOW:
                recommendation["speed_factor"] = 0.5
                recommendation["suggested_action"] = "slow"
                recommendation["warning"] = f"Obstacle at {front.min_distance:.2f}m ahead"

        # Check back sector
        back = result.get_sector("back")
        if back and back.safety_level == SafetyLevel.STOP:
            recommendation["can_move_backward"] = False

        # Check side sectors for turning
        left = result.get_sector("left")
        if left and left.safety_level == SafetyLevel.STOP:
            recommendation["can_turn_left"] = False

        right = result.get_sector("right")
        if right and right.safety_level == SafetyLevel.STOP:
            recommendation["can_turn_right"] = False

        return recommendation


class ContinuousDetector:
    """
    Continuous obstacle detection with filtering and history.

    Provides smoothed readings by averaging over multiple scans,
    reducing noise from single-frame outliers.
    """

    def __init__(
        self,
        history_size: int = 5,
        **kwargs
    ):
        """
        Initialize continuous detector.

        Args:
            history_size: Number of scans to average over
            **kwargs: Arguments passed to SectorBasedDetector
        """
        self.detector = SectorBasedDetector(**kwargs)
        self.history_size = history_size
        self.history: List[DetectionResult] = []

    def update(self, points: List[ScanPoint]) -> DetectionResult:
        """
        Process new scan and return filtered result.

        Args:
            points: New scan points

        Returns:
            Filtered detection result
        """
        result = self.detector.process_scan(points)

        self.history.append(result)
        if len(self.history) > self.history_size:
            self.history.pop(0)

        # For safety, use the most restrictive (closest) reading
        return self._get_filtered_result()

    def _get_filtered_result(self) -> DetectionResult:
        """Get filtered result from history."""
        if not self.history:
            return None

        if len(self.history) == 1:
            return self.history[0]

        # Use minimum distance across history for safety
        latest = self.history[-1]
        filtered_sectors = {}

        for name in latest.sectors:
            distances = [h.sectors[name].min_distance for h in self.history
                        if name in h.sectors]
            min_dist = min(distances) if distances else float('inf')

            # Create filtered sector reading
            filtered_sectors[name] = SectorReading(
                name=name,
                min_distance=min_dist,
                avg_distance=latest.sectors[name].avg_distance,
                point_count=latest.sectors[name].point_count,
                safety_level=self.detector._get_safety_level(min_dist),
                angle_range=latest.sectors[name].angle_range
            )

        # Recalculate overall result
        closest_sector = min(filtered_sectors.items(),
                           key=lambda x: x[1].min_distance)[0]
        closest_distance = filtered_sectors[closest_sector].min_distance
        worst_safety = max(s.safety_level.value for s in filtered_sectors.values())

        return DetectionResult(
            sectors=filtered_sectors,
            closest_sector=closest_sector,
            closest_distance=closest_distance,
            overall_safety=SafetyLevel(worst_safety),
            timestamp=latest.timestamp
        )

    def reset(self):
        """Clear history."""
        self.history.clear()
