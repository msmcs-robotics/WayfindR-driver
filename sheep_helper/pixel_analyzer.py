"""
Pixel Analyzer Module
Analyzes cropped face regions for dark vs light pixel ratio
Used to detect potential necrotic (dark/black) tissue in sheep
"""

import cv2
import numpy as np


class PixelAnalyzer:
    # Case classifications
    CASE_HEALTHY = 1      # More light pixels - healthy tissue
    CASE_ALERT = 2        # More dark pixels - potential necrotic tissue
    CASE_UNCERTAIN = 0    # Equal or unable to determine

    def __init__(self, darkness_threshold=100, light_threshold=150):
        """
        Initialize pixel analyzer

        Args:
            darkness_threshold: Grayscale value below which pixels are "dark"
                               (potential necrotic tissue appears black)
            light_threshold: Grayscale value above which pixels are "light"
                            (healthy shaved skin appears lighter)
        """
        self.darkness_threshold = darkness_threshold
        self.light_threshold = light_threshold

    def analyze_region(self, image, border_percent=0.10):
        """
        Analyze a cropped face region for dark vs light pixels

        Args:
            image: BGR image (cropped face region)
            border_percent: Additional border to include (default 10%)

        Returns:
            dict with analysis results:
                - case: CASE_HEALTHY (1), CASE_ALERT (2), or CASE_UNCERTAIN (0)
                - dark_ratio: Percentage of dark pixels (0.0 to 1.0)
                - light_ratio: Percentage of light pixels (0.0 to 1.0)
                - dark_count: Number of dark pixels
                - light_count: Number of light pixels
                - total_pixels: Total pixels analyzed
                - mean_brightness: Average grayscale value
        """
        if image is None or image.size == 0:
            return {
                "case": self.CASE_UNCERTAIN,
                "dark_ratio": 0.0,
                "light_ratio": 0.0,
                "dark_count": 0,
                "light_count": 0,
                "total_pixels": 0,
                "mean_brightness": 0
            }

        # Convert to grayscale for analysis
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        total_pixels = gray.size

        # Count dark pixels (potential necrotic tissue - appears black)
        dark_mask = gray < self.darkness_threshold
        dark_count = np.sum(dark_mask)

        # Count light pixels (healthy tissue - appears lighter)
        light_mask = gray > self.light_threshold
        light_count = np.sum(light_mask)

        # Calculate ratios
        dark_ratio = dark_count / total_pixels if total_pixels > 0 else 0
        light_ratio = light_count / total_pixels if total_pixels > 0 else 0

        # Mean brightness
        mean_brightness = np.mean(gray)

        # Determine case
        if dark_count > light_count:
            case = self.CASE_ALERT  # More dark = potential issue
        elif light_count > dark_count:
            case = self.CASE_HEALTHY  # More light = healthy
        else:
            case = self.CASE_UNCERTAIN

        return {
            "case": case,
            "dark_ratio": dark_ratio,
            "light_ratio": light_ratio,
            "dark_count": int(dark_count),
            "light_count": int(light_count),
            "total_pixels": int(total_pixels),
            "mean_brightness": float(mean_brightness)
        }

    def get_case_name(self, case):
        """
        Get human-readable case name

        Args:
            case: Case number (0, 1, or 2)

        Returns:
            String description
        """
        if case == self.CASE_HEALTHY:
            return "HEALTHY"
        elif case == self.CASE_ALERT:
            return "ALERT"
        else:
            return "UNCERTAIN"

    def crop_with_border(self, frame, x, y, w, h, border_percent=0.10):
        """
        Crop a region from frame with additional border

        Args:
            frame: Full frame image
            x, y, w, h: Detection bounding box
            border_percent: Percentage to expand (0.10 = 10%)

        Returns:
            Cropped image with border, clamped to frame bounds
        """
        frame_h, frame_w = frame.shape[:2]

        # Calculate expanded region
        border_w = int(w * border_percent)
        border_h = int(h * border_percent)

        x1 = max(0, x - border_w)
        y1 = max(0, y - border_h)
        x2 = min(frame_w, x + w + border_w)
        y2 = min(frame_h, y + h + border_h)

        return frame[y1:y2, x1:x2].copy()


# Colors for visualization
CASE_COLORS = {
    PixelAnalyzer.CASE_HEALTHY: (0, 255, 0),    # Green - healthy
    PixelAnalyzer.CASE_ALERT: (0, 0, 255),       # Red - alert
    PixelAnalyzer.CASE_UNCERTAIN: (0, 255, 255)  # Yellow - uncertain
}
