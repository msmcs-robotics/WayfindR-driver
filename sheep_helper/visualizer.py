"""
Visualization Module
Handles drawing boxes and overlays on frames
"""

import cv2


class Visualizer:
    # Colors in BGR format
    GREEN = (0, 255, 0)      # Face detection box
    YELLOW = (0, 255, 255)   # Segmentation border box
    WHITE = (255, 255, 255)  # Text

    def __init__(self, face_box_thickness=2, border_box_thickness=2):
        """
        Initialize visualizer

        Args:
            face_box_thickness: Line thickness for face boxes
            border_box_thickness: Line thickness for border boxes
        """
        self.face_box_thickness = face_box_thickness
        self.border_box_thickness = border_box_thickness

    def draw_face_box(self, frame, x, y, w, h):
        """
        Draw green box around detected face

        Args:
            frame: Image to draw on
            x, y, w, h: Bounding box coordinates
        """
        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            self.GREEN,
            self.face_box_thickness
        )

    def draw_border_box(self, frame, x, y, w, h, frame_width, frame_height):
        """
        Draw yellow box for segmentation border (with bounds checking)

        Args:
            frame: Image to draw on
            x, y, w, h: Bounding box coordinates (may be negative or exceed frame)
            frame_width, frame_height: Frame dimensions for clamping
        """
        # Clamp coordinates to frame bounds
        x1 = max(0, x)
        y1 = max(0, y)
        x2 = min(frame_width - 1, x + w)
        y2 = min(frame_height - 1, y + h)

        cv2.rectangle(
            frame,
            (x1, y1),
            (x2, y2),
            self.YELLOW,
            self.border_box_thickness
        )

    def draw_info(self, frame, face_count, fps=None):
        """
        Draw info text on frame

        Args:
            frame: Image to draw on
            face_count: Number of faces detected
            fps: Frames per second (optional)
        """
        text = f"Faces: {face_count}"
        if fps is not None:
            text += f" | FPS: {fps:.1f}"

        cv2.putText(
            frame,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            self.WHITE,
            2
        )

    def draw_instructions(self, frame):
        """
        Draw keyboard instructions at bottom of frame
        """
        h = frame.shape[0]
        cv2.putText(
            frame,
            "Press 'q' to quit | 's' to save screenshot",
            (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            self.WHITE,
            1
        )
