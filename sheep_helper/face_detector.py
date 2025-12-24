"""
Face Detection Module
Uses OpenCV's Haar Cascade for face detection
"""

import cv2
import os


class FaceDetector:
    def __init__(self, scale_factor=1.1, min_neighbors=5, min_size=(30, 30)):
        """
        Initialize face detector with Haar Cascade

        Args:
            scale_factor: How much image size is reduced at each scale
            min_neighbors: How many neighbors each candidate rectangle should have
            min_size: Minimum face size to detect
        """
        # Load the pre-trained Haar Cascade for face detection
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        if self.face_cascade.empty():
            raise RuntimeError(f"Failed to load cascade classifier from {cascade_path}")

        self.scale_factor = scale_factor
        self.min_neighbors = min_neighbors
        self.min_size = min_size

    def detect_faces(self, frame):
        """
        Detect faces in a frame

        Args:
            frame: BGR image from OpenCV

        Returns:
            List of (x, y, w, h) tuples for each detected face
        """
        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=self.min_size
        )

        return faces

    def get_face_with_border(self, x, y, w, h, border_percent=0.1):
        """
        Calculate expanded bounding box with border

        Args:
            x, y, w, h: Original face bounding box
            border_percent: Percentage to expand (0.1 = 10%)

        Returns:
            (x, y, w, h) of expanded bounding box
        """
        border_w = int(w * border_percent)
        border_h = int(h * border_percent)

        new_x = x - border_w
        new_y = y - border_h
        new_w = w + (2 * border_w)
        new_h = h + (2 * border_h)

        return (new_x, new_y, new_w, new_h)
