"""
Face Tracker Module
Compares detected faces against saved known_faces images
Assigns existing ID if match found, creates new ID if not
"""

import cv2
import numpy as np
import os
from datetime import datetime
from pathlib import Path
import json


class FaceTracker:
    def __init__(self, known_faces_folder="known_faces", base_path=None,
                 match_threshold=0.5, position_threshold=100):
        """
        Initialize face tracker

        Args:
            known_faces_folder: Folder containing saved face images
            base_path: Base path for folders (defaults to script directory)
            match_threshold: Similarity threshold for face matching (0-1)
            position_threshold: Max pixel distance for same-frame tracking
        """
        if base_path is None:
            base_path = Path(__file__).parent

        self.known_faces_path = Path(base_path) / known_faces_folder
        self.known_faces_path.mkdir(exist_ok=True)

        self.match_threshold = match_threshold
        self.position_threshold = position_threshold

        # Known faces database: {id: {'histogram': hist, 'image_path': path, ...}}
        self.known_faces_db = {}

        # Current frame tracking (for avoiding duplicate detections in same frame)
        self.frame_tracks = {}  # id -> last_center for current session

        # Load existing known faces from disk
        self._load_known_faces()

        # Get next available ID
        self.next_id = self._get_next_id()

        # Session stats
        self.session_detections = []  # List of (face_id, case, timestamp)

        print(f"Face Tracker initialized:")
        print(f"  Known faces folder: {self.known_faces_path}")
        print(f"  Loaded {len(self.known_faces_db)} known faces")
        print(f"  Next new ID: {self.next_id}")

    def _get_next_id(self):
        """Get the next available face ID"""
        max_id = 0
        for f in self.known_faces_path.glob("sheep_*.png"):
            try:
                parts = f.stem.split('_')
                if len(parts) >= 2:
                    face_id = int(parts[1])
                    max_id = max(max_id, face_id)
            except (ValueError, IndexError):
                pass
        return max_id + 1

    def _load_known_faces(self):
        """Load all known face images and compute their histograms"""
        print("Loading known faces from disk...")

        for img_path in sorted(self.known_faces_path.glob("sheep_*.png")):
            try:
                # Parse ID from filename: sheep_XXXX_timestamp.png
                parts = img_path.stem.split('_')
                if len(parts) >= 2:
                    face_id = int(parts[1])

                    # Load image and compute histogram
                    image = cv2.imread(str(img_path))
                    if image is not None:
                        hist = self._compute_histogram(image)

                        # Store in database (keep only first/best image per ID)
                        if face_id not in self.known_faces_db:
                            self.known_faces_db[face_id] = {
                                'histogram': hist,
                                'image_path': img_path,
                                'image': image
                            }
            except Exception as e:
                print(f"  Warning: Could not load {img_path}: {e}")

        print(f"  Loaded {len(self.known_faces_db)} unique face IDs")

    def _compute_histogram(self, image):
        """
        Compute color histogram for face matching

        Args:
            image: BGR face image

        Returns:
            Normalized histogram
        """
        if image is None or image.size == 0:
            return None

        # Resize to standard size for consistent comparison
        resized = cv2.resize(image, (64, 64))

        # Convert to HSV for better color matching
        hsv = cv2.cvtColor(resized, cv2.COLOR_BGR2HSV)

        # Compute histogram
        hist = cv2.calcHist([hsv], [0, 1], None, [50, 60], [0, 180, 0, 256])
        cv2.normalize(hist, hist, 0, 1, cv2.NORM_MINMAX)

        return hist

    def _compare_histograms(self, hist1, hist2):
        """
        Compare two histograms for similarity

        Returns:
            Similarity score (0-1, higher is more similar)
        """
        if hist1 is None or hist2 is None:
            return 0.0

        # Use correlation method
        score = cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)
        # Normalize to 0-1 range (correlation can be -1 to 1)
        return max(0, score)

    def _find_matching_known_face(self, histogram):
        """
        Search known faces database for a match

        Args:
            histogram: Histogram of the face to match

        Returns:
            (face_id, similarity) if match found, (None, 0) otherwise
        """
        if histogram is None:
            return None, 0

        best_match_id = None
        best_similarity = 0

        for face_id, data in self.known_faces_db.items():
            similarity = self._compare_histograms(histogram, data['histogram'])

            if similarity > best_similarity and similarity >= self.match_threshold:
                best_similarity = similarity
                best_match_id = face_id

        return best_match_id, best_similarity

    def _get_face_center(self, x, y, w, h):
        """Get center point of face bounding box"""
        return (x + w // 2, y + h // 2)

    def _distance(self, p1, p2):
        """Calculate Euclidean distance between two points"""
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def _is_same_detection(self, center, face_id):
        """Check if this is the same detection as tracked in current frame"""
        if face_id in self.frame_tracks:
            last_center = self.frame_tracks[face_id]
            if self._distance(center, last_center) < self.position_threshold:
                return True
        return False

    def process_face(self, face_image, bbox, frame):
        """
        Process a single detected face

        Args:
            face_image: Cropped face image
            bbox: (x, y, w, h) bounding box
            frame: Full frame (for saving context)

        Returns:
            dict with:
                - face_id: Assigned ID (existing or new)
                - is_new: True if this is a newly created ID
                - is_known: True if matched to existing known face
                - similarity: Match similarity score (if known)
        """
        x, y, w, h = bbox
        center = self._get_face_center(x, y, w, h)

        # Compute histogram for this face
        histogram = self._compute_histogram(face_image)

        # First, check if this matches a known face from the database
        matched_id, similarity = self._find_matching_known_face(histogram)

        if matched_id is not None:
            # Found a match in known faces!
            # Update frame tracking
            self.frame_tracks[matched_id] = center

            return {
                'face_id': matched_id,
                'is_new': False,
                'is_known': True,
                'similarity': similarity
            }

        else:
            # No match found - check if it's close to a recent detection in this session
            # (to avoid creating duplicate IDs for same face in consecutive frames)
            for tracked_id, tracked_center in self.frame_tracks.items():
                if self._distance(center, tracked_center) < self.position_threshold:
                    # Same position as recently tracked face
                    self.frame_tracks[tracked_id] = center
                    return {
                        'face_id': tracked_id,
                        'is_new': False,
                        'is_known': tracked_id in self.known_faces_db,
                        'similarity': 0.0
                    }

            # Truly new face - create new ID
            new_id = self.next_id
            self.next_id += 1

            # Save to disk
            self._save_face_image(new_id, face_image)

            # Add to known faces database
            self.known_faces_db[new_id] = {
                'histogram': histogram,
                'image_path': self.known_faces_path / f"sheep_{new_id:04d}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png",
                'image': face_image.copy()
            }

            # Track in current frame
            self.frame_tracks[new_id] = center

            return {
                'face_id': new_id,
                'is_new': True,
                'is_known': False,
                'similarity': 0.0
            }

    def _save_face_image(self, face_id, image):
        """Save cropped face image"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"sheep_{face_id:04d}_{timestamp}.png"
        filepath = self.known_faces_path / filename

        cv2.imwrite(str(filepath), image)
        print(f"Saved new face: {filename}")

    def record_detection(self, face_id, case):
        """Record a detection for session stats"""
        self.session_detections.append({
            'face_id': face_id,
            'case': case,
            'timestamp': datetime.now().isoformat()
        })

    def clear_frame_tracks(self):
        """
        Clear stale frame tracks (call periodically to allow re-detection)
        Keeps tracks for recently seen faces
        """
        # Keep tracks but could implement timeout logic here
        pass

    def get_known_face_count(self):
        """Get count of known faces in database"""
        return len(self.known_faces_db)

    def save_session_log(self):
        """Save a log of all detections from this session"""
        log_path = self.known_faces_path / f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

        # Count cases per face
        face_stats = {}
        for detection in self.session_detections:
            fid = detection['face_id']
            case = detection['case']
            if fid not in face_stats:
                face_stats[fid] = {'case1': 0, 'case2': 0, 'case0': 0}
            face_stats[fid][f'case{case}'] = face_stats[fid].get(f'case{case}', 0) + 1

        log_data = {
            'session_start': datetime.now().isoformat(),
            'total_known_faces': len(self.known_faces_db),
            'detections_this_session': len(self.session_detections),
            'face_stats': face_stats,
            'detections': self.session_detections[-100:]  # Last 100 detections
        }

        with open(log_path, 'w') as f:
            json.dump(log_data, f, indent=2)

        print(f"Session log saved: {log_path}")


# Allow testing the face tracker directly
if __name__ == "__main__":
    print("Face Tracker Test")
    print("=" * 40)

    tracker = FaceTracker()

    print(f"\nKnown faces: {tracker.get_known_face_count()}")
    print(f"Next ID: {tracker.next_id}")

    print("\nKnown face IDs:")
    for fid in sorted(tracker.known_faces_db.keys()):
        print(f"  - ID {fid}: {tracker.known_faces_db[fid]['image_path'].name}")
